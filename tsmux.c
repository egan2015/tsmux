

#include <pthread.h>

#include "tsmux.h"

#include "vlc_common.h"
#include "vlc_block.h"
#include "vlc_codecs.h"

#include "dvbpsi/dvbpsi.h"
#include "dvbpsi/demux.h"
#include "dvbpsi/descriptor.h"
#include "dvbpsi/pat.h"
#include "dvbpsi/pmt.h"
#include "dvbpsi/sdt.h"
#include "dvbpsi/dr.h"
#include "dvbpsi/psi.h"

#include "bits.h"
#include "pes.h"
#include "csa.h"

#define MAX_PMT 64       /* Maximum number of programs. FIXME: I just chose an arbitary number. Where is the maximum in the spec? */
#define MAX_PMT_PID 64       /* Maximum pids in each pmt.  FIXME: I just chose an arbitary number. Where is the maximum in the spec? */


typedef struct pmt_map_t   /* Holds the mapping between the pmt-pid/pmt table */
{
    int i_pid;
    unsigned long i_prog;
} pmt_map_t;

typedef struct sdt_desc_t
{
    char *psz_provider;
    char *psz_service_name;  /* name of program */
} sdt_desc_t;


typedef struct
{
    int     i_depth;
    block_t *p_first;
    block_t **pp_last;
} sout_buffer_chain_t;

static inline void BufferChainInit  ( sout_buffer_chain_t *c )
{
    c->i_depth = 0;
    c->p_first = NULL;
    c->pp_last = &c->p_first;
}

static inline void BufferChainAppend( sout_buffer_chain_t *c, block_t *b )
{
    *c->pp_last = b;
    c->i_depth++;

    while( b->p_next )
    {
        b = b->p_next;
        c->i_depth++;
    }
    c->pp_last = &b->p_next;
}

static inline block_t *BufferChainGet( sout_buffer_chain_t *c )
{
    block_t *b = c->p_first;

    if( b )
    {
        c->i_depth--;
        c->p_first = b->p_next;

        if( c->p_first == NULL )
        {
            c->pp_last = &c->p_first;
        }

        b->p_next = NULL;
    }
    return b;
}

static inline block_t *BufferChainPeek( sout_buffer_chain_t *c )
{
    block_t *b = c->p_first;

    return b;
}

static inline void BufferChainClean( sout_buffer_chain_t *c )
{
    block_t *b;

    while( ( b = BufferChainGet( c ) ) )
    {
        block_Release( b );
    }
    BufferChainInit( c );
}

struct sout_param_t{
	
};

struct sout_input_t{

	block_fifo_t   *p_fifo;
	es_format_t    *p_fmt;
	ts_stream_t    *p_sys;
};

struct ts_stream_t{

	int             i_pid;
    vlc_fourcc_t    i_codec;

    int             i_stream_type;
    int             i_stream_id;
    int             i_continuity_counter;
    bool            b_discontinuity;

    /* to be used for carriege of DIV3 */
    vlc_fourcc_t    i_bih_codec;
    int             i_bih_width, i_bih_height;

    /* Specific to mpeg4 in mpeg2ts */
    int             i_es_id;

    int             i_decoder_specific_info;
    uint8_t         *p_decoder_specific_info;

    /* language is iso639-2T */
    int             i_langs;
    uint8_t         *lang;

    sout_buffer_chain_t chain_pes;
    mtime_t             i_pes_dts;
    mtime_t             i_pes_length;
    int                 i_pes_used;
};

struct sout_mux_t{
    int             i_pcr_pid;
    
    sout_input_t    *p_pcr_input;

    pthread_mutex_t     csa_lock;

    int             i_audio_bound;
    int             i_video_bound;

    bool            b_es_id_pid;
    bool            b_sdt;
    int             i_pid_video;
    int             i_pid_audio;
    int             i_pid_spu;
    int             i_pid_free; /* first usable pid */

    int             i_tsid;
    int             i_netid;
    int             i_num_pmt;
    int             i_pmtslots;
    int             i_pat_version_number;
    ts_stream_t     pat;

    int             i_pmt_version_number;
    ts_stream_t     pmt[MAX_PMT];
    pmt_map_t       pmtmap[MAX_PMT_PID];
    int             i_pmt_program_number[MAX_PMT];
    sdt_desc_t      sdt_descriptors[MAX_PMT];
    bool            b_data_alignment;

    int             i_mpeg4_streams;

    int             i_null_continuity_counter;  /* Needed ? */
    ts_stream_t     sdt;
    dvbpsi_pmt_t    *dvbpmt;

    /* for TS building */
    int64_t         i_bitrate_min;
    int64_t         i_bitrate_max;

    int64_t         i_shaping_delay;
    int64_t         i_pcr_delay;

    int64_t         i_dts_delay;

    bool            b_use_key_frames;

    mtime_t         i_pcr;  /* last PCR emited */

    csa_t           *csa;
    int             i_csa_pkt_size;
    bool            b_crypt_audio;
    bool            b_crypt_video;
    
    // sout_mux_t 
    int            i_nb_inputs;
	sout_input_t   *pp_inputs[10];
};


/* Reserve a pid and return it */
static int  AllocatePID( sout_mux_t *p_sys, int i_cat )
{
    int i_pid;
    if ( i_cat == VIDEO_ES && p_sys->i_pid_video )
    {
        i_pid = p_sys->i_pid_video;
        p_sys->i_pid_video = 0;
    }
    else if ( i_cat == AUDIO_ES && p_sys->i_pid_audio )
    {
        i_pid = p_sys->i_pid_audio;
        p_sys->i_pid_audio = 0;
    }
    else if ( i_cat == SPU_ES && p_sys->i_pid_spu )
    {
        i_pid = p_sys->i_pid_spu;
        p_sys->i_pid_spu = 0;
    }
    else
    {
        i_pid = ++p_sys->i_pid_free;
    }
    return i_pid;
}

static int pmtcompare( const void *pa, const void *pb )
{
    if ( ((pmt_map_t *)pa)->i_pid  < ((pmt_map_t *)pb)->i_pid )
        return -1;
    else if ( ((pmt_map_t *)pa)->i_pid  > ((pmt_map_t *)pb)->i_pid )
        return 1;
    else
        return 0;
}

static int intcompare( const void *pa, const void *pb )
{
    if ( *(int *)pa  < *(int *)pb )
        return -1;
    else if ( *(int *)pa > *(int *)pb )
        return 1;
    else
        return 0;
}


static block_t *FixPES( sout_mux_t *p_mux, block_fifo_t *p_fifo );
static block_t *Add_ADTS( block_t *, es_format_t * );
static void TSSchedule  ( sout_mux_t *p_mux, sout_buffer_chain_t *p_chain_ts,
                          mtime_t i_pcr_length, mtime_t i_pcr_dts );
static void TSDate      ( sout_mux_t *p_mux, sout_buffer_chain_t *p_chain_ts,
                          mtime_t i_pcr_length, mtime_t i_pcr_dts );
static void GetPAT( sout_mux_t *p_mux, sout_buffer_chain_t *c );
static void GetPMT( sout_mux_t *p_mux, sout_buffer_chain_t *c );

static block_t *TSNew( sout_mux_t *p_mux, ts_stream_t *p_stream, bool b_pcr );
static void TSSetPCR( block_t *p_ts, mtime_t i_dts );

static void PEStoTS  ( sout_buffer_chain_t *, block_t *, ts_stream_t * );

sout_mux_t* soutMuxOpen( sout_param_t * p_param )
{
	int i;
	sout_mux_t * p_sys = malloc( sizeof(sout_mux_t));
	if( !p_sys )
        return VLC_ENOMEM;
	p_sys->i_pmtslots = p_sys->b_sdt = 0;
    p_sys->i_num_pmt = 1;
    p_sys->dvbpmt = NULL;
    memset( &p_sys->pmtmap, 0, sizeof(p_sys->pmtmap) );
    pthread_mutex_init( &p_sys->csa_lock ,NULL);

    for ( i = 0; i < MAX_PMT; i++ )
        p_sys->sdt_descriptors[i].psz_service_name
            = p_sys->sdt_descriptors[i].psz_provider = NULL;
    memset( p_sys->sdt_descriptors, 0, sizeof(sdt_desc_t) );

    p_sys->i_audio_bound = 0;
    p_sys->i_video_bound = 0;
    p_sys->b_es_id_pid = false;
    
    p_sys->i_pat_version_number = 0x90 & 0x1f;
    p_sys->pat.i_pid = 0;
    p_sys->pat.i_continuity_counter = 0;
    p_sys->pat.b_discontinuity = false;
    p_sys->i_tsid = 0;
    
    p_sys->i_netid = 0;

    p_sys->i_pmt_version_number = 0;
    for( i = 0; i < p_sys->i_num_pmt; i++ )
    {
        p_sys->pmt[i].i_continuity_counter = 0;
        p_sys->pmt[i].b_discontinuity = false;
    }

    p_sys->sdt.i_pid = 0x11;
    p_sys->sdt.i_continuity_counter = 0;
    p_sys->sdt.b_discontinuity = false;
    p_sys->b_sdt = false;
	p_sys->b_data_alignment = true;
	
	for( i = 0; i < p_sys->i_num_pmt; i++ )
		p_sys->i_pmt_program_number[i] = i + 1;
		
    for( i = 0; i < p_sys->i_num_pmt; i++ )
        p_sys->pmt[i].i_pid = 0x42 + i;		

    p_sys->i_pid_free = p_sys->pmt[p_sys->i_num_pmt - 1].i_pid + 1;

    p_sys->i_pid_video = 0;
    if ( p_sys->i_pid_video > p_sys->i_pid_free )
    {
        p_sys->i_pid_free = p_sys->i_pid_video + 1;
    }

    p_sys->i_pid_audio = 0;
    if ( p_sys->i_pid_audio > p_sys->i_pid_free )
    {
        p_sys->i_pid_free = p_sys->i_pid_audio + 1;
    }

    p_sys->i_pid_spu = 0;
    if ( p_sys->i_pid_spu > p_sys->i_pid_free )
    {
        p_sys->i_pid_free = p_sys->i_pid_spu + 1;
    }

    p_sys->i_pcr_pid = 0x1fff;

    p_sys->p_pcr_input = NULL;

    p_sys->i_mpeg4_streams = 0;

    p_sys->i_null_continuity_counter = 0;

    /* Allow to create constrained stream */
    p_sys->i_bitrate_min = 0;

    p_sys->i_bitrate_max = 0;

    if( p_sys->i_bitrate_min > 0 && p_sys->i_bitrate_max > 0 &&
        p_sys->i_bitrate_min > p_sys->i_bitrate_max )
    {
		fprintf( stderr, "incompatible minimum and maximum bitrate, "
                 "disabling bitrate control" );
        p_sys->i_bitrate_min = 0;
        p_sys->i_bitrate_max = 0;
    }
    if( p_sys->i_bitrate_min > 0 || p_sys->i_bitrate_max > 0 )
    {
        fprintf( stderr, "bmin and bmax no more supported "
                 "(if you need them report it)" );
    }

    p_sys->i_shaping_delay = (int64_t)200 * 1000;
    if( p_sys->i_shaping_delay <= 0 )
    {
        fprintf( stderr,
                 "invalid shaping (%"PRId64"ms) resetting to 200ms",
                 p_sys->i_shaping_delay / 1000 );
        p_sys->i_shaping_delay = 200000;
    }

    p_sys->i_pcr_delay = (int64_t)70 * 1000;
    if( p_sys->i_pcr_delay <= 0 ||
        p_sys->i_pcr_delay >= p_sys->i_shaping_delay )
    {
        fprintf( stderr,
                 "invalid pcr delay (%"PRId64"ms) resetting to 70ms",
                 p_sys->i_pcr_delay / 1000 );
        p_sys->i_pcr_delay = 70000;
    }

    p_sys->i_dts_delay = (int64_t)400 * 1000;

    fprintf( stderr, "shaping=%"PRId64" pcr=%"PRId64" dts_delay=%"PRId64,
             p_sys->i_shaping_delay, p_sys->i_pcr_delay, p_sys->i_dts_delay );

    p_sys->b_use_key_frames = false;

    /* for TS generation */
    p_sys->i_pcr    = 0;

    p_sys->csa      = NULL;

    p_sys->b_crypt_audio = false;

    p_sys->b_crypt_video = false;
	
	return p_sys;
}


void soutMuxClose( sout_mux_t * p_sys )
{
	
    for( int i = 0; i < MAX_PMT; i++ )
    {
        free( p_sys->sdt_descriptors[i].psz_service_name );
        free( p_sys->sdt_descriptors[i].psz_provider );
    }

    pthread_mutex_destroy( &p_sys->csa_lock );
    free( p_sys->dvbpmt );
    free( p_sys );	
}


int  soutMuxWrite( sout_mux_t * p_sys, unsigned char * p_data , uint16_t i_size,
				  int64_t i_length, int64_t i_pts, int64_t i_dts, int64_t i_flags)
{
	return 0;
}


int  soutAddStream( sout_mux_t* p_sys, es_format_t *p_fmt)
{
	return 0;
}


int  soutDelStream( sout_mux_t* p_sys, ts_stream_t * p_stream )
{
	return 0;
}
