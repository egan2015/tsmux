

#include <pthread.h>
#include <limits.h>

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

#include "dvbpsi_compat.h"


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
	sout_mux_t 	   *p_mux;
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

    int             i_extra;
    uint8_t         *p_extra;

    /* language is iso639-2T */
    int             i_langs;
    uint8_t         *lang;

    sout_buffer_chain_t chain_pes;
    mtime_t             i_pes_dts;
    mtime_t             i_pes_length;
    int                 i_pes_used;
    bool                b_key_frame;
};

struct sout_mux_t{
    int             i_pcr_pid;
    
    sout_input_t    *p_pcr_input;

    pthread_mutex_t     csa_lock;

    int             i_audio_bound;
    int             i_video_bound;
    
#if (DVBPSI_VERSION_INT >= DVBPSI_VERSION_WANTED(1,0,0))
    dvbpsi_t        *p_dvbpsi;
#endif

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
    mtime_t         first_dts;

    bool            b_use_key_frames;

    mtime_t         i_pcr;  /* last PCR emited */

    csa_t           *csa;
    int             i_csa_pkt_size;
    bool            b_crypt_audio;
    bool            b_crypt_video;
    
    void (*p_access)(void* p_private ,unsigned char * p_ts_data , size_t i_size  );
    void 		   *p_private;
    
    // sout_mux_t 
    int            i_nb_inputs;
	sout_input_t   *pp_inputs[10];
		
};

static uint32_t GetDescriptorLength24b( int i_length )
{
    uint32_t i_l1, i_l2, i_l3;

    i_l1 = i_length&0x7f;
    i_l2 = ( i_length >> 7 )&0x7f;
    i_l3 = ( i_length >> 14 )&0x7f;

    return( 0x808000 | ( i_l3 << 16 ) | ( i_l2 << 8 ) | i_l1 );
}

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


static int AddStream( sout_mux_t *, sout_input_t * );
static int DelStream( sout_mux_t *, sout_input_t * );
static int Mux      ( sout_mux_t * );

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


sout_mux_t* soutOpen( sout_param_t * p_param ,sout_ts_write_cb callback, void* p_private )
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

	p_sys->p_access = callback;
	p_sys->p_private = p_private;
	
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
                 "disabling bitrate control\n" );
        p_sys->i_bitrate_min = 0;
        p_sys->i_bitrate_max = 0;
    }
    if( p_sys->i_bitrate_min > 0 || p_sys->i_bitrate_max > 0 )
    {
        fprintf( stderr, "bmin and bmax no more supported "
                 "(if you need them report it)\n" );
    }

    p_sys->i_shaping_delay = (int64_t)200 * 1000;
    if( p_sys->i_shaping_delay <= 0 )
    {
        fprintf( stderr,
                 "invalid shaping (%"PRId64"ms) resetting to 200ms\n",
                 p_sys->i_shaping_delay / 1000 );
        p_sys->i_shaping_delay = 200000;
    }

    p_sys->i_pcr_delay = (int64_t)70 * 1000;
    if( p_sys->i_pcr_delay <= 0 ||
        p_sys->i_pcr_delay >= p_sys->i_shaping_delay )
    {
        fprintf( stderr,
                 "invalid pcr delay (%"PRId64"ms) resetting to 70ms\n",
                 p_sys->i_pcr_delay / 1000 );
        p_sys->i_pcr_delay = 70000;
    }

    p_sys->i_dts_delay = (int64_t)400 * 1000;

    fprintf( stderr, "shaping=%"PRId64" pcr=%"PRId64" dts_delay=%"PRId64"\n",
             p_sys->i_shaping_delay, p_sys->i_pcr_delay, p_sys->i_dts_delay );

    p_sys->b_use_key_frames = false;

    /* for TS generation */
    p_sys->i_pcr    = 0;

    p_sys->csa      = NULL;

    p_sys->b_crypt_audio = false;

    p_sys->b_crypt_video = false;
    
    p_sys->i_nb_inputs = 0;
	
	return p_sys;
}


void soutClose( sout_mux_t * p_sys )
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


int  sout_stream_mux( sout_input_t * p_input, unsigned char * p_es_data , uint16_t i_size,
				  int64_t i_length, int64_t i_pts, int64_t i_dts, int64_t i_flags)
{
	block_t *p_es = block_Alloc( i_size );
	memcpy( p_es->p_buffer, p_es_data, i_size );
	p_es->i_buffer = i_size;
	p_es->i_pts = i_pts;
	p_es->i_dts = i_dts;
	p_es->i_flags = i_flags;	
	block_FifoPut( p_input->p_fifo,p_es);
	
	return Mux(p_input->p_mux);
}

int  sout_block_mux(sout_input_t * p_input , block_t *p_nal )
{
	block_FifoPut( p_input->p_fifo,p_nal);
	
	return Mux(p_input->p_mux);	
}

sout_input_t * soutAddStream( sout_mux_t* p_sys, es_format_t *p_fmt)
{
	sout_input_t * p_input = malloc( sizeof( sout_input_t) ) ;
	
	p_input->p_fifo = block_FifoNew();
	
	if ( !p_input->p_fifo ){
		fprintf(stderr,"create sout input fifo error\n");
		
		free( p_input);
		return 0;
	}
	p_input->p_fmt = p_fmt;
	if ( AddStream( p_sys, p_input ) != 0 ){
		
		fprintf(stderr,"create sout input fifo error\n");
		block_FifoRelease(p_input->p_fifo);
		free(p_input);
		return 0;
	}
	p_input->p_mux = p_sys;
	p_sys->pp_inputs[p_sys->i_nb_inputs++] = p_input;
	fprintf(stderr,"i_nb_inputs :%d\n",p_sys->i_nb_inputs);
	return p_input;
}

int  soutDelStream( sout_mux_t* p_sys, sout_input_t * p_input )
{
	
	return DelStream(p_sys, p_input) ;
}

/*****************************************************************************
 * AddStream: called for each stream addition
 *****************************************************************************/
static int AddStream( sout_mux_t *p_mux, sout_input_t *p_input )
{
    sout_mux_t      *p_sys = p_mux;
    ts_stream_t     *p_stream;

    p_input->p_sys = p_stream = calloc( 1, sizeof( ts_stream_t ) );
    if( !p_stream )
        goto oom;

    if ( p_sys->b_es_id_pid )
        p_stream->i_pid = p_input->p_fmt->i_id & 0x1fff;
    else
        p_stream->i_pid = AllocatePID( p_sys, p_input->p_fmt->i_cat );

    p_stream->i_codec = p_input->p_fmt->i_codec;

    p_stream->i_stream_type = -1;
    p_stream->i_pes_length = 0;
    switch( p_input->p_fmt->i_codec )
    {
    /* VIDEO */

    case VLC_CODEC_MPGV:
    case VLC_CODEC_MP2V:
    case VLC_CODEC_MP1V:
        /* TODO: do we need to check MPEG-I/II ? */
        p_stream->i_stream_type = 0x02;
        p_stream->i_stream_id = 0xe0;
        break;
    case VLC_CODEC_MP4V:
        p_stream->i_stream_type = 0x10;
        p_stream->i_stream_id = 0xe0;
        p_stream->i_es_id = p_stream->i_pid;
        break;
    case VLC_CODEC_H264:
        p_stream->i_stream_type = 0x1b;
        p_stream->i_stream_id = 0xe0;
        break;
    /* XXX dirty dirty but somebody want crapy MS-codec XXX */
    case VLC_CODEC_H263I:
    case VLC_CODEC_H263:
    case VLC_CODEC_WMV3:
    case VLC_CODEC_WMV2:
    case VLC_CODEC_WMV1:
    case VLC_CODEC_DIV3:
    case VLC_CODEC_DIV2:
    case VLC_CODEC_DIV1:
    case VLC_CODEC_MJPG:
        p_stream->i_stream_type = 0xa0; /* private */
        p_stream->i_stream_id = 0xa0;   /* beurk */
        p_stream->i_bih_codec  = p_input->p_fmt->i_codec;
        p_stream->i_bih_width  = p_input->p_fmt->video.i_width;
        p_stream->i_bih_height = p_input->p_fmt->video.i_height;
        break;
    case VLC_CODEC_DIRAC:
        /* stream_id makes use of stream_id_extension */
        p_stream->i_stream_id = (PES_EXTENDED_STREAM_ID << 8) | 0x60;
        p_stream->i_stream_type = 0xd1;
        break;

    /* AUDIO */

    case VLC_CODEC_MPGA:
    case VLC_CODEC_MP3:
        p_stream->i_stream_type =
            p_input->p_fmt->audio.i_rate >= 32000 ? 0x03 : 0x04;
        p_stream->i_stream_id = 0xc0;
        break;
    case VLC_CODEC_A52:
        p_stream->i_stream_type = 0x81;
        p_stream->i_stream_id = 0xbd;
        break;
    case VLC_CODEC_EAC3:
        p_stream->i_stream_type = 0x06;
        p_stream->i_stream_id = 0xbd;
        break;
    case VLC_CODEC_DVD_LPCM:
        p_stream->i_stream_type = 0x83;
        p_stream->i_stream_id = 0xbd;
        break;
    case VLC_CODEC_DTS:
        p_stream->i_stream_type = 0x06;
        p_stream->i_stream_id = 0xbd;
        break;
    case VLC_CODEC_MP4A:
        /* XXX: make that configurable in some way when LOAS
         * is implemented for AAC in TS */
        //p_stream->i_stream_type = 0x11; /* LOAS/LATM */
        p_stream->i_stream_type = 0x0f; /* ADTS */
        p_stream->i_stream_id = 0xc0;
        p_stream->i_es_id = p_stream->i_pid;
        break;

    /* TEXT */

    case VLC_CODEC_SPU:
        p_stream->i_stream_type = 0x82;
        p_stream->i_stream_id = 0xbd;
        break;
    case VLC_CODEC_SUBT:
        p_stream->i_stream_type = 0x12;
        p_stream->i_stream_id = 0xfa;
        p_sys->i_mpeg4_streams++;
        p_stream->i_es_id = p_stream->i_pid;
        break;
    case VLC_CODEC_DVBS:
        p_stream->i_stream_type = 0x06;
        p_stream->i_es_id = p_input->p_fmt->subs.dvb.i_id;
        p_stream->i_stream_id = 0xbd;
        break;
    case VLC_CODEC_TELETEXT:
        p_stream->i_stream_type = 0x06;
        p_stream->i_stream_id = 0xbd; /* FIXME */
        break;
    }

    if (p_stream->i_stream_type == -1)
    {
        fprintf( stderr, "rejecting stream with unsupported codec %4.4s",
                  (char*)&p_stream->i_codec );
        free( p_stream );
        return VLC_EGENERIC;
    }

    p_stream->i_langs = 1 + p_input->p_fmt->i_extra_languages;
    p_stream->lang = calloc(1, p_stream->i_langs * 4);
    if( !p_stream->lang )
        goto oom;

    fprintf( stderr, "adding input codec=%4.4s pid=%d\n",
             (char*)&p_stream->i_codec, p_stream->i_pid );

    for (int i = 0; i < p_stream->i_langs; i++) {
        char *lang = (i == 0)
            ? p_input->p_fmt->psz_language
            : p_input->p_fmt->p_extra_languages[i-1].psz_language;

        if (!lang)
            continue;

       // const char *code = GetIso639_2LangCode(lang);
       // if (*code)
       // {
       //     memcpy(&p_stream->lang[i*4], code, 3);
       //     p_stream->lang[i*4+3] = 0x00; /* audio type: 0x00 undefined */
       //     fprintf( stderr, "    - lang=%3.3s", &p_stream->lang[i*4] );
       // }
    }

    /* Create decoder specific info for subt */
    if( p_stream->i_codec == VLC_CODEC_SUBT )
    {
        p_stream->i_extra = 55;
        p_stream->p_extra = malloc( p_stream->i_extra );
        if (!p_stream->p_extra)
            goto oom;

        uint8_t *p = p_stream->p_extra;
        p[0] = 0x10;    /* textFormat, 0x10 for 3GPP TS 26.245 */
        p[1] = 0x00;    /* flags: 1b: associated video info flag
                                3b: reserved
                                1b: duration flag
                                3b: reserved */
        p[2] = 52;      /* remaining size */

        p += 3;

        p[0] = p[1] = p[2] = p[3] = 0; p+=4;    /* display flags */
        *p++ = 0;  /* horizontal justification (-1: left, 0 center, 1 right) */
        *p++ = 1;  /* vertical   justification (-1: top, 0 center, 1 bottom) */

        p[0] = p[1] = p[2] = 0x00; p+=3;/* background rgb */
        *p++ = 0xff;                    /* background a */

        p[0] = p[1] = 0; p += 2;        /* text box top */
        p[0] = p[1] = 0; p += 2;        /* text box left */
        p[0] = p[1] = 0; p += 2;        /* text box bottom */
        p[0] = p[1] = 0; p += 2;        /* text box right */

        p[0] = p[1] = 0; p += 2;        /* start char */
        p[0] = p[1] = 0; p += 2;        /* end char */
        p[0] = p[1] = 0; p += 2;        /* default font id */

        *p++ = 0;                       /* font style flags */
        *p++ = 12;                      /* font size */

        p[0] = p[1] = p[2] = 0x00; p+=3;/* foreground rgb */
        *p++ = 0x00;                    /* foreground a */

        p[0] = p[1] = p[2] = 0; p[3] = 22; p += 4;
        memcpy( p, "ftab", 4 ); p += 4;
        *p++ = 0; *p++ = 1;             /* entry count */
        p[0] = p[1] = 0; p += 2;        /* font id */
        *p++ = 9;                       /* font name length */
        memcpy( p, "Helvetica", 9 );    /* font name */
    }
    else
    {
        /* Copy extra data (VOL for MPEG-4 and extra BitMapInfoHeader for VFW */
        es_format_t *fmt = p_input->p_fmt;
        if( fmt->i_extra > 0 )
        {
            p_stream->i_extra = fmt->i_extra;
            p_stream->p_extra = malloc( fmt->i_extra );
            if( !p_stream->p_extra )
                goto oom;

            memcpy( p_stream->p_extra, fmt->p_extra, fmt->i_extra );
        }
    }

    /* Init pes chain */
    BufferChainInit( &p_stream->chain_pes );

    /* We only change PMT version (PAT isn't changed) */
    p_sys->i_pmt_version_number = ( p_sys->i_pmt_version_number + 1 )%32;

    /* Update pcr_pid */
    if( p_input->p_fmt->i_cat != SPU_ES &&
        ( p_sys->i_pcr_pid == 0x1fff || p_input->p_fmt->i_cat == VIDEO_ES ) )
    {
        if( p_sys->p_pcr_input )
        {
            /* There was already a PCR stream, so clean context */
            /* FIXME */
        }
        p_sys->i_pcr_pid   = p_stream->i_pid;
        p_sys->p_pcr_input = p_input;

        fprintf( stderr, "new PCR PID is %d\n", p_sys->i_pcr_pid );
    }

    return VLC_SUCCESS;

oom:
    if(p_stream)
    {
        free(p_stream->lang);
        free(p_stream);
    }
    return VLC_ENOMEM;
}


/*****************************************************************************
 * DelStream: called before a stream deletion
 *****************************************************************************/
static int DelStream( sout_mux_t *p_mux, sout_input_t *p_input )
{
    sout_mux_t  *p_sys = p_mux;
    ts_stream_t     *p_stream = (ts_stream_t*)p_input->p_sys;
    int              pid = 0;

   fprintf( stderr, "removing input pid=%d\n", p_stream->i_pid );

    if( p_sys->i_pcr_pid == p_stream->i_pid )
    {
        /* Find a new pcr stream (Prefer Video Stream) */
        p_sys->i_pcr_pid = 0x1fff;
        p_sys->p_pcr_input = NULL;
        for (int i = 0; i < p_mux->i_nb_inputs; i++ )
        {
            if( p_mux->pp_inputs[i] == p_input )
            {
                continue;
            }

            if( p_mux->pp_inputs[i]->p_fmt->i_cat == VIDEO_ES )
            {
                p_sys->i_pcr_pid  =
                    ((ts_stream_t*)p_mux->pp_inputs[i]->p_sys)->i_pid;
                p_sys->p_pcr_input= p_mux->pp_inputs[i];
                break;
            }
            else if( p_mux->pp_inputs[i]->p_fmt->i_cat != SPU_ES &&
                     p_sys->i_pcr_pid == 0x1fff )
            {
                p_sys->i_pcr_pid  =
                    ((ts_stream_t*)p_mux->pp_inputs[i]->p_sys)->i_pid;
                p_sys->p_pcr_input= p_mux->pp_inputs[i];
            }
        }
        if( p_sys->p_pcr_input )
        {
            /* Empty TS buffer */
            /* FIXME */
        }
       fprintf( stderr, "new PCR PID is %d\n", p_sys->i_pcr_pid );
    }

    /* Empty all data in chain_pes */
    BufferChainClean( &p_stream->chain_pes );

    free(p_stream->lang);
    free( p_stream->p_extra );
    if( p_stream->i_stream_id == 0xfa ||
        p_stream->i_stream_id == 0xfb ||
        p_stream->i_stream_id == 0xfe )
    {
        p_sys->i_mpeg4_streams--;
    }

//    pid = var_GetInteger( p_mux, SOUT_CFG_PREFIX "pid-video" );
    if ( pid > 0 && pid == p_stream->i_pid )
    {
        p_sys->i_pid_video = pid;
       fprintf( stderr, "freeing video PID %d", pid);
    }
//    pid = var_GetInteger( p_mux, SOUT_CFG_PREFIX "pid-audio" );
    if ( pid > 0 && pid == p_stream->i_pid )
    {
        p_sys->i_pid_audio = pid;
       fprintf( stderr, "freeing audio PID %d", pid);
    }
//    pid = var_GetInteger( p_mux, SOUT_CFG_PREFIX "pid-spu" );
    if ( pid > 0 && pid == p_stream->i_pid )
    {
        p_sys->i_pid_spu = pid;
       fprintf( stderr, "freeing spu PID %d", pid);
    }

    free( p_stream );

    /* We only change PMT version (PAT isn't changed) */
    p_sys->i_pmt_version_number++;
    p_sys->i_pmt_version_number %= 32;

    return VLC_SUCCESS;
}

static void SetHeader( sout_buffer_chain_t *c,
                        int depth )
{
    block_t *p_ts = BufferChainPeek( c );
    while( depth > 0 )
    {
        p_ts = p_ts->p_next;
        depth--;
    }
    p_ts->i_flags |= BLOCK_FLAG_HEADER;
}

/* returns true if needs more data */
static bool MuxStreams(sout_mux_t *p_mux )
{
    sout_mux_t  *p_sys = p_mux;
    ts_stream_t *p_pcr_stream = (ts_stream_t*)p_sys->p_pcr_input->p_sys;

    sout_buffer_chain_t chain_ts;
    mtime_t i_shaping_delay = p_pcr_stream->b_key_frame
        ? p_pcr_stream->i_pes_length
        : p_sys->i_shaping_delay;

    bool b_ok = true;

    /* Accumulate enough data in the pcr stream (>i_shaping_delay) */
    /* Accumulate enough data in all other stream ( >= length of pcr)*/

    for (int i = -1; !b_ok || i < p_mux->i_nb_inputs; i++ )
    {
        if (i == p_mux->i_nb_inputs)
        {
            /* get enough PES packet for all input */
            b_ok = true;
            i = -1;
        }
        sout_input_t *p_input;

        if( i == -1 )
            p_input = p_sys->p_pcr_input;
        else if( p_mux->pp_inputs[i]->p_sys == p_pcr_stream )
            continue;
        else
            p_input = p_mux->pp_inputs[i];
        ts_stream_t *p_stream = (ts_stream_t*)p_input->p_sys;

        if( ( p_stream != p_pcr_stream ||
              p_stream->i_pes_length >= i_shaping_delay ) &&
            p_stream->i_pes_dts + p_stream->i_pes_length >=
            p_pcr_stream->i_pes_dts + p_pcr_stream->i_pes_length )
            continue;


        /* Need more data */        
        if( block_FifoCount( p_input->p_fifo ) <= 1 )
        {
            if( ( p_input->p_fmt->i_cat == AUDIO_ES ) ||
                ( p_input->p_fmt->i_cat == VIDEO_ES ) )
            {
                /* We need more data */
			    return true;
            }
            else if( block_FifoCount( p_input->p_fifo ) <= 0 )
            {
                /* spu, only one packet is needed */
                continue;
            }
            else if( p_input->p_fmt->i_cat == SPU_ES )
            {
                /* Don't mux the SPU yet if it is too early */
                block_t *p_spu = block_FifoShow( p_input->p_fifo );

                int64_t i_spu_delay = p_spu->i_dts - p_sys->first_dts - p_pcr_stream->i_pes_dts;
                if( ( i_spu_delay > i_shaping_delay ) &&
                    ( i_spu_delay < INT64_C(100000000) ) )
                    continue;

                if ( ( i_spu_delay >= INT64_C(100000000) ) ||
                     ( i_spu_delay < INT64_C(10000) ) )
                {
                    BufferChainClean( &p_stream->chain_pes );
                    p_stream->i_pes_dts = 0;
                    p_stream->i_pes_used = 0;
                    p_stream->i_pes_length = 0;
                    continue;
                }
            }
        }
        b_ok = false;

        block_t *p_data;
        if( p_stream == p_pcr_stream || p_sys->b_data_alignment
             || ((p_input->p_fmt->i_codec != VLC_CODEC_MPGA ) &&
                 (p_input->p_fmt->i_codec != VLC_CODEC_MP3) ) )
        {
            p_data = block_FifoGet( p_input->p_fifo );
            if (p_data->i_pts <= VLC_TS_INVALID)
                p_data->i_pts = p_data->i_dts;

            if( p_input->p_fmt->i_codec == VLC_CODEC_MP4A )
                p_data = Add_ADTS( p_data, p_input->p_fmt );
        }
        else
            p_data = FixPES( p_mux, p_input->p_fifo );


        if( block_FifoCount( p_input->p_fifo ) > 0 &&
            p_input->p_fmt->i_cat != SPU_ES )
        {
            block_t *p_next = block_FifoShow( p_input->p_fifo );
            p_data->i_length = p_next->i_dts - p_data->i_dts;
        }
        else if( p_input->p_fmt->i_codec !=
                   VLC_CODEC_SUBT )
            p_data->i_length = 1000;

        if (p_sys->first_dts == 0)
            p_sys->first_dts = p_data->i_dts;

        p_data->i_dts -= p_sys->first_dts;
        p_data->i_pts -= p_sys->first_dts;

        if( ( p_pcr_stream->i_pes_dts > 0 &&
              p_data->i_dts - 10000000 > p_pcr_stream->i_pes_dts +
              p_pcr_stream->i_pes_length ) ||
            p_data->i_dts < p_stream->i_pes_dts ||
            ( p_stream->i_pes_dts > 0 &&
              p_input->p_fmt->i_cat != SPU_ES &&
              p_data->i_dts - 10000000 > p_stream->i_pes_dts +
              p_stream->i_pes_length ) )
        {
            fprintf( stderr, "packet with too strange dts "
                      "(dts=%"PRId64",old=%"PRId64",pcr=%"PRId64")\n",
                      p_data->i_dts, p_stream->i_pes_dts,
                      p_pcr_stream->i_pes_dts );
            block_Release( p_data );

            BufferChainClean( &p_stream->chain_pes );
            p_stream->i_pes_dts = 0;
            p_stream->i_pes_used = 0;
            p_stream->i_pes_length = 0;

            if( p_input->p_fmt->i_cat != SPU_ES )
            {
                BufferChainClean( &p_pcr_stream->chain_pes );
                p_pcr_stream->i_pes_dts = 0;
                p_pcr_stream->i_pes_used = 0;
                p_pcr_stream->i_pes_length = 0;
            }

            continue;
        }

        int i_header_size = 0;
        int i_max_pes_size = 0;
        int b_data_alignment = 0;
        if( p_input->p_fmt->i_cat == SPU_ES ) switch (p_input->p_fmt->i_codec)
        {
        case VLC_CODEC_SUBT:
            /* Prepend header */
            p_data = block_Realloc( p_data, 2, p_data->i_buffer );
            p_data->p_buffer[0] = ( (p_data->i_buffer - 2) >> 8) & 0xff;
            p_data->p_buffer[1] = ( (p_data->i_buffer - 2)     ) & 0xff;

            /* remove trailling \0 if any */
            if( p_data->i_buffer > 2 && !p_data->p_buffer[p_data->i_buffer-1] )
                p_data->i_buffer--;

            /* Append a empty sub (sub text only) */
            if( p_data->i_length > 0 &&
                ( p_data->i_buffer != 1 || *p_data->p_buffer != ' ' ) )
            {
                block_t *p_spu = block_Alloc( 3 );

                p_spu->i_dts = p_data->i_dts + p_data->i_length;
                p_spu->i_pts = p_spu->i_dts;
                p_spu->i_length = 1000;

                p_spu->p_buffer[0] = 0;
                p_spu->p_buffer[1] = 1;
                p_spu->p_buffer[2] = ' ';

                EStoPES( &p_spu, p_spu, p_input->p_fmt,
                             p_stream->i_stream_id, 1, 0, 0, 0 );
                p_data->p_next = p_spu;
            }
            break;

        case VLC_CODEC_TELETEXT:
            /* EN 300 472 */
            i_header_size = 0x24;
            b_data_alignment = 1;
            break;

        case VLC_CODEC_DVBS:
            /* EN 300 743 */
            b_data_alignment = 1;
            break;
        }
        else if( p_data->i_length < 0 || p_data->i_length > 2000000 )
        {
            /* FIXME choose a better value, but anyway we
             * should never have to do that */
            p_data->i_length = 1000;
        }

        p_stream->i_pes_length += p_data->i_length;
        if( p_stream->i_pes_dts == 0 )
        {
            p_stream->i_pes_dts = p_data->i_dts;
        }

        /* Convert to pes */
        if( p_stream->i_stream_id == 0xa0 && p_data->i_pts <= 0 )
        {
            /* XXX yes I know, it's awful, but it's needed,
             * so don't remove it ... */
            p_data->i_pts = p_data->i_dts;
        }

        if( p_input->p_fmt->i_codec == VLC_CODEC_DIRAC )
        {
            b_data_alignment = 1;
            /* dirac pes packets should be unbounded in
             * length, specify a suitibly large max size */
            i_max_pes_size = INT_MAX;
        }

        EStoPES ( &p_data, p_data, p_input->p_fmt, p_stream->i_stream_id,
                       1, b_data_alignment, i_header_size,
                       i_max_pes_size );

        BufferChainAppend( &p_stream->chain_pes, p_data );

        if( p_sys->b_use_key_frames && p_stream == p_pcr_stream
            && (p_data->i_flags & BLOCK_FLAG_TYPE_I)
            && !(p_data->i_flags & BLOCK_FLAG_NO_KEYFRAME)
            && (p_stream->i_pes_length > 400000) )
        {
            i_shaping_delay = p_stream->i_pes_length;
            p_stream->b_key_frame = 1;
        }
    }

    /* save */
    const mtime_t i_pcr_length = p_pcr_stream->i_pes_length;
    p_pcr_stream->b_key_frame = 0;

    fprintf( stderr, "starting muxing %lldms\n", i_pcr_length / 1000 ); 
    /* 2: calculate non accurate total size of muxed ts */
    int i_packet_count = 0;
    for (int i = 0; i < p_mux->i_nb_inputs; i++ )
    {
        ts_stream_t *p_stream = (ts_stream_t*)p_mux->pp_inputs[i]->p_sys;

        /* False for pcr stream but it will be enough to do PCR algo */
        for (block_t *p_pes = p_stream->chain_pes.p_first; p_pes != NULL;
             p_pes = p_pes->p_next )
        {
            int i_size = p_pes->i_buffer;
            if( p_pes->i_dts + p_pes->i_length >
                p_pcr_stream->i_pes_dts + p_pcr_stream->i_pes_length )
            {
                mtime_t i_frag = p_pcr_stream->i_pes_dts +
                    p_pcr_stream->i_pes_length - p_pes->i_dts;
                if( i_frag < 0 )
                {
                    /* Next stream */
                    break;
                }
                i_size = p_pes->i_buffer * i_frag / p_pes->i_length;
            }
            i_packet_count += ( i_size + 183 ) / 184;
        }
    }
    /* add overhead for PCR (not really exact) */
    i_packet_count += (8 * i_pcr_length / p_sys->i_pcr_delay + 175) / 176;

    /* 3: mux PES into TS */
    BufferChainInit( &chain_ts );
    /* append PAT/PMT  -> FIXME with big pcr delay it won't have enough pat/pmt */
    bool pat_was_previous = true; //This is to prevent unnecessary double PAT/PMT insertions
    GetPAT( p_mux, &chain_ts );
    GetPMT( p_mux, &chain_ts );
    int i_packet_pos = 0;
    i_packet_count += chain_ts.i_depth;
    /* fprintf( stderr, "estimated pck=%d", i_packet_count ); */

    const mtime_t i_pcr_dts = p_pcr_stream->i_pes_dts;
    for (;;)
    {
        int          i_stream = -1;
        mtime_t      i_dts = 0;
        ts_stream_t  *p_stream;

        /* Select stream (lowest dts) */
        for (int i = 0; i < p_mux->i_nb_inputs; i++ )
        {
            p_stream = (ts_stream_t*)p_mux->pp_inputs[i]->p_sys;

            if( p_stream->i_pes_dts == 0 )
            {
                continue;
            }

            if( i_stream == -1 || p_stream->i_pes_dts < i_dts )
            {
                i_stream = i;
                i_dts = p_stream->i_pes_dts;
            }
        }
        if( i_stream == -1 || i_dts > i_pcr_dts + i_pcr_length )
        {
            break;
        }
        p_stream = (ts_stream_t*)p_mux->pp_inputs[i_stream]->p_sys;
        sout_input_t *p_input = p_mux->pp_inputs[i_stream];

        /* do we need to issue pcr */
        bool b_pcr = false;
        if( p_stream == p_pcr_stream &&
            i_pcr_dts + i_packet_pos * i_pcr_length / i_packet_count >=
            p_sys->i_pcr + p_sys->i_pcr_delay )
        {
            b_pcr = true;
            p_sys->i_pcr = i_pcr_dts + i_packet_pos *
                i_pcr_length / i_packet_count;
        }

        /* Build the TS packet */
        block_t *p_ts = TSNew( p_mux, p_stream, b_pcr );
        if( p_sys->csa != NULL &&
             (p_input->p_fmt->i_cat != AUDIO_ES || p_sys->b_crypt_audio) &&
             (p_input->p_fmt->i_cat != VIDEO_ES || p_sys->b_crypt_video) )
        {
            p_ts->i_flags |= BLOCK_FLAG_SCRAMBLED;
        }
        i_packet_pos++;

        /* Write PAT/PMT before every keyframe if use-key-frames is enabled,
         * this helps to do segmenting with livehttp-output so it can cut segment
         * and start new one with pat,pmt,keyframe*/
        if( ( p_sys->b_use_key_frames ) && ( p_ts->i_flags & BLOCK_FLAG_TYPE_I ) )
        {
            if( likely( !pat_was_previous ) )
            {
                int startcount = chain_ts.i_depth;
                GetPAT( p_mux, &chain_ts );
                GetPMT( p_mux, &chain_ts );
                SetHeader( &chain_ts, startcount );
                i_packet_count += (chain_ts.i_depth - startcount );
            } else {
                SetHeader( &chain_ts, 0); //We just inserted pat/pmt,so just flag it instead of adding new one
            }
        }
        pat_was_previous = false;

        /* */
        BufferChainAppend( &chain_ts, p_ts );
    }
    /* 4: date and send */
    TSSchedule( p_mux, &chain_ts, i_pcr_length, i_pcr_dts );
    return false;
}

/*****************************************************************************
 * Mux: Call each time there is new data for at least one stream
 *****************************************************************************
 *
 *****************************************************************************/

#if 1
static int Mux( sout_mux_t *p_mux )
{
    sout_mux_t  *p_sys = p_mux;

    if( p_sys->i_pcr_pid == 0x1fff )
    {
        for (int i = 0; i < p_mux->i_nb_inputs; i++ )
        {
            block_FifoEmpty( p_mux->pp_inputs[i]->p_fifo );
        }
        fprintf( stderr, "waiting for PCR streams" );
        return VLC_SUCCESS;
    }

    while (!MuxStreams(p_mux))
        ;
          
    return VLC_SUCCESS;
}
#else

static int Mux( sout_mux_t *p_mux )
{
    sout_mux_t  *p_sys = p_mux;
    ts_stream_t     *p_pcr_stream;

    if( p_sys->i_pcr_pid == 0x1fff )
    {
        int i;
        for( i = 0; i < p_mux->i_nb_inputs; i++ )
        {
            block_FifoEmpty( p_mux->pp_inputs[i]->p_fifo );
        }
        fprintf( stderr, "waiting for PCR streams" );
        return VLC_SUCCESS;
    }
    
    p_pcr_stream = (ts_stream_t*)p_sys->p_pcr_input->p_sys;


    for( ;; )
    {
        sout_buffer_chain_t chain_ts;
        int                 i_packet_count;
        int                 i_packet_pos;
        mtime_t             i_pcr_dts;
        mtime_t             i_pcr_length;
        mtime_t             i_shaping_delay;
        int i;

        if( p_pcr_stream->b_key_frame )
        {
            i_shaping_delay = p_pcr_stream->i_pes_length;
        }
        else
        {
            i_shaping_delay = p_sys->i_shaping_delay;
        }

        /* 1: get enough PES packet for all input */
		//fprintf(stderr , "1: get enough PES packet for all input \n");
   
   //   for( ;; )
        {
            bool b_ok = true;
            block_t *p_data;

            /* Accumulate enough data in the pcr stream (>i_shaping_delay) */
            /* Accumulate enough data in all other stream ( >= length of pcr)*/
            for( i = -1; i < p_mux->i_nb_inputs; i++ )
            {
                sout_input_t *p_input;
                ts_stream_t *p_stream;
                int64_t i_spu_delay = 0;

                if( i == -1 )
                    p_input = p_sys->p_pcr_input;
                else if( p_mux->pp_inputs[i]->p_sys == p_pcr_stream )
                    continue;
                else
                    p_input = p_mux->pp_inputs[i];
                p_stream = (ts_stream_t*)p_input->p_sys;
                if( ( ( p_stream == p_pcr_stream ) &&
                      ( p_stream->i_pes_length < i_shaping_delay ) ) ||
                    ( p_stream->i_pes_dts + p_stream->i_pes_length <
                      p_pcr_stream->i_pes_dts + p_pcr_stream->i_pes_length ) )
                {
                    /* Need more data */
                    
                    if( block_FifoCount( p_input->p_fifo ) <= 1 )
                    {
                        if( ( p_input->p_fmt->i_cat == AUDIO_ES ) ||
                            ( p_input->p_fmt->i_cat == VIDEO_ES ) )
                        {
                            /* We need more data */
                            fprintf(stderr,"We need more data \n");
                            return VLC_SUCCESS;
                        }
                        else if( block_FifoCount( p_input->p_fifo ) <= 0 )
                        {
                            /* spu, only one packet is needed */
                            continue;
                        }
                        else if( p_input->p_fmt->i_cat == SPU_ES )
                        {
                            /* Don't mux the SPU yet if it is too early */
                            block_t *p_spu = block_FifoShow( p_input->p_fifo );

                            i_spu_delay =
                                p_spu->i_dts - p_pcr_stream->i_pes_dts;

                            if( ( i_spu_delay > i_shaping_delay ) &&
                                ( i_spu_delay < INT64_C(100000000) ) )
                                continue;

                            if ( ( i_spu_delay >= INT64_C(100000000) ) ||
                                 ( i_spu_delay < INT64_C(10000) ) )
                            {
                                BufferChainClean( &p_stream->chain_pes );
                                p_stream->i_pes_dts = 0;
                                p_stream->i_pes_used = 0;
                                p_stream->i_pes_length = 0;
                                continue;
                            }
                        }
                    }
                    
                    b_ok = false;
                    if( p_stream == p_pcr_stream || p_sys->b_data_alignment
                         || p_input->p_fmt->i_codec !=
                             VLC_CODEC_MPGA )
                    {
                        p_data = block_FifoGet( p_input->p_fifo );

                        if( p_input->p_fmt->i_codec ==
                                VLC_CODEC_MP4A )
                            p_data = Add_ADTS( p_data, p_input->p_fmt );
                    }
                    else
                        p_data = FixPES( p_mux, p_input->p_fifo );

                    if( block_FifoCount( p_input->p_fifo ) > 0 &&
                        p_input->p_fmt->i_cat != SPU_ES )
                    {
                        block_t *p_next = block_FifoShow( p_input->p_fifo );
                        p_data->i_length = p_next->i_dts - p_data->i_dts;
                    }
                    else if( p_input->p_fmt->i_codec !=
                               VLC_CODEC_SUBT )
                        p_data->i_length = 1000;

                    if( ( p_pcr_stream->i_pes_dts > 0 &&
                          p_data->i_dts - 10000000 > p_pcr_stream->i_pes_dts +
                          p_pcr_stream->i_pes_length ) ||
                        p_data->i_dts < p_stream->i_pes_dts ||
                        ( p_stream->i_pes_dts > 0 &&
                          p_input->p_fmt->i_cat != SPU_ES &&
                          p_data->i_dts - 10000000 > p_stream->i_pes_dts +
                          p_stream->i_pes_length ) )
                    {
                        fprintf( stderr, "packet with too strange dts "
                                  "(dts=%"PRId64",old=%"PRId64",pcr=%"PRId64")\n",
                                  p_data->i_dts, p_stream->i_pes_dts,
                                  p_pcr_stream->i_pes_dts );
                        block_Release( p_data );

                        BufferChainClean( &p_stream->chain_pes );
                        p_stream->i_pes_dts = 0;
                        p_stream->i_pes_used = 0;
                        p_stream->i_pes_length = 0;

                        if( p_input->p_fmt->i_cat != SPU_ES )
                        {
                            BufferChainClean( &p_pcr_stream->chain_pes );
                            p_pcr_stream->i_pes_dts = 0;
                            p_pcr_stream->i_pes_used = 0;
                            p_pcr_stream->i_pes_length = 0;
                        }
                    }
                    else
                    {
                        int i_header_size = 0;
                        int i_max_pes_size = 0;
                        int b_data_alignment = 0;
                        if( p_input->p_fmt->i_cat == SPU_ES )
                        {
                            if( p_input->p_fmt->i_codec ==
                                VLC_CODEC_SUBT )
                            {
                                /* Prepend header */
                                p_data = block_Realloc( p_data, 2,
                                                        p_data->i_buffer );
                                p_data->p_buffer[0] =
                                    ( (p_data->i_buffer - 2) >> 8) & 0xff;
                                p_data->p_buffer[1] =
                                    ( (p_data->i_buffer - 2)     ) & 0xff;

                                /* remove trailling \0 if any */
                                if( p_data->i_buffer > 2 &&
                                    p_data->p_buffer[p_data->i_buffer -1] ==
                                    '\0' )
                                    p_data->i_buffer--;

                                /* Append a empty sub (sub text only) */
                                if( p_data->i_length > 0 &&
                                    !( p_data->i_buffer == 1 &&
                                       *p_data->p_buffer == ' ' ) )
                                {
                                    block_t *p_spu = block_New( p_mux, 3 );

                                    p_spu->i_dts = p_spu->i_pts =
                                        p_data->i_dts + p_data->i_length;
                                    p_spu->i_length = 1000;

                                    p_spu->p_buffer[0] = 0;
                                    p_spu->p_buffer[1] = 1;
                                    p_spu->p_buffer[2] = ' ';

                                    EStoPES(&p_spu, p_spu,
                                                 p_input->p_fmt,
                                                 p_stream->i_stream_id, 1,
                                                 0, 0, 0 );
                                    p_data->p_next = p_spu;
                                }
                            }
                            else if( p_input->p_fmt->i_codec ==
                                       VLC_CODEC_TELETEXT )
                            {
                                /* EN 300 472 */
                                i_header_size = 0x24;
                                b_data_alignment = 1;
                            }
                            else if( p_input->p_fmt->i_codec ==
                                       VLC_CODEC_DVBS )
                            {
                                /* EN 300 743 */
                                b_data_alignment = 1;
                            }
                        }
                        else if( p_data->i_length < 0 ||
                                 p_data->i_length > 2000000 )
                        {
                            /* FIXME choose a better value, but anyway we
                             * should never have to do that */
                            p_data->i_length = 1000;
                        }
                        p_stream->i_pes_length += p_data->i_length;
                        if( p_stream->i_pes_dts == 0 )
                        {
                            p_stream->i_pes_dts = p_data->i_dts;
                            fprintf( stderr, "set stream i_pes_dts %"PRId64"/%"PRId64"\n",
							p_stream->i_pes_dts,p_data->i_dts);
                        }

                        /* Convert to pes */
                        if( p_stream->i_stream_id == 0xa0 &&
                            p_data->i_pts <= 0 )
                        {
                            /* XXX yes I know, it's awful, but it's needed,
                             * so don't remove it ... */
                            p_data->i_pts = p_data->i_dts;
                        }

                        if( p_input->p_fmt->i_codec ==
                                   VLC_CODEC_DIRAC )
                        {
                            b_data_alignment = 1;
                            /* dirac pes packets should be unbounded in
                             * length, specify a suitibly large max size */
                            i_max_pes_size = INT_MAX;
                        }
                        
                        EStoPES (&p_data, p_data,
                                  p_input->p_fmt, p_stream->i_stream_id,
                                  1, b_data_alignment, i_header_size,
                                  i_max_pes_size );
						
						fprintf(stderr,"EStoPES: %d\n",p_data->i_buffer);
                        BufferChainAppend(&p_stream->chain_pes, p_data );
                        
                        if( p_sys->b_use_key_frames && p_stream == p_pcr_stream
                            && (p_data->i_flags & BLOCK_FLAG_TYPE_I)
                            && !(p_data->i_flags & BLOCK_FLAG_NO_KEYFRAME)
                            && (p_stream->i_pes_length > 400000) )
                        {
                            i_shaping_delay = p_stream->i_pes_length;
                            p_stream->b_key_frame = 1;
                        }
                    }
                }
            }

            if( b_ok )
            {
                break;
            }
        }

        /* save */
        i_pcr_dts      = p_pcr_stream->i_pes_dts;
        i_pcr_length   = p_pcr_stream->i_pes_length;
        p_pcr_stream->b_key_frame = 0;

        fprintf( stderr, "starting muxing %lldms\n", i_pcr_length / 1000 );
        /* 2: calculate non accurate total size of muxed ts */
        i_packet_count = 0;
        for( i = 0; i < p_mux->i_nb_inputs; i++ )
        {
            ts_stream_t *p_stream = (ts_stream_t*)p_mux->pp_inputs[i]->p_sys;
            block_t *p_pes;

            /* False for pcr stream but it will be enough to do PCR algo */
            for( p_pes = p_stream->chain_pes.p_first; p_pes != NULL;
                 p_pes = p_pes->p_next )
            {
                int i_size = p_pes->i_buffer;
                if( p_pes->i_dts + p_pes->i_length >
                    p_pcr_stream->i_pes_dts + p_pcr_stream->i_pes_length )
                {
                    mtime_t i_frag = p_pcr_stream->i_pes_dts +
                        p_pcr_stream->i_pes_length - p_pes->i_dts;
                    if( i_frag < 0 )
                    {
                        /* Next stream */
                        break;
                    }
                    i_size = p_pes->i_buffer * i_frag / p_pes->i_length;
                }
                i_packet_count += ( i_size + 183 ) / 184;
            }
        }
        /* add overhead for PCR (not really exact) */
        i_packet_count += (8 * i_pcr_length / p_sys->i_pcr_delay + 175) / 176;

        /* 3: mux PES into TS */
        BufferChainInit( &chain_ts );
        /* append PAT/PMT  -> FIXME with big pcr delay it won't have enough pat/pmt */
        GetPAT( p_mux, &chain_ts );
        GetPMT( p_mux, &chain_ts );
        i_packet_pos = 0;
        i_packet_count += chain_ts.i_depth;
        
        fprintf( stderr, "estimated pck=%d\n", i_packet_count ); 

        for( ;; )
        {
            int          i_stream;
            mtime_t      i_dts;
            ts_stream_t  *p_stream;
            sout_input_t *p_input;
            block_t      *p_ts;
            bool         b_pcr;

            /* Select stream (lowest dts) */
            for( i = 0, i_stream = -1, i_dts = 0; i < p_mux->i_nb_inputs; i++ )
            {
                p_stream = (ts_stream_t*)p_mux->pp_inputs[i]->p_sys;

				fprintf( stderr, "dts %"PRId64"/%"PRId64"\n",
						 p_stream->i_pes_dts,i_dts);
						 
                if( p_stream->i_pes_dts == 0 )
                {
                    continue;
                }
				
				
                if( i_stream == -1 ||
                    p_stream->i_pes_dts < i_dts )
                {
                    i_stream = i;
                    i_dts = p_stream->i_pes_dts;
                }
            }
            
            if( i_stream == -1 || i_dts > i_pcr_dts + i_pcr_length )
            {
				fprintf(stderr,"why brek %d \n",i_stream);
                break;
            }
            
            p_stream = (ts_stream_t*)p_mux->pp_inputs[i_stream]->p_sys;
            p_input = p_mux->pp_inputs[i_stream];

            /* do we need to issue pcr */
            b_pcr = false;
            if( p_stream == p_pcr_stream &&
                i_pcr_dts + i_packet_pos * i_pcr_length / i_packet_count >=
                p_sys->i_pcr + p_sys->i_pcr_delay )
            {
                b_pcr = true;
                p_sys->i_pcr = i_pcr_dts + i_packet_pos *
                    i_pcr_length / i_packet_count;
            }

            /* Build the TS packet */
            p_ts = TSNew( p_mux, p_stream, b_pcr );
            
            fprintf(stderr,"TSNew :%d\n",i_packet_pos);
            if( p_sys->csa != NULL &&
                 (p_input->p_fmt->i_cat != AUDIO_ES || p_sys->b_crypt_audio) &&
                 (p_input->p_fmt->i_cat != VIDEO_ES || p_sys->b_crypt_video) )
            {
                p_ts->i_flags |= BLOCK_FLAG_SCRAMBLED;
            }
            i_packet_pos++;

            /* */
            BufferChainAppend( &chain_ts, p_ts );
        }

        /* 4: date and send */
        TSSchedule( p_mux, &chain_ts, i_pcr_length, i_pcr_dts );
    }
}

#endif

#define STD_PES_PAYLOAD 170
static block_t *FixPES( sout_mux_t *p_mux, block_fifo_t *p_fifo )
{
    VLC_UNUSED(p_mux);
    block_t *p_data;
    size_t i_size;


    p_data = block_FifoShow( p_fifo );
    i_size = p_data->i_buffer;

    if( i_size == STD_PES_PAYLOAD )
    {
        return block_FifoGet( p_fifo );
    }
    else if( i_size > STD_PES_PAYLOAD )
    {
        block_t *p_new = block_Alloc( STD_PES_PAYLOAD );
        memcpy( p_new->p_buffer, p_data->p_buffer, STD_PES_PAYLOAD );
        p_new->i_pts = p_data->i_pts;
        p_new->i_dts = p_data->i_dts;
        p_new->i_length = p_data->i_length * STD_PES_PAYLOAD
                            / p_data->i_buffer;
        p_data->i_buffer -= STD_PES_PAYLOAD;
        p_data->p_buffer += STD_PES_PAYLOAD;
        p_data->i_pts += p_new->i_length;
        p_data->i_dts += p_new->i_length;
        p_data->i_length -= p_new->i_length;
        p_data->i_flags |= BLOCK_FLAG_NO_KEYFRAME;
        return p_new;
    }
    else
    {
        block_t *p_next;
        int i_copy;

        p_data = block_FifoGet( p_fifo );
        p_data = block_Realloc( p_data, 0, STD_PES_PAYLOAD );
        p_next = block_FifoShow( p_fifo );
        if ( p_data->i_flags & BLOCK_FLAG_NO_KEYFRAME )
        {
            p_data->i_flags &= ~BLOCK_FLAG_NO_KEYFRAME;
            p_data->i_pts = p_next->i_pts;
            p_data->i_dts = p_next->i_dts;
        }
        i_copy = __MIN( STD_PES_PAYLOAD - i_size, p_next->i_buffer );

        memcpy( &p_data->p_buffer[i_size], p_next->p_buffer, i_copy );
        p_next->i_pts += p_next->i_length * i_copy / p_next->i_buffer;
        p_next->i_dts += p_next->i_length * i_copy / p_next->i_buffer;
        p_next->i_length -= p_next->i_length * i_copy / p_next->i_buffer;
        p_next->i_buffer -= i_copy;
        p_next->p_buffer += i_copy;
        p_next->i_flags |= BLOCK_FLAG_NO_KEYFRAME;

        if( !p_next->i_buffer )
        {
            p_next = block_FifoGet( p_fifo );
            block_Release( p_next );
        }
        return p_data;
    }
}


static block_t *Add_ADTS( block_t *p_data, es_format_t *p_fmt )
{
#define ADTS_HEADER_SIZE 7 /* CRC needs 2 more bytes */

    uint8_t *p_extra = p_fmt->p_extra;

	fprintf(stderr,"Add_ADTS \n");

    if( !p_data || p_fmt->i_extra < 2 || !p_extra )
        return p_data; /* no data to construct the headers */

    size_t frame_length = p_data->i_buffer + ADTS_HEADER_SIZE;
    int i_index = ( (p_extra[0] << 1) | (p_extra[1] >> 7) ) & 0x0f;
    int i_profile = (p_extra[0] >> 3) - 1; /* i_profile < 4 */

    if( i_index == 0x0f && p_fmt->i_extra < 5 )
        return p_data; /* not enough data */

    int i_channels = (p_extra[i_index == 0x0f ? 4 : 1] >> 3) & 0x0f;

    /* keep a copy in case block_Realloc() fails */
    block_t *p_bak_block = block_Duplicate( p_data );
    if( !p_bak_block ) /* OOM, block_Realloc() is likely to lose our block */
        return p_data; /* the frame isn't correct but that's the best we have */

    block_t *p_new_block = block_Realloc( p_data, ADTS_HEADER_SIZE,
                                            p_data->i_buffer );
    if( !p_new_block )
        return p_bak_block; /* OOM, send the (incorrect) original frame */

    block_Release( p_bak_block ); /* we don't need the copy anymore */


    uint8_t *p_buffer = p_new_block->p_buffer;

    /* fixed header */
    p_buffer[0] = 0xff;
    p_buffer[1] = 0xf1; /* 0xf0 | 0x00 | 0x00 | 0x01 */
    p_buffer[2] = (i_profile << 6) | ((i_index & 0x0f) << 2) | ((i_channels >> 2) & 0x01) ;
    p_buffer[3] = (i_channels << 6) | ((frame_length >> 11) & 0x03);

    /* variable header (starts at last 2 bits of 4th byte) */

    int i_fullness = 0x7ff; /* 0x7ff means VBR */
    /* XXX: We should check if it's CBR or VBR, but no known implementation
     * do that, and it's a pain to calculate this field */

    p_buffer[4] = frame_length >> 3;
    p_buffer[5] = ((frame_length & 0x07) << 5) | ((i_fullness >> 6) & 0x1f);
    p_buffer[6] = ((i_fullness & 0x3f) << 2) /* | 0xfc */;

    return p_new_block;
}

static void TSSchedule( sout_mux_t *p_mux, sout_buffer_chain_t *p_chain_ts,
                        mtime_t i_pcr_length, mtime_t i_pcr_dts )
{
    sout_mux_t  *p_sys = p_mux;
    sout_buffer_chain_t new_chain;
    int i_packet_count = p_chain_ts->i_depth;


    BufferChainInit( &new_chain );

    if ( i_pcr_length <= 0 )
    {
        i_pcr_length = i_packet_count;
    }

    for (int i = 0; i < i_packet_count; i++ )
    {
        block_t *p_ts = BufferChainGet( p_chain_ts );
        mtime_t i_new_dts = i_pcr_dts + i_pcr_length * i / i_packet_count;

        BufferChainAppend( &new_chain, p_ts );

        if (!p_ts->i_dts || p_ts->i_dts + p_sys->i_dts_delay * 2/3 >= i_new_dts)
            continue;

        mtime_t i_max_diff = i_new_dts - p_ts->i_dts;
        mtime_t i_cut_dts = p_ts->i_dts;

        p_ts = BufferChainPeek( p_chain_ts );
        i++;
        i_new_dts = i_pcr_dts + i_pcr_length * i / i_packet_count;
        while ( p_ts != NULL && i_new_dts - p_ts->i_dts >= i_max_diff )
        {
            p_ts = BufferChainGet( p_chain_ts );
            i_max_diff = i_new_dts - p_ts->i_dts;
            i_cut_dts = p_ts->i_dts;
            BufferChainAppend( &new_chain, p_ts );

            p_ts = BufferChainPeek( p_chain_ts );
            i++;
            i_new_dts = i_pcr_dts + i_pcr_length * i / i_packet_count;
        }
        fprintf( stderr, "adjusting rate at %"PRId64"/%"PRId64" (%d/%d)",
                 i_cut_dts - i_pcr_dts, i_pcr_length, new_chain.i_depth,
                 p_chain_ts->i_depth );
        if ( new_chain.i_depth )
            TSDate( p_mux, &new_chain, i_cut_dts - i_pcr_dts, i_pcr_dts );
        if ( p_chain_ts->i_depth )
            TSSchedule( p_mux, p_chain_ts, i_pcr_dts + i_pcr_length - i_cut_dts,
                        i_cut_dts );
        return;
    }

    if ( new_chain.i_depth )
        TSDate( p_mux, &new_chain, i_pcr_length, i_pcr_dts );
}

static block_t *WritePSISection( dvbpsi_psi_section_t* p_section )
{
    block_t   *p_psi, *p_first = NULL;

    while( p_section )
    {
        int i_size = (uint32_t)(p_section->p_payload_end - p_section->p_data) +
                  (p_section->b_syntax_indicator ? 4 : 0);

        p_psi = block_Alloc( i_size + 1 );
        if( !p_psi )
            goto error;
        p_psi->i_pts = 0;
        p_psi->i_dts = 0;
        p_psi->i_length = 0;
        p_psi->i_buffer = i_size + 1;

        p_psi->p_buffer[0] = 0; /* pointer */
        memcpy( p_psi->p_buffer + 1,
                p_section->p_data,
                i_size );

        block_ChainAppend( &p_first, p_psi );

        p_section = p_section->p_next;
    }

    return( p_first );

error:
    if( p_first )
        block_ChainRelease( p_first );
    return NULL;
}

static void GetPAT( sout_mux_t *p_mux,
                    sout_buffer_chain_t *c )
{
    sout_mux_t       *p_sys = p_mux;
    block_t              *p_pat;
    dvbpsi_pat_t         pat;
    dvbpsi_psi_section_t *p_section;


    dvbpsi_InitPAT( &pat, p_sys->i_tsid, p_sys->i_pat_version_number,
                    1 );      /* b_current_next */
    /* add all programs */
    for (unsigned i = 0; i < p_sys->i_num_pmt; i++ )
        dvbpsi_PATAddProgram( &pat, p_sys->i_pmt_program_number[i],
                              p_sys->pmt[i].i_pid );

#if (DVBPSI_VERSION_INT >= DVBPSI_VERSION_WANTED(1,0,0))
    p_section = dvbpsi_pat_sections_generate( p_sys->p_dvbpsi, &pat, 0 );
#else
    p_section = dvbpsi_GenPATSections( &pat, 0 /* max program per section */ );
#endif
    p_pat = WritePSISection( p_section );

    PEStoTS( c, p_pat, &p_sys->pat );

    dvbpsi_DeletePSISections( p_section );
    dvbpsi_EmptyPAT( &pat );
}

static void GetPMTmpeg4(sout_mux_t *p_mux)
{
    sout_mux_t *p_sys = p_mux;
    uint8_t iod[4096];
    bits_buffer_t bits, bits_fix_IOD;

    /* Make valgrind happy : it works at byte level not bit one so
     * bit_write confuse it (but DON'T CHANGE the way that bit_write is
     * working (needed when fixing some bits) */
    memset( iod, 0, 4096 );

    bits_initwrite( &bits, 4096, iod );
    /* IOD_label_scope */
    bits_write( &bits, 8,   0x11 );
    /* IOD_label */
    bits_write( &bits, 8,   0x01 );
    /* InitialObjectDescriptor */
    bits_align( &bits );
    bits_write( &bits, 8,   0x02 );     /* tag */
    bits_fix_IOD = bits;    /* save states to fix length later */
    bits_write( &bits, 24,
        GetDescriptorLength24b( 0 ) );  /* variable length (fixed later) */
    bits_write( &bits, 10,  0x01 );     /* ObjectDescriptorID */
    bits_write( &bits, 1,   0x00 );     /* URL Flag */
    bits_write( &bits, 1,   0x00 );     /* includeInlineProfileLevelFlag */
    bits_write( &bits, 4,   0x0f );     /* reserved */
    bits_write( &bits, 8,   0xff );     /* ODProfile (no ODcapability ) */
    bits_write( &bits, 8,   0xff );     /* sceneProfile */
    bits_write( &bits, 8,   0xfe );     /* audioProfile (unspecified) */
    bits_write( &bits, 8,   0xfe );     /* visualProfile( // ) */
    bits_write( &bits, 8,   0xff );     /* graphicProfile (no ) */
    for (int i_stream = 0; i_stream < p_mux->i_nb_inputs; i_stream++ )
    {
        ts_stream_t *p_stream = (ts_stream_t*)p_mux->pp_inputs[i_stream]->p_sys;

        if( p_stream->i_stream_id != 0xfa && p_stream->i_stream_id != 0xfb &&
            p_stream->i_stream_id != 0xfe )
            continue;

        bits_buffer_t bits_fix_ESDescr, bits_fix_Decoder;
        /* ES descriptor */
        bits_align( &bits );
        bits_write( &bits, 8,   0x03 );     /* ES_DescrTag */
        bits_fix_ESDescr = bits;
        bits_write( &bits, 24,
                    GetDescriptorLength24b( 0 ) ); /* variable size */
        bits_write( &bits, 16,  p_stream->i_es_id );
        bits_write( &bits, 1,   0x00 );     /* streamDependency */
        bits_write( &bits, 1,   0x00 );     /* URL Flag */
        bits_write( &bits, 1,   0x00 );     /* OCRStreamFlag */
        bits_write( &bits, 5,   0x1f );     /* streamPriority */

        /* DecoderConfigDesciptor */
        bits_align( &bits );
        bits_write( &bits, 8,   0x04 ); /* DecoderConfigDescrTag */
        bits_fix_Decoder = bits;
        bits_write( &bits, 24,  GetDescriptorLength24b( 0 ) );
        if( p_stream->i_stream_type == 0x10 )
        {
            bits_write( &bits, 8, 0x20 );   /* Visual 14496-2 */
            bits_write( &bits, 6, 0x04 );   /* VisualStream */
        }
        else if( p_stream->i_stream_type == 0x1b )
        {
            bits_write( &bits, 8, 0x21 );   /* Visual 14496-2 */
            bits_write( &bits, 6, 0x04 );   /* VisualStream */
        }
        else if( p_stream->i_stream_type == 0x11 ||
                 p_stream->i_stream_type == 0x0f )
        {
            bits_write( &bits, 8, 0x40 );   /* Audio 14496-3 */
            bits_write( &bits, 6, 0x05 );   /* AudioStream */
        }
        else if( p_stream->i_stream_type == 0x12 &&
                 p_stream->i_codec == VLC_CODEC_SUBT )
        {
            bits_write( &bits, 8, 0x0B );   /* Text Stream */
            bits_write( &bits, 6, 0x04 );   /* VisualStream */
        }
        else
        {
            bits_write( &bits, 8, 0x00 );
            bits_write( &bits, 6, 0x00 );

            fprintf( stderr, "Unsupported stream_type => broken IOD" );
        }
        bits_write( &bits, 1,   0x00 );         /* UpStream */
        bits_write( &bits, 1,   0x01 );         /* reserved */
        bits_write( &bits, 24,  1024 * 1024 );  /* bufferSizeDB */
        bits_write( &bits, 32,  0x7fffffff );   /* maxBitrate */
        bits_write( &bits, 32,  0 );            /* avgBitrate */

        if( p_stream->i_extra > 0 )
        {
            /* DecoderSpecificInfo */
            bits_align( &bits );
            bits_write( &bits, 8,   0x05 ); /* tag */
            bits_write( &bits, 24, GetDescriptorLength24b(
                        p_stream->i_extra ) );
            for (int i = 0; i < p_stream->i_extra; i++ )
            {
                bits_write( &bits, 8,
                    ((uint8_t*)p_stream->p_extra)[i] );
            }
        }
        /* fix Decoder length */
        bits_write( &bits_fix_Decoder, 24,
                    GetDescriptorLength24b( bits.i_data -
                    bits_fix_Decoder.i_data - 3 ) );

        /* SLConfigDescriptor : predefined (0x01) */
        bits_align( &bits );
        bits_write( &bits, 8,   0x06 ); /* tag */
        bits_write( &bits, 24,  GetDescriptorLength24b( 8 ) );
        bits_write( &bits, 8,   0x01 );/* predefined */
        bits_write( &bits, 1,   0 );   /* durationFlag */
        bits_write( &bits, 32,  0 );   /* OCRResolution */
        bits_write( &bits, 8,   0 );   /* OCRLength */
        bits_write( &bits, 8,   0 );   /* InstantBitrateLength */
        bits_align( &bits );

        /* fix ESDescr length */
        bits_write( &bits_fix_ESDescr, 24,
                    GetDescriptorLength24b( bits.i_data -
                    bits_fix_ESDescr.i_data - 3 ) );
    }
    bits_align( &bits );
    /* fix IOD length */
    bits_write( &bits_fix_IOD, 24,
                GetDescriptorLength24b(bits.i_data - bits_fix_IOD.i_data - 3 ));

    dvbpsi_PMTAddDescriptor(&p_sys->dvbpmt[0], 0x1d, bits.i_data, bits.p_data);
}


static void GetPMT( sout_mux_t *p_mux, sout_buffer_chain_t *c )
{
    sout_mux_t *p_sys = p_mux;


    if( p_sys->dvbpmt == NULL )
    {
        p_sys->dvbpmt = malloc( p_sys->i_num_pmt * sizeof(dvbpsi_pmt_t) );
        if( p_sys->dvbpmt == NULL )
            return;
    }

    dvbpsi_sdt_t sdt;
    if( p_sys->b_sdt )
        dvbpsi_InitSDT( &sdt, p_sys->i_tsid, 1, 1, p_sys->i_netid );

    for (unsigned i = 0; i < p_sys->i_num_pmt; i++ )
    {
        dvbpsi_InitPMT( &p_sys->dvbpmt[i],
                        p_sys->i_pmt_program_number[i],   /* program number */
                        p_sys->i_pmt_version_number,
                        1,      /* b_current_next */
                        p_sys->i_pcr_pid );

        if( !p_sys->b_sdt )
            continue;

        dvbpsi_sdt_service_t *p_service = dvbpsi_SDTAddService( &sdt,
            p_sys->i_pmt_program_number[i],  /* service id */
            0,         /* eit schedule */
            0,         /* eit present */
            4,         /* running status ("4=RUNNING") */
            0 );       /* free ca */

        const char *psz_sdtprov = p_sys->sdt_descriptors[i].psz_provider;
        const char *psz_sdtserv = p_sys->sdt_descriptors[i].psz_service_name;

        if( !psz_sdtprov || !psz_sdtserv )
            continue;
        size_t provlen = VLC_CLIP(strlen(psz_sdtprov), 0, 255);
        size_t servlen = VLC_CLIP(strlen(psz_sdtserv), 0, 255);

        uint8_t psz_sdt_desc[3 + provlen + servlen];

        psz_sdt_desc[0] = 0x01; /* digital television service */

        /* service provider name length */
        psz_sdt_desc[1] = (char)provlen;
        memcpy( &psz_sdt_desc[2], psz_sdtprov, provlen );

        /* service name length */
        psz_sdt_desc[ 2 + provlen ] = (char)servlen;
        memcpy( &psz_sdt_desc[3+provlen], psz_sdtserv, servlen );

#if (DVBPSI_VERSION_INT >= DVBPSI_VERSION_WANTED(1,0,0))
        dvbpsi_sdt_service_descriptor_add( p_service, 0x48,
                                           (3 + provlen + servlen),
                                           psz_sdt_desc );
#else
        dvbpsi_SDTServiceAddDescriptor( p_service, 0x48,
                3 + provlen + servlen, psz_sdt_desc );
#endif
    }

    if( p_sys->i_mpeg4_streams > 0 )
        GetPMTmpeg4(p_mux);

    for (int i_stream = 0; i_stream < p_mux->i_nb_inputs; i_stream++ )
    {
        ts_stream_t *p_stream = (ts_stream_t*)p_mux->pp_inputs[i_stream]->p_sys;

        int i_pidinput = p_mux->pp_inputs[i_stream]->p_fmt->i_id;
        pmt_map_t *p_usepid = bsearch( &i_pidinput, p_sys->pmtmap,
                    p_sys->i_pmtslots, sizeof(pmt_map_t), intcompare );

        /* If there's an error somewhere, dump it to the first pmt */
        unsigned prog = p_usepid ? p_usepid->i_prog : 0;

        dvbpsi_pmt_es_t *p_es = dvbpsi_PMTAddES( &p_sys->dvbpmt[prog],
                    p_stream->i_stream_type, p_stream->i_pid );

        if( p_stream->i_stream_id == 0xfa || p_stream->i_stream_id == 0xfb )
        {
            uint8_t     es_id[2];

            /* SL descriptor */
            es_id[0] = (p_stream->i_es_id >> 8)&0xff;
            es_id[1] = (p_stream->i_es_id)&0xff;
            dvbpsi_PMTESAddDescriptor( p_es, 0x1f, 2, es_id );
        }
        else if( p_stream->i_stream_type == 0xa0 )
        {
            uint8_t data[512];
            int i_extra = __MIN( p_stream->i_extra, 502 );

            /* private DIV3 descripor */
            memcpy( &data[0], &p_stream->i_bih_codec, 4 );
            data[4] = ( p_stream->i_bih_width >> 8 )&0xff;
            data[5] = ( p_stream->i_bih_width      )&0xff;
            data[6] = ( p_stream->i_bih_height>> 8 )&0xff;
            data[7] = ( p_stream->i_bih_height     )&0xff;
            data[8] = ( i_extra >> 8 )&0xff;
            data[9] = ( i_extra      )&0xff;
            if( i_extra > 0 )
            {
                memcpy( &data[10], p_stream->p_extra, i_extra );
            }

            /* 0xa0 is private */
            dvbpsi_PMTESAddDescriptor( p_es, 0xa0, i_extra + 10, data );
        }
        else if( p_stream->i_stream_type == 0x81 )
        {
            uint8_t format[4] = { 'A', 'C', '-', '3'};

            /* "registration" descriptor : "AC-3" */
            dvbpsi_PMTESAddDescriptor( p_es, 0x05, 4, format );
        }
        else if( p_stream->i_codec == VLC_CODEC_DIRAC )
        {
            /* Dirac registration descriptor */

            uint8_t data[4] = { 'd', 'r', 'a', 'c' };
            dvbpsi_PMTESAddDescriptor( p_es, 0x05, 4, data );
        }
        else if( p_stream->i_codec == VLC_CODEC_DTS )
        {
            /* DTS registration descriptor (ETSI TS 101 154 Annex F) */

            /* DTS format identifier, frame size 1024 - FIXME */
            uint8_t data[4] = { 'D', 'T', 'S', '2' };
            dvbpsi_PMTESAddDescriptor( p_es, 0x05, 4, data );
        }
        else if( p_stream->i_codec == VLC_CODEC_EAC3 )
        {
            uint8_t data[1] = { 0x00 };
            dvbpsi_PMTESAddDescriptor( p_es, 0x7a, 1, data );
        }
        else if( p_stream->i_codec == VLC_CODEC_TELETEXT )
        {
            if( p_stream->i_extra )
            {
                dvbpsi_PMTESAddDescriptor( p_es, 0x56,
                                           p_stream->i_extra,
                                           p_stream->p_extra );
            }
            continue;
        }
        else if( p_stream->i_codec == VLC_CODEC_DVBS )
        {
            /* DVB subtitles */
            if( p_stream->i_extra )
            {
                /* pass-through from the TS demux */
                dvbpsi_PMTESAddDescriptor( p_es, 0x59,
                                           p_stream->i_extra,
                                           p_stream->p_extra );
            }
#if 0            
            else
            {
                /* from the dvbsub transcoder */
                dvbpsi_dvb_subtitling_dr_t descr;
                dvbpsi_subtitle_t sub;
                dvbpsi_descriptor_t *p_descr;

                memcpy( sub.i_iso6392_language_code, p_stream->lang, 3 );
                sub.i_subtitling_type = 0x10; /* no aspect-ratio criticality */
                sub.i_composition_page_id = p_stream->i_es_id & 0xFF;
                sub.i_ancillary_page_id = p_stream->i_es_id >> 16;

                descr.i_subtitles_number = 1;
                descr.p_subtitle[0] = sub;

                p_descr = dvbpsi_GenSubtitlingDr( &descr, 0 );
                /* Work around bug in old libdvbpsi */ p_descr->i_length = 8;
                dvbpsi_PMTESAddDescriptor( p_es, p_descr->i_tag,
                                           p_descr->i_length, p_descr->p_data );
            }
#endif
            continue;
        }

        if( p_stream->i_langs )
        {
            dvbpsi_PMTESAddDescriptor( p_es, 0x0a, 4*p_stream->i_langs,
                p_stream->lang);
        }
    }

    for (unsigned i = 0; i < p_sys->i_num_pmt; i++ )
    {
        dvbpsi_psi_section_t *sect;
#if (DVBPSI_VERSION_INT >= DVBPSI_VERSION_WANTED(1,0,0))
        sect = dvbpsi_pmt_sections_generate( p_sys->p_dvbpsi, &p_sys->dvbpmt[i] );
#else
        sect = dvbpsi_GenPMTSections( &p_sys->dvbpmt[i] );
#endif
        block_t *pmt = WritePSISection( sect );
        PEStoTS( c, pmt, &p_sys->pmt[i] );
        dvbpsi_DeletePSISections(sect);
        dvbpsi_EmptyPMT( &p_sys->dvbpmt[i] );
    }

    if( p_sys->b_sdt )
    {
        dvbpsi_psi_section_t *sect;
#if (DVBPSI_VERSION_INT >= DVBPSI_VERSION_WANTED(1,0,0))
        sect = dvbpsi_sdt_sections_generate( p_sys->p_dvbpsi, &sdt );
#else
        sect = dvbpsi_GenSDTSections( &sdt );
#endif
        block_t *p_sdt = WritePSISection( sect );
        PEStoTS( c, p_sdt, &p_sys->sdt );
        dvbpsi_DeletePSISections( sect );
        dvbpsi_EmptySDT( &sdt );
    }
}

static void TSDate( sout_mux_t *p_mux, sout_buffer_chain_t *p_chain_ts,
                    mtime_t i_pcr_length, mtime_t i_pcr_dts )
{
    sout_mux_t  *p_sys = p_mux;
    int i_packet_count = p_chain_ts->i_depth;

    if ( i_pcr_length / 1000 > 0 )
    {
        int i_bitrate = ((uint64_t)i_packet_count * 188 * 8000)
                          / (uint64_t)(i_pcr_length / 1000);
        if ( p_sys->i_bitrate_max && p_sys->i_bitrate_max < i_bitrate )
        {
           //fprintf( stderr, "max bitrate exceeded at %"PRId64
           //           " (%d bi/s for %d pkt in %"PRId64" us)",
           //           i_pcr_dts + p_sys->i_shaping_delay * 3 / 2 - mdate(),
           //           i_bitrate, i_packet_count, i_pcr_length);
           ;
        }
    }
    else
    {
        /* This shouldn't happen, but happens in some rare heavy load
         * and packet losses conditions. */
        i_pcr_length = i_packet_count;
    }

    /* fprintf( stderr, "real pck=%d", i_packet_count ); */
    for (int i = 0; i < i_packet_count; i++ )
    {
        block_t *p_ts = BufferChainGet( p_chain_ts );
        mtime_t i_new_dts = i_pcr_dts + i_pcr_length * i / i_packet_count;

        p_ts->i_dts    = i_new_dts;
        p_ts->i_length = i_pcr_length / i_packet_count;

        if( p_ts->i_flags & BLOCK_FLAG_CLOCK )
        {
            /* fprintf( stderr, "pcr=%lld ms", p_ts->i_dts / 1000 ); */
            TSSetPCR( p_ts, p_ts->i_dts - p_sys->i_dts_delay );
        }
        if( p_ts->i_flags & BLOCK_FLAG_SCRAMBLED )
        {
            pthread_mutex_lock( &p_sys->csa_lock );
            csa_Encrypt( p_sys->csa, p_ts->p_buffer, p_sys->i_csa_pkt_size );
            pthread_mutex_unlock( &p_sys->csa_lock );
        }

        /* latency */
        p_ts->i_dts += p_sys->i_shaping_delay * 3 / 2;

        //sout_AccessOutWrite( p_mux->p_access, p_ts );
        // TODO: sout_AccessOutWrite
        if ( p_sys->p_access )
			p_sys->p_access(p_sys->p_private,p_ts->p_buffer,p_ts->i_buffer);
    }
}

static block_t *TSNew( sout_mux_t *p_mux, ts_stream_t *p_stream,
                       bool b_pcr )
{
    VLC_UNUSED(p_mux);
    block_t *p_pes = p_stream->chain_pes.p_first;

    bool b_new_pes = false;
    bool b_adaptation_field = false;


    int i_payload_max = 184 - ( b_pcr ? 8 : 0 );

    if( p_stream->i_pes_used <= 0 )
    {
        b_new_pes = true;
    }
    int i_payload = __MIN( (int)p_pes->i_buffer - p_stream->i_pes_used,
                       i_payload_max );

    if( b_pcr || i_payload < i_payload_max )
    {
        b_adaptation_field = true;
    }

    block_t *p_ts = block_Alloc( 188 );

    if (b_new_pes && !(p_pes->i_flags & BLOCK_FLAG_NO_KEYFRAME) && p_pes->i_flags & BLOCK_FLAG_TYPE_I)
    {
        p_ts->i_flags |= BLOCK_FLAG_TYPE_I;
    }

    p_ts->i_dts = p_pes->i_dts;

    p_ts->p_buffer[0] = 0x47;
    p_ts->p_buffer[1] = ( b_new_pes ? 0x40 : 0x00 ) |
        ( ( p_stream->i_pid >> 8 )&0x1f );
    p_ts->p_buffer[2] = p_stream->i_pid & 0xff;
    p_ts->p_buffer[3] = ( b_adaptation_field ? 0x30 : 0x10 ) |
        p_stream->i_continuity_counter;

    p_stream->i_continuity_counter = (p_stream->i_continuity_counter+1)%16;
    p_stream->b_discontinuity = p_pes->i_flags & BLOCK_FLAG_DISCONTINUITY;

    if( b_adaptation_field )
    {
        int i_stuffing = i_payload_max - i_payload;
        if( b_pcr )
        {
            p_ts->i_flags |= BLOCK_FLAG_CLOCK;

            p_ts->p_buffer[4] = 7 + i_stuffing;
            p_ts->p_buffer[5] = 0x10;   /* flags */
            if( p_stream->b_discontinuity )
            {
                p_ts->p_buffer[5] |= 0x80; /* flag TS dicontinuity */
                p_stream->b_discontinuity = false;
            }
            p_ts->p_buffer[6] = 0 &0xff;
            p_ts->p_buffer[7] = 0 &0xff;
            p_ts->p_buffer[8] = 0 &0xff;
            p_ts->p_buffer[9] = 0 &0xff;
            p_ts->p_buffer[10]= ( 0 &0x80 ) | 0x7e;
            p_ts->p_buffer[11]= 0;

            for (int i = 12; i < 12 + i_stuffing; i++ )
            {
                p_ts->p_buffer[i] = 0xff;
            }
        }
        else
        {
            p_ts->p_buffer[4] = i_stuffing - 1;
            if( i_stuffing > 1 )
            {
                p_ts->p_buffer[5] = 0x00;
                for (int i = 6; i < 6 + i_stuffing - 2; i++ )
                {
                    p_ts->p_buffer[i] = 0xff;
                }
            }
        }
    }

    /* copy payload */
    memcpy( &p_ts->p_buffer[188 - i_payload],
            &p_pes->p_buffer[p_stream->i_pes_used], i_payload );

    p_stream->i_pes_used += i_payload;
    p_stream->i_pes_dts = p_pes->i_dts + p_pes->i_length *
        p_stream->i_pes_used / p_pes->i_buffer;
    p_stream->i_pes_length -= p_pes->i_length * i_payload / p_pes->i_buffer;

    if( p_stream->i_pes_used >= (int)p_pes->i_buffer )
    {
        block_Release(BufferChainGet( &p_stream->chain_pes ));

        p_pes = p_stream->chain_pes.p_first;
        p_stream->i_pes_length = 0;
        if( p_pes )
        {
            p_stream->i_pes_dts = p_pes->i_dts;
            while( p_pes )
            {
                p_stream->i_pes_length += p_pes->i_length;
                p_pes = p_pes->p_next;
            }
        }
        else
        {
            p_stream->i_pes_dts = 0;
        }
        p_stream->i_pes_used = 0;
    }
    return p_ts;
}

static void TSSetPCR( block_t *p_ts, mtime_t i_dts )
{
    mtime_t i_pcr = 9 * i_dts / 100;
	
    p_ts->p_buffer[6]  = ( i_pcr >> 25 )&0xff;
    p_ts->p_buffer[7]  = ( i_pcr >> 17 )&0xff;
    p_ts->p_buffer[8]  = ( i_pcr >> 9  )&0xff;
    p_ts->p_buffer[9]  = ( i_pcr >> 1  )&0xff;
    p_ts->p_buffer[10]|= ( i_pcr << 7  )&0x80;
}

static void PEStoTS( sout_buffer_chain_t *c, block_t *p_pes,
                     ts_stream_t *p_stream )
{
    /* get PES total size */
    uint8_t *p_data = p_pes->p_buffer;
    int      i_size = p_pes->i_buffer;

    bool    b_new_pes = true;

    for (;;)
    {
        /* write header
         * 8b   0x47    sync byte
         * 1b           transport_error_indicator
         * 1b           payload_unit_start
         * 1b           transport_priority
         * 13b          pid
         * 2b           transport_scrambling_control
         * 2b           if adaptation_field 0x03 else 0x01
         * 4b           continuity_counter
         */

        int i_copy = __MIN( i_size, 184 );
        bool b_adaptation_field = i_size < 184;
        block_t *p_ts = block_Alloc( 188 );

        p_ts->p_buffer[0] = 0x47;
        p_ts->p_buffer[1] = ( b_new_pes ? 0x40 : 0x00 )|
                            ( ( p_stream->i_pid >> 8 )&0x1f );
        p_ts->p_buffer[2] = p_stream->i_pid & 0xff;
        p_ts->p_buffer[3] = ( b_adaptation_field ? 0x30 : 0x10 )|
                            p_stream->i_continuity_counter;

        b_new_pes = false;
        p_stream->i_continuity_counter = (p_stream->i_continuity_counter+1)%16;

        if( b_adaptation_field )
        {
            int i_stuffing = 184 - i_copy;

            p_ts->p_buffer[4] = i_stuffing - 1;
            if( i_stuffing > 1 )
            {
                p_ts->p_buffer[5] = 0x00;
                if( p_stream->b_discontinuity )
                {
                    p_ts->p_buffer[5] |= 0x80;
                    p_stream->b_discontinuity = false;
                }
                for (int i = 6; i < 6 + i_stuffing - 2; i++ )
                {
                    p_ts->p_buffer[i] = 0xff;
                }
            }
        }
        /* copy payload */
        memcpy( &p_ts->p_buffer[188 - i_copy], p_data, i_copy );
        p_data += i_copy;
        i_size -= i_copy;

        BufferChainAppend( c, p_ts );

        if( i_size <= 0 )
        {
            block_t *p_next = p_pes->p_next;

            p_pes->p_next = NULL;
            block_Release( p_pes );
            if( p_next == NULL )
                return;

            b_new_pes = true;
            p_pes = p_next;
            i_size = p_pes->i_buffer;
            p_data = p_pes->p_buffer;
        }
    }
}


