#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "vlc_block.h"
#include "vlc_block_helper.h"
#include "vlc_bits.h"

#include "csa.h"
#include "tsmux.h"

enum
{
    STATE_NOSYNC,
    STATE_NEXT_SYNC
};

/****************************************************************************
 * Local prototypes
 ****************************************************************************/
typedef struct
{
    int i_nal_type;
    int i_nal_ref_idc;

    int i_frame_type;
    int i_pic_parameter_set_id;
    int i_frame_num;

    int i_field_pic_flag;
    int i_bottom_field_flag;

    int i_idr_pic_id;

    int i_pic_order_cnt_lsb;
    int i_delta_pic_order_cnt_bottom;

    int i_delta_pic_order_cnt0;
    int i_delta_pic_order_cnt1;
} slice_t;

#define SPS_MAX (32)
#define PPS_MAX (256)

#define H264_PACKET_SIZE 2048

typedef struct {	    
    int i_state;
    block_bytestream_t bytestream;
    size_t i_offset;
    bool   b_flushing;

    int i_startcode;
    const uint8_t *p_startcode;

    int i_au_prepend;
    const uint8_t *p_au_prepend;
    
    unsigned i_au_min_size;
    
    /* */
    bool    b_slice;
    block_t *p_frame;
    bool    b_frame_sps;
    bool    b_frame_pps;

    bool   b_header;
    bool   b_sps;
    bool   b_pps;
    block_t *pp_sps[SPS_MAX];
    block_t *pp_pps[PPS_MAX];


    /* Useful values of the Sequence Parameter Set */
    int i_log2_max_frame_num;
    int b_frame_mbs_only;
    int i_pic_order_cnt_type;
    int i_delta_pic_order_always_zero_flag;
    int i_log2_max_pic_order_cnt_lsb;

    /* Value from Picture Parameter Set */
    int i_pic_order_present_flag;

    /* Useful values of the Slice Header */
    slice_t slice;

    /* */
    mtime_t i_frame_pts;
    mtime_t i_frame_dts;
	
	es_format_t fmt_out;
	
}h264_demux_t;

enum nal_unit_type_e
{
    NAL_UNKNOWN = 0,
    NAL_SLICE   = 1,
    NAL_SLICE_DPA   = 2,
    NAL_SLICE_DPB   = 3,
    NAL_SLICE_DPC   = 4,
    NAL_SLICE_IDR   = 5,    /* ref_idc != 0 */
    NAL_SEI         = 6,    /* ref_idc == 0 */
    NAL_SPS         = 7,
    NAL_PPS         = 8,
    NAL_AU_DELIMITER= 9
    /* ref_idc == 0 for 6,9,10,11,12 */
};

enum nal_priority_e
{
    NAL_PRIORITY_DISPOSABLE = 0,
    NAL_PRIORITY_LOW        = 1,
    NAL_PRIORITY_HIGH       = 2,
    NAL_PRIORITY_HIGHEST    = 3,
};


#define BLOCK_FLAG_PRIVATE_AUD (1 << BLOCK_FLAG_PRIVATE_SHIFT)


static const uint8_t p_h264_startcode[3] = { 0x00, 0x00, 0x01 };

static inline int bs_read_ue( bs_t *s )
{
    int i = 0;

    while( bs_read1( s ) == 0 && s->p < s->p_end && i < 32 )
    {
        i++;
    }
    return( ( 1 << i) - 1 + bs_read( s, i ) );
}

static inline int bs_read_se( bs_t *s )
{
    int val = bs_read_ue( s );

    return val&0x01 ? (val+1)/2 : -(val/2);
}

static block_t *PacketizeParse(h264_demux_t * p_dec,bool *pb_ts_used, block_t * );
static block_t *ParseNALBlock( h264_demux_t *, bool *pb_used_ts, block_t * );
static block_t *CreateAnnexbNAL( h264_demux_t *, const uint8_t *p, int );

static block_t *OutputPicture( h264_demux_t *p_dec );
static void PutSPS( h264_demux_t *p_dec, block_t *p_frag );
static void PutPPS( h264_demux_t *p_dec, block_t *p_frag );
static void ParseSlice( h264_demux_t *p_dec, bool *pb_new_picture, slice_t *p_slice,
                        int i_nal_ref_idc, int i_nal_type, const block_t *p_frag );
static void ParseSei( h264_demux_t *, block_t * );

static block_t *CreateAnnexbNAL( h264_demux_t *p_dec, const uint8_t *p, int i_size )
{
    block_t *p_nal;

    p_nal = block_New( p_dec, 4 + i_size );
    if( !p_nal ) return NULL;

    /* Add start code */
    p_nal->p_buffer[0] = 0x00;
    p_nal->p_buffer[1] = 0x00;
    p_nal->p_buffer[2] = 0x00;
    p_nal->p_buffer[3] = 0x01;

    /* Copy nalu */
    memcpy( &p_nal->p_buffer[4], p, i_size );

    VLC_UNUSED(p_dec);
    return p_nal;
}

static void CreateDecodedNAL( uint8_t **pp_ret, int *pi_ret,
                              const uint8_t *src, int i_src )
{
    const uint8_t *end = &src[i_src];
    uint8_t *dst = malloc( i_src );

    *pp_ret = dst;

    if( dst )
    {
        while( src < end )
        {
            if( src < end - 3 && src[0] == 0x00 && src[1] == 0x00 &&
                src[2] == 0x03 )
            {
                *dst++ = 0x00;
                *dst++ = 0x00;

                src += 3;
                continue;
            }
            *dst++ = *src++;
        }
    }
    *pi_ret = dst - *pp_ret;
}


static h264_demux_t * demux_open( )
{
	int i;
	h264_demux_t * p_pack = malloc( sizeof( h264_demux_t) );
	
	p_pack->i_state = STATE_NOSYNC;
	p_pack->bytestream = block_BytestreamInit();
	p_pack->i_offset = 0;
    p_pack->b_flushing = false;

    p_pack->i_au_prepend = 1;
    p_pack->p_au_prepend = p_h264_startcode;
    p_pack->i_au_min_size = 5;

    p_pack->i_startcode = sizeof(p_h264_startcode);
    p_pack->p_startcode = p_h264_startcode;
    
    p_pack->b_slice = false;
    p_pack->p_frame = NULL;
    p_pack->b_frame_sps = false;
    p_pack->b_frame_pps = false;

    p_pack->b_header= false;
    p_pack->b_sps   = false;
    p_pack->b_pps   = false;
    for( i = 0; i < SPS_MAX; i++ )
        p_pack->pp_sps[i] = NULL;
    for( i = 0; i < PPS_MAX; i++ )
        p_pack->pp_pps[i] = NULL;

	p_pack->fmt_out.i_codec = VLC_CODEC_H264;
	p_pack->fmt_out.i_cat = VIDEO_ES;
	p_pack->fmt_out.i_extra = 0;
    p_pack->fmt_out.p_extra = NULL;	
    
    p_pack->slice.i_nal_type = -1;
    p_pack->slice.i_nal_ref_idc = -1;
    p_pack->slice.i_idr_pic_id = -1;
    p_pack->slice.i_frame_num = -1;
    p_pack->slice.i_frame_type = 0;
    p_pack->slice.i_pic_parameter_set_id = -1;
    p_pack->slice.i_field_pic_flag = 0;
    p_pack->slice.i_bottom_field_flag = -1;
    p_pack->slice.i_pic_order_cnt_lsb = -1;
    p_pack->slice.i_delta_pic_order_cnt_bottom = -1;

    p_pack->i_frame_dts = VLC_TS_INVALID;
    p_pack->i_frame_pts = VLC_TS_INVALID;
    
    

	return p_pack;
}

static void demux_close( h264_demux_t * p_sys)
{
	int i;
	if( p_sys->p_frame )
        block_ChainRelease( p_sys->p_frame );
    for( i = 0; i < SPS_MAX; i++ )
    {
        if( p_sys->pp_sps[i] )
            block_Release( p_sys->pp_sps[i] );
    }
    for( i = 0; i < PPS_MAX; i++ )
    {
        if( p_sys->pp_pps[i] )
            block_Release( p_sys->pp_pps[i] );
    }
    
	 block_BytestreamRelease( &p_sys->bytestream );
	 free(p_sys);
}

static block_t * demux(h264_demux_t * p_pack, block_t **pp_block)
{
	if( !pp_block || !*pp_block )
        return NULL;

    if( (*pp_block)->i_flags&(BLOCK_FLAG_DISCONTINUITY|BLOCK_FLAG_CORRUPTED) )
    {
        const bool b_broken = ( (*pp_block)->i_flags&BLOCK_FLAG_CORRUPTED ) != 0;
        if( b_broken )
        {
            p_pack->i_state = STATE_NOSYNC;
            block_BytestreamEmpty( &p_pack->bytestream );
            p_pack->i_offset = 0;
        }

        block_Release( *pp_block );
        return NULL;
    }

    block_BytestreamPush( &p_pack->bytestream, *pp_block );

    for( ;; )
    {
        bool b_used_ts;
        block_t *p_pic;

        switch( p_pack->i_state )
        {
        case STATE_NOSYNC:
            /* Find a startcode */
            if( !block_FindStartcodeFromOffset( &p_pack->bytestream, &p_pack->i_offset,
                                                p_pack->p_startcode, p_pack->i_startcode ) )
                p_pack->i_state = STATE_NEXT_SYNC;

            if( p_pack->i_offset )
            {
                block_SkipBytes( &p_pack->bytestream, p_pack->i_offset );
                p_pack->i_offset = 0;
                block_BytestreamFlush( &p_pack->bytestream );
            }

            if( p_pack->i_state != STATE_NEXT_SYNC )
                return NULL; /* Need more data */

            p_pack->i_offset = 1; /* To find next startcode */

        case STATE_NEXT_SYNC:
            /* Find the next startcode */
            if( block_FindStartcodeFromOffset( &p_pack->bytestream, &p_pack->i_offset,
                                               p_pack->p_startcode, p_pack->i_startcode ) )
            {
                if( !p_pack->b_flushing || !p_pack->bytestream.p_chain )
                    return NULL; /* Need more data */

                /* When flusing and we don't find a startcode, suppose that
                 * the data extend up to the end */
                block_ChainProperties( p_pack->bytestream.p_chain,
                                       NULL, &p_pack->i_offset, NULL );
                p_pack->i_offset -= p_pack->bytestream .i_offset;

                if( p_pack->i_offset <= (size_t)p_pack->i_startcode )
                    return NULL;
            }

            block_BytestreamFlush( &p_pack->bytestream );

            /* Get the new fragment and set the pts/dts */
            block_t *p_block_bytestream = p_pack->bytestream.p_block;

            p_pic = block_New( p_dec, p_pack->i_offset + p_pack->i_au_prepend );
            p_pic->i_pts = p_block_bytestream->i_pts;
            p_pic->i_dts = p_block_bytestream->i_dts;

            block_GetBytes( &p_pack->bytestream, &p_pic->p_buffer[p_pack->i_au_prepend],
                            p_pic->i_buffer - p_pack->i_au_prepend );
            if( p_pack->i_au_prepend > 0 )
                memcpy( p_pic->p_buffer, p_pack->p_au_prepend, p_pack->i_au_prepend );

            p_pack->i_offset = 0;

            /* Parse the NAL */
            if( p_pic->i_buffer < p_pack->i_au_min_size )
            {
                block_Release( p_pic );
                p_pic = NULL;
            }
            else
            {
                p_pic = PacketizeParse(p_pack, &b_used_ts, p_pic );
                if( b_used_ts )
                {
                    p_block_bytestream->i_dts = VLC_TS_INVALID;
                    p_block_bytestream->i_pts = VLC_TS_INVALID;
                }
            }

            if( !p_pic )
            {
                p_pack->i_state = STATE_NOSYNC;
                break;
            }
            
            if( VLC_SUCCESS )
            {
                p_pack->i_state = STATE_NOSYNC;
                block_Release( p_pic );
                break;
            }

            /* So p_block doesn't get re-added several times */
            *pp_block = block_BytestreamPop( &p_pack->bytestream );

            p_pack->i_state = STATE_NOSYNC;

            return p_pic;
        }
    }
}


static block_t *PacketizeParse( h264_demux_t * p_dec, bool *pb_ts_used, block_t *p_block )
{
 
    /* Remove trailing 0 bytes */
    while( p_block->i_buffer > 5 && p_block->p_buffer[p_block->i_buffer-1] == 0x00 )
        p_block->i_buffer--;

    return ParseNALBlock( p_dec, pb_ts_used, p_block );
}

/*****************************************************************************
 * ParseNALBlock: parses annexB type NALs
 * All p_frag blocks are required to start with 0 0 0 1 4-byte startcode
 *****************************************************************************/
static block_t *ParseNALBlock( h264_demux_t *p_dec, bool *pb_used_ts, block_t *p_frag )
{
    h264_demux_t *p_sys = p_dec;
    block_t *p_pic = NULL;

    const int i_nal_ref_idc = (p_frag->p_buffer[4] >> 5)&0x03;
    const int i_nal_type = p_frag->p_buffer[4]&0x1f;
    const mtime_t i_frag_dts = p_frag->i_dts;
    const mtime_t i_frag_pts = p_frag->i_pts;

    if( p_sys->b_slice && ( !p_sys->b_sps || !p_sys->b_pps ) )
    {
        block_ChainRelease( p_sys->p_frame );
        fprintf( stderr, "waiting for SPS/PPS\n" );

        /* Reset context */
        p_sys->slice.i_frame_type = 0;
        p_sys->p_frame = NULL;
        p_sys->b_frame_sps = false;
        p_sys->b_frame_pps = false;
        p_sys->b_slice = false;
        //cc_Flush( &p_sys->cc_next );
    }

    if( ( !p_sys->b_sps || !p_sys->b_pps ) &&
        i_nal_type >= NAL_SLICE && i_nal_type <= NAL_SLICE_IDR )
    {
        p_sys->b_slice = true;
        /* Fragment will be discarded later on */
    }
    else if( i_nal_type >= NAL_SLICE && i_nal_type <= NAL_SLICE_IDR )
    {
        slice_t slice;
        bool  b_new_picture;

        ParseSlice( p_dec, &b_new_picture, &slice, i_nal_ref_idc, i_nal_type, p_frag );

        /* */
        if( b_new_picture && p_sys->b_slice )
            p_pic = OutputPicture( p_dec );

        /* */
        p_sys->slice = slice;
        p_sys->b_slice = true;
    }
    else if( i_nal_type == NAL_SPS )
    {
        if( p_sys->b_slice )
            p_pic = OutputPicture( p_dec );
        p_sys->b_frame_sps = true;

        PutSPS( p_dec, p_frag );

        /* Do not append the SPS because we will insert it on keyframes */
        p_frag = NULL;
    }
    else if( i_nal_type == NAL_PPS )
    {
        if( p_sys->b_slice )
            p_pic = OutputPicture( p_dec );
        p_sys->b_frame_pps = true;

        PutPPS( p_dec, p_frag );

        /* Do not append the PPS because we will insert it on keyframes */
        p_frag = NULL;
    }
    else if( i_nal_type == NAL_AU_DELIMITER ||
             i_nal_type == NAL_SEI ||
             ( i_nal_type >= 13 && i_nal_type <= 18 ) )
    {
        if( p_sys->b_slice )
            p_pic = OutputPicture( p_dec );

        /* Parse SEI for CC support */
        if( i_nal_type == NAL_SEI )
        {
            ParseSei( p_dec, p_frag );
        }
        else if( i_nal_type == NAL_AU_DELIMITER )
        {
            if( p_sys->p_frame && (p_sys->p_frame->i_flags & BLOCK_FLAG_PRIVATE_AUD) )
            {
                block_Release( p_frag );
                p_frag = NULL;
            }
            else
            {
                p_frag->i_flags |= BLOCK_FLAG_PRIVATE_AUD;
            }
        }
    }

    /* Append the block */
    if( p_frag )
        block_ChainAppend( &p_sys->p_frame, p_frag );

    *pb_used_ts = false;
    if( p_sys->i_frame_dts <= VLC_TS_INVALID && 
        p_sys->i_frame_pts <= VLC_TS_INVALID )
    {
        p_sys->i_frame_dts = i_frag_dts;
        p_sys->i_frame_pts = i_frag_pts;
        *pb_used_ts = true;
    }
    return p_pic;
}

static block_t *OutputPicture( h264_demux_t *p_dec )
{
    h264_demux_t *p_sys = p_dec;
    block_t *p_pic;

    if( !p_sys->b_header && p_sys->slice.i_frame_type != BLOCK_FLAG_TYPE_I)
        return NULL;

    const bool b_sps_pps_i = p_sys->slice.i_frame_type == BLOCK_FLAG_TYPE_I &&
                             p_sys->b_sps &&
                             p_sys->b_pps;
    if( b_sps_pps_i || p_sys->b_frame_sps || p_sys->b_frame_pps )
    {
        block_t *p_head = NULL;
        if( p_sys->p_frame->i_flags & BLOCK_FLAG_PRIVATE_AUD )
        {
            p_head = p_sys->p_frame;
            p_sys->p_frame = p_sys->p_frame->p_next;
        }

        block_t *p_list = NULL;
        for( int i = 0; i < SPS_MAX && (b_sps_pps_i || p_sys->b_frame_sps); i++ )
        {
            if( p_sys->pp_sps[i] )
                block_ChainAppend( &p_list, block_Duplicate( p_sys->pp_sps[i] ) );
        }
        for( int i = 0; i < PPS_MAX && (b_sps_pps_i || p_sys->b_frame_pps); i++ )
        {
            if( p_sys->pp_pps[i] )
                block_ChainAppend( &p_list, block_Duplicate( p_sys->pp_pps[i] ) );
        }
        
        if( b_sps_pps_i && p_list )
            p_sys->b_header = true;
		
		if ( p_sys->b_header )
		{
			if ( p_sys->fmt_out.p_extra )
				free(p_sys->fmt_out.p_extra);
			block_ChainProperties(p_list,NULL,&p_sys->fmt_out.i_extra,NULL);
			p_sys->fmt_out.p_extra = malloc( p_sys->fmt_out.i_extra );
			block_ChainExtract(p_list,p_sys->fmt_out.p_extra,p_sys->fmt_out.i_extra);
		}
		
        if( p_head )
            p_head->p_next = p_list;
        else
            p_head = p_list;
        block_ChainAppend( &p_head, p_sys->p_frame );

        p_pic = block_ChainGather( p_head );
    }
    else
    {
        p_pic = block_ChainGather( p_sys->p_frame );
    }
    p_pic->i_dts = p_sys->i_frame_dts;
    p_pic->i_pts = p_sys->i_frame_pts;
    p_pic->i_length = 0;    /* FIXME */
    p_pic->i_flags |= p_sys->slice.i_frame_type;
    p_pic->i_flags &= ~BLOCK_FLAG_PRIVATE_AUD;

    p_sys->slice.i_frame_type = 0;
    p_sys->p_frame = NULL;
    p_sys->i_frame_dts = VLC_TS_INVALID;
    p_sys->i_frame_pts = VLC_TS_INVALID;
    p_sys->b_frame_sps = false;
    p_sys->b_frame_pps = false;
    p_sys->b_slice = false;


    return p_pic;
}

static void PutSPS( h264_demux_t *p_dec, block_t *p_frag )
{
    h264_demux_t *p_sys = p_dec;

    uint8_t *pb_dec = NULL;
    int     i_dec = 0;
    bs_t s;
    int i_tmp;
    int i_sps_id;

    CreateDecodedNAL( &pb_dec, &i_dec, &p_frag->p_buffer[5],
                     p_frag->i_buffer - 5 );

    bs_init( &s, pb_dec, i_dec );
    int i_profile_idc = bs_read( &s, 8 );
    p_dec->fmt_out.i_profile = i_profile_idc;
    /* Skip constraint_set0123, reserved(4) */
    bs_skip( &s, 1+1+1+1 + 4 );
    p_dec->fmt_out.i_level = bs_read( &s, 8 );
    /* sps id */
    i_sps_id = bs_read_ue( &s );
    if( i_sps_id >= SPS_MAX )
    {
        fprintf( stderr, "invalid SPS (sps_id=%d)\n", i_sps_id );
        free( pb_dec );
        block_Release( p_frag );
        return;
    }

    if( i_profile_idc == 100 || i_profile_idc == 110 ||
        i_profile_idc == 122 || i_profile_idc == 244 ||
        i_profile_idc ==  44 || i_profile_idc ==  83 ||
        i_profile_idc ==  86 )
    {
        /* chroma_format_idc */
        const int i_chroma_format_idc = bs_read_ue( &s );
        if( i_chroma_format_idc == 3 )
            bs_skip( &s, 1 ); /* separate_colour_plane_flag */
        /* bit_depth_luma_minus8 */
        bs_read_ue( &s );
        /* bit_depth_chroma_minus8 */
        bs_read_ue( &s );
        /* qpprime_y_zero_transform_bypass_flag */
        bs_skip( &s, 1 );
        /* seq_scaling_matrix_present_flag */
        i_tmp = bs_read( &s, 1 );
        if( i_tmp )
        {
            for( int i = 0; i < ((3 != i_chroma_format_idc) ? 8 : 12); i++ )
            {
                /* seq_scaling_list_present_flag[i] */
                i_tmp = bs_read( &s, 1 );
                if( !i_tmp )
                    continue;
                const int i_size_of_scaling_list = (i < 6 ) ? 16 : 64;
                /* scaling_list (...) */
                int i_lastscale = 8;
                int i_nextscale = 8;
                for( int j = 0; j < i_size_of_scaling_list; j++ )
                {
                    if( i_nextscale != 0 )
                    {
                        /* delta_scale */
                        i_tmp = bs_read_se( &s );
                        i_nextscale = ( i_lastscale + i_tmp + 256 ) % 256;
                        /* useDefaultScalingMatrixFlag = ... */
                    }
                    /* scalinglist[j] */
                    i_lastscale = ( i_nextscale == 0 ) ? i_lastscale : i_nextscale;
                }
            }
        }
    }

    /* Skip i_log2_max_frame_num */
    p_sys->i_log2_max_frame_num = bs_read_ue( &s );
    if( p_sys->i_log2_max_frame_num > 12)
        p_sys->i_log2_max_frame_num = 12;
    /* Read poc_type */
    p_sys->i_pic_order_cnt_type = bs_read_ue( &s );
    if( p_sys->i_pic_order_cnt_type == 0 )
    {
        /* skip i_log2_max_poc_lsb */
        p_sys->i_log2_max_pic_order_cnt_lsb = bs_read_ue( &s );
        if( p_sys->i_log2_max_pic_order_cnt_lsb > 12 )
            p_sys->i_log2_max_pic_order_cnt_lsb = 12;
    }
    else if( p_sys->i_pic_order_cnt_type == 1 )
    {
        int i_cycle;
        /* skip b_delta_pic_order_always_zero */
        p_sys->i_delta_pic_order_always_zero_flag = bs_read( &s, 1 );
        /* skip i_offset_for_non_ref_pic */
        bs_read_se( &s );
        /* skip i_offset_for_top_to_bottom_field */
        bs_read_se( &s );
        /* read i_num_ref_frames_in_poc_cycle */
        i_cycle = bs_read_ue( &s );
        if( i_cycle > 256 ) i_cycle = 256;
        while( i_cycle > 0 )
        {
            /* skip i_offset_for_ref_frame */
            bs_read_se(&s );
            i_cycle--;
        }
    }
    /* i_num_ref_frames */
    bs_read_ue( &s );
    /* b_gaps_in_frame_num_value_allowed */
    bs_skip( &s, 1 );

    /* Read size */
    p_dec->fmt_out.video.i_width  = 16 * ( bs_read_ue( &s ) + 1 );
    p_dec->fmt_out.video.i_height = 16 * ( bs_read_ue( &s ) + 1 );

    /* b_frame_mbs_only */
    p_sys->b_frame_mbs_only = bs_read( &s, 1 );
    p_dec->fmt_out.video.i_height *=  ( 2 - p_sys->b_frame_mbs_only );
    if( p_sys->b_frame_mbs_only == 0 )
    {
        bs_skip( &s, 1 );
    }
    /* b_direct8x8_inference */
    bs_skip( &s, 1 );

    /* crop */
    i_tmp = bs_read( &s, 1 );
    if( i_tmp )
    {
        /* left */
        bs_read_ue( &s );
        /* right */
        bs_read_ue( &s );
        /* top */
        bs_read_ue( &s );
        /* bottom */
        bs_read_ue( &s );
    }

    /* vui */
    i_tmp = bs_read( &s, 1 );
    if( i_tmp )
    {
        /* read the aspect ratio part if any */
        i_tmp = bs_read( &s, 1 );
        if( i_tmp )
        {
            static const struct { int w, h; } sar[17] =
            {
                { 0,   0 }, { 1,   1 }, { 12, 11 }, { 10, 11 },
                { 16, 11 }, { 40, 33 }, { 24, 11 }, { 20, 11 },
                { 32, 11 }, { 80, 33 }, { 18, 11 }, { 15, 11 },
                { 64, 33 }, { 160,99 }, {  4,  3 }, {  3,  2 },
                {  2,  1 },
            };
            int i_sar = bs_read( &s, 8 );
            int w, h;

            if( i_sar < 17 )
            {
                w = sar[i_sar].w;
                h = sar[i_sar].h;
            }
            else if( i_sar == 255 )
            {
                w = bs_read( &s, 16 );
                h = bs_read( &s, 16 );
            }
            else
            {
                w = 0;
                h = 0;
            }

            if( w != 0 && h != 0 )
            {
                p_dec->fmt_out.video.i_sar_num = w;
                p_dec->fmt_out.video.i_sar_den = h;
            }
            else
            {
                p_dec->fmt_out.video.i_sar_num = 1;
                p_dec->fmt_out.video.i_sar_den = 1;
            }
        }
    }

    free( pb_dec );

    /* We have a new SPS */
    if( !p_sys->b_sps )
        fprintf( stderr, "found NAL_SPS (sps_id=%d)\n", i_sps_id );
    p_sys->b_sps = true;

    if( p_sys->pp_sps[i_sps_id] )
        block_Release( p_sys->pp_sps[i_sps_id] );
    p_sys->pp_sps[i_sps_id] = p_frag;
}

static void PutPPS( h264_demux_t *p_dec, block_t *p_frag )
{
    h264_demux_t *p_sys = p_dec;
    bs_t s;
    int i_pps_id;
    int i_sps_id;

    bs_init( &s, &p_frag->p_buffer[5], p_frag->i_buffer - 5 );
    i_pps_id = bs_read_ue( &s ); // pps id
    i_sps_id = bs_read_ue( &s ); // sps id
    if( i_pps_id >= PPS_MAX || i_sps_id >= SPS_MAX )
    {
        fprintf( stderr, "invalid PPS (pps_id=%d sps_id=%d)\n", i_pps_id, i_sps_id );
        block_Release( p_frag );
        return;
    }
    bs_skip( &s, 1 ); // entropy coding mode flag
    p_sys->i_pic_order_present_flag = bs_read( &s, 1 );
    /* TODO */

    /* We have a new PPS */
    if( !p_sys->b_pps )
        fprintf( stderr, "found NAL_PPS (pps_id=%d sps_id=%d)\n", i_pps_id, i_sps_id );
    p_sys->b_pps = true;

    if( p_sys->pp_pps[i_pps_id] )
        block_Release( p_sys->pp_pps[i_pps_id] );
    p_sys->pp_pps[i_pps_id] = p_frag;
}

static void ParseSlice( h264_demux_t *p_dec, bool *pb_new_picture, slice_t *p_slice,
                        int i_nal_ref_idc, int i_nal_type, const block_t *p_frag )
{
    h264_demux_t *p_sys = p_dec;
    uint8_t *pb_dec;
    int i_dec;
    int i_first_mb, i_slice_type;
    slice_t slice;
    bs_t s;

    /* do not convert the whole frame */
    CreateDecodedNAL( &pb_dec, &i_dec, &p_frag->p_buffer[5],
                     __MIN( p_frag->i_buffer - 5, 60 ) );
    bs_init( &s, pb_dec, i_dec );

    /* first_mb_in_slice */
    i_first_mb = bs_read_ue( &s );

    /* slice_type */
    switch( (i_slice_type = bs_read_ue( &s )) )
    {
    case 0: case 5:
        slice.i_frame_type = BLOCK_FLAG_TYPE_P;
        break;
    case 1: case 6:
        slice.i_frame_type = BLOCK_FLAG_TYPE_B;
        break;
    case 2: case 7:
        slice.i_frame_type = BLOCK_FLAG_TYPE_I;
        break;
    case 3: case 8: /* SP */
        slice.i_frame_type = BLOCK_FLAG_TYPE_P;
        break;
    case 4: case 9:
        slice.i_frame_type = BLOCK_FLAG_TYPE_I;
        break;
    default:
        slice.i_frame_type = 0;
        break;
    }

    /* */
    slice.i_nal_type = i_nal_type;
    slice.i_nal_ref_idc = i_nal_ref_idc;

    slice.i_pic_parameter_set_id = bs_read_ue( &s );
    slice.i_frame_num = bs_read( &s, p_sys->i_log2_max_frame_num + 4 );

    slice.i_field_pic_flag = 0;
    slice.i_bottom_field_flag = -1;
    if( !p_sys->b_frame_mbs_only )
    {
        /* field_pic_flag */
        slice.i_field_pic_flag = bs_read( &s, 1 );
        if( slice.i_field_pic_flag )
            slice.i_bottom_field_flag = bs_read( &s, 1 );
    }

    slice.i_idr_pic_id = p_sys->slice.i_idr_pic_id;
    if( slice.i_nal_type == NAL_SLICE_IDR )
        slice.i_idr_pic_id = bs_read_ue( &s );

    slice.i_pic_order_cnt_lsb = -1;
    slice.i_delta_pic_order_cnt_bottom = -1;
    slice.i_delta_pic_order_cnt0 = 0;
    slice.i_delta_pic_order_cnt1 = 0;
    if( p_sys->i_pic_order_cnt_type == 0 )
    {
        slice.i_pic_order_cnt_lsb = bs_read( &s, p_sys->i_log2_max_pic_order_cnt_lsb + 4 );
        if( p_sys->i_pic_order_present_flag && !slice.i_field_pic_flag )
            slice.i_delta_pic_order_cnt_bottom = bs_read_se( &s );
    }
    else if( (p_sys->i_pic_order_cnt_type == 1) &&
             (!p_sys->i_delta_pic_order_always_zero_flag) )
    {
        slice.i_delta_pic_order_cnt0 = bs_read_se( &s );
        if( p_sys->i_pic_order_present_flag && !slice.i_field_pic_flag )
            slice.i_delta_pic_order_cnt1 = bs_read_se( &s );
    }
    free( pb_dec );

    /* Detection of the first VCL NAL unit of a primary coded picture
     * (cf. 7.4.1.2.4) */
    bool b_pic = false;
    if( slice.i_frame_num != p_sys->slice.i_frame_num ||
        slice.i_pic_parameter_set_id != p_sys->slice.i_pic_parameter_set_id ||
        slice.i_field_pic_flag != p_sys->slice.i_field_pic_flag ||
        slice.i_nal_ref_idc != p_sys->slice.i_nal_ref_idc )
        b_pic = true;
    if( (slice.i_bottom_field_flag != -1) &&
        (p_sys->slice.i_bottom_field_flag != -1) &&
        (slice.i_bottom_field_flag != p_sys->slice.i_bottom_field_flag) )
        b_pic = true;
    if( p_sys->i_pic_order_cnt_type == 0 &&
        ( slice.i_pic_order_cnt_lsb != p_sys->slice.i_pic_order_cnt_lsb ||
          slice.i_delta_pic_order_cnt_bottom != p_sys->slice.i_delta_pic_order_cnt_bottom ) )
        b_pic = true;
    else if( p_sys->i_pic_order_cnt_type == 1 &&
             ( slice.i_delta_pic_order_cnt0 != p_sys->slice.i_delta_pic_order_cnt0 ||
               slice.i_delta_pic_order_cnt1 != p_sys->slice.i_delta_pic_order_cnt1 ) )
        b_pic = true;
    if( ( slice.i_nal_type == NAL_SLICE_IDR || p_sys->slice.i_nal_type == NAL_SLICE_IDR ) &&
        ( slice.i_nal_type != p_sys->slice.i_nal_type || slice.i_idr_pic_id != p_sys->slice.i_idr_pic_id ) )
            b_pic = true;

    /* */
    *pb_new_picture = b_pic;
    *p_slice = slice;
}

static void ParseSei( h264_demux_t *p_dec, block_t *p_frag )
{
    h264_demux_t *p_sys = p_dec;
    uint8_t *pb_dec;
    int i_dec;

    /* */
    CreateDecodedNAL( &pb_dec, &i_dec, &p_frag->p_buffer[5], p_frag->i_buffer - 5 );
    if( !pb_dec )
        return;

    /* The +1 is for rbsp trailing bits */
    for( int i_used = 0; i_used+1 < i_dec; )
    {
        /* Read type */
        int i_type = 0;
        while( i_used+1 < i_dec )
        {
            const int i_byte = pb_dec[i_used++];
            i_type += i_byte;
            if( i_byte != 0xff )
                break;
        }
        /* Read size */
        int i_size = 0;
        while( i_used+1 < i_dec )
        {
            const int i_byte = pb_dec[i_used++];
            i_size += i_byte;
            if( i_byte != 0xff )
                break;
        }
        /* Check room */
        if( i_used + i_size + 1 > i_dec )
            break;

        /* Look for user_data_registered_itu_t_t35 */
        if( i_type == 4 )
        {
            static const uint8_t p_dvb1_data_start_code[] = {
                0xb5,
                0x00, 0x31,
                0x47, 0x41, 0x39, 0x34
            };
            const int      i_t35 = i_size;
            const uint8_t *p_t35 = &pb_dec[i_used];

            /* Check for we have DVB1_data() */
            //if( i_t35 >= 5 &&
            //    !memcmp( p_t35, p_dvb1_data_start_code, sizeof(p_dvb1_data_start_code) ) )
            //{
            //    cc_Extract( &p_sys->cc_next, true, &p_t35[3], i_t35 - 3 );
            //}
        }
        i_used += i_size;
    }

    free( pb_dec );
}


static block_t * demux_read( FILE* fd ){
	block_t *p_es = block_Alloc(H264_PACKET_SIZE);
	int bytes = fread(p_es->p_buffer,1,p_es->i_buffer,fd);
	//fprintf(stderr,"read h264 data : %d\n",bytes);
	if ( bytes > 0 ){
		p_es->i_buffer = bytes;
		return p_es;
	}
	block_Release(p_es);
	return NULL;
}

FILE * fd_ts = 0;
static void h264_ts_callback(void * p_private, unsigned char * p_ts_data , size_t i_size )
{
//	fprintf( stderr , " Receive h264 ts stream :%d\n",i_size);
	if ( fd_ts ) fwrite( p_ts_data,1,i_size,fd_ts);
}

static void demux_h264 ( const char * filename ){
	block_t *p_block_in, *p_block_out;
	uint32_t i_count_nals = 0;
	mtime_t i_dts = 0; 
	h264_demux_t * p_pack = demux_open();
	sout_mux_t * p_mux = soutOpen(NULL,h264_ts_callback,NULL);
	sout_input_t * p_h264_input = 0;
	FILE *fd = fopen( filename ,"rb");
	fd_ts = fopen("h264.ts","w+b");
	while ( (p_block_in = demux_read(fd) ) )
	{
		p_block_in->i_dts = VLC_TS_0;
		p_block_in->i_pts = VLC_TS_0;

		while( (p_block_out = demux(p_pack,&p_block_in)))
		while ( p_block_out ){
			block_t * p_next = p_block_out->p_next;
			
			p_block_out->p_next = NULL;
			
			int	 i_nal_type = p_block_out->p_buffer[4]&0x1f;

			fprintf(stderr,"demux h264 : %d startCode %d %d %d %d , naltype : %d nal_length : %d\n"
							,++i_count_nals
							,p_block_out->p_buffer[0]
							,p_block_out->p_buffer[1]
							,p_block_out->p_buffer[2]
							,p_block_out->p_buffer[3]
							,i_nal_type
							,p_block_out->i_buffer);
			
			if ( p_pack->b_header && p_h264_input == NULL ){
				fprintf(stderr ,"add new stream\n");
				p_h264_input = soutAddStream(p_mux,&p_pack->fmt_out);
			}
			p_block_out->i_dts = VLC_TS_0 + i_dts;
			p_block_out->i_pts = VLC_TS_0 + i_dts;
			if ( p_h264_input )
				sout_block_mux(p_h264_input,p_block_out);						
				//if ( fd_ts ) fwrite(p_block_out->p_buffer,1,p_block_out->i_buffer,fd_ts);
			p_block_out = p_next;
			i_dts += (int64_t)((double)1000000.0 / 15);
			
		}
	}
	fprintf(stderr,"video (%d x %d ) \n",p_pack->fmt_out.video.i_width
										,p_pack->fmt_out.video.i_height);
	if ( p_h264_input )
		soutDelStream(p_mux,p_h264_input);
	soutClose(p_mux);
	demux_close(p_pack);
	
	if ( fd )
		fclose(fd);
	if ( fd_ts )
		fclose(fd_ts);
}

int main( int argc , char ** argv)
{
	fprintf(stderr,"this is a test for mpeg ts stream...\n");
		
	demux_h264(argv[1]);
	
	fprintf(stderr,"demux h264 end\n");
	
	return 0;
}
