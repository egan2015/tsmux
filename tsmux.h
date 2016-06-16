#ifndef _TSMUX_H_
#define _TSMUX_H_ 1

#include "vlc_common.h"
#include "vlc_es.h"

typedef struct sout_mux_t  sout_mux_t;

typedef struct ts_stream_t ts_stream_t;

typedef struct sout_param_t sout_param_t;

typedef struct sout_input_t sout_input_t;

typedef void (* sout_ts_write_cb)(  void* p_private ,
									unsigned char *p_ts_data,
									size_t i_size);
                                   
sout_mux_t* soutOpen( sout_param_t * ,sout_ts_write_cb , void* );

void soutClose( sout_mux_t * );

int  sout_stream_mux( sout_input_t * , unsigned char * p_es_data , uint16_t i_size,
				  int64_t i_length, int64_t i_pts, int64_t i_dts, int64_t i_flags);
int  sout_block_mux(sout_input_t * , block_t *);
				  
sout_input_t *  soutAddStream( sout_mux_t* , es_format_t *p_fmt);

int  soutDelStream( sout_mux_t* , sout_input_t *);



#endif
