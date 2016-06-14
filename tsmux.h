#ifndef _TSMUX_H_
#define _TSMUX_H_ 1

#include "vlc_common.h"

typedef struct sout_mux_t  sout_mux_t;

typedef struct ts_stream_t ts_stream_t;

typedef struct sout_param_t sout_param_t;

typedef struct sout_input_t sout_input_t;

sout_mux_t* soutMuxOpen( sout_param_t * );

void soutMuxClose( sout_mux_t * );
int  soutMuxWrite( sout_mux_t * , unsigned char * p_data , uint16_t i_size,
				  int64_t i_length, int64_t i_pts, int64_t i_dts, int64_t i_flags);
int  soutAddStream( sout_mux_t* , es_format_t *p_fmt);
int  soutDelStream( sout_mux_t* , ts_stream_t *);


#endif
