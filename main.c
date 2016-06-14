#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vlc_block.h"
#include "csa.h"
#include "tsmux.h"

int main( int argc , char ** argv)
{
	fprintf(stderr,"this is a test for mpeg ts stream...\n");

	char *p = "hello block_t";
	unsigned char *pp;
	csa_t *p_csa = 0;
	block_t *p_buffer = block_Alloc( strlen(p) );
	pp=p_buffer->p_buffer;
	p_buffer->p_buffer = (unsigned char *)p;
	fprintf(stderr,"test block_t : %s,%zu\n",(char*)p_buffer->p_buffer,p_buffer->i_buffer);
	p_buffer->p_buffer = pp;
	block_Release(p_buffer);
	
	p_csa = csa_New();
	
	csa_Delete(p_csa);
	
	sout_mux_t * p_mux = soutOpen(NULL,NULL,0);
	
	soutClose(p_mux);
	
	return 0;
}
