#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vlc_block.h"

int main( int argc , char ** argv)
{
	fprintf(stderr,"This is a test for MPEG TS Stream...\n");
	
  char *p = "hello block_t";
  unsigned char *pp;
	block_t *p_buffer = block_Alloc( strlen(p) );
  pp=p_buffer->p_buffer;
	p_buffer->p_buffer = (unsigned char *)p;
  fprintf(stderr,"Test block_t : %s,%zu\n",(char*)p_buffer->p_buffer,p_buffer->i_buffer);
	p_buffer->p_buffer = pp;
	block_Release(p_buffer);
	
	return 0;
}
