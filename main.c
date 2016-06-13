#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vlc_block.h"

int main( int argc , char ** argv)
{
	fprintf(stderr,"This is a test for MPEG TS Stream...\n");
	block_t *p_buffer = block_Alloc( 1024 );
	
	strcpy((char*)p_buffer->p_buffer,"hello block_t");
	
	fprintf(stderr,"Test block_t : %s,%zu\n",(char*)p_buffer->p_buffer,p_buffer->i_buffer);
	
	block_Release(p_buffer);
	
	return 0;
}
