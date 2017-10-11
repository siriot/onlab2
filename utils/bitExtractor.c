#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define TYPE_2_WORD_COUNT_MASK	0x07FFFFFF
#define TYPE_2_READ				0x50000000


int compare(uint8_t *m, uint8_t *d1, uint8_t *d2)
{
	int i;
	for(i=0;i<4;i++)
		if( (d1[i] & m[i]) != (d2[i] & m[i]))
			return -1;
	return 0;
}

uint32_t convert(uint8_t *buf)
{
	uint32_t retval = 0;
	int i;
	for(i=0;i<4;i++)
	{
		retval <<= 8;
		retval += buf[i];
	}
	return retval;
}

int readUntil(uint8_t * pattern, uint8_t * mask, FILE *f)
{
	uint8_t buf[4] = {0,0,0,0};
	uint8_t *p = buf;
	uint8_t tmp;

	do 
	{
		// read next byte
		if(fread(&tmp,1,1,f) != 1)
			return -1;
		else
		{
			p[0] = p[1];
			p[1] = p[2];
			p[2] = p[3];
			p[3] = tmp;
		}
	} while( compare(mask,buf,pattern) );

	return 0;
}


int main(int argc, char **argv)
{
	FILE * fin, *fout;
	uint8_t buf[4];
	uint32_t stream_length_word;
	uint32_t i;

	uint8_t pattern[4] = {0x50, 0x00, 0x00, 0x00};
	uint8_t mask[4] = {0xF8, 0, 0, 0};

	uint8_t synch_pattern[4] = {0xaa,0x99,0x55,0x66};
	uint8_t synch_mask[4] = {0xff,0xff,0xff,0xff};

	if(argc  == 1)
	{
		fin = stdin;
		fout = stdout;
	}
	else
	{
		fin=fopen(argv[1],"rb");
		if(!fin)
		{
			fprintf(stderr,"Cannot open input file.\n");
			return -1;
		}

		if(argc == 2)
		{
			fout = stdout;
		}
		else
		{
			fout = fopen(argv[3],"wb");
			if(!fout)
			{
				fprintf(stderr,"Cannot open output file.\n");
				return -1;
			}
		}
	}

// files open,  lets skip the header
	if(readUntil(synch_pattern, synch_mask, fin))
	{
		fprintf(stderr,"Cannot find header end.\n");
		return -1;
	}


// read until 
	do	
	{
		// read next byte
		if(fread(buf,4,1,fin) != 1)
		{
			fprintf(stderr,"Cannot find TYPE 2 read command.\n");
			return -1;
		}
	}while( compare(mask,buf,pattern));
	//buf contains type 2 read command

	stream_length_word = convert(buf) & TYPE_2_WORD_COUNT_MASK;

	fprintf(stderr,"Extracting stream with %d words.\n",stream_length_word);

	for(i=0;i<stream_length_word;i++)
	{
		if(fread(buf,4,1,fin)!= 1)
		{
			fprintf(stderr,"Stream ended too early.\n");
			return -1;
		}
		fwrite(buf,4,1,fout);
	}

	fclose(fin);
	fclose(fout);
	fprintf(stderr,"Extraction ready.\n");

	return 0;
}