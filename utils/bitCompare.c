#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main(int argc, char **argv)
{
	FILE * fmask, *fin1, *fin2; 
	uint32_t buf_mask = 0;
	uint32_t buf_in1 = 0;
	uint32_t buf_in2 = 0;
	uint32_t wc = 0;

	if(argc < 4)
	{
		printf("Not enough arguments.\n");
		printf("Usage: <program name> <mask file> <input file 1> <input file 2>.\n");
		return -1;
	}
	fmask = fopen(argv[1],"rb");
	fin1 = fopen(argv[2],"rb");
	fin2 = fopen(argv[3],"rb");

	if(fmask == NULL || fin1 == NULL || fin2 == NULL)
	{
		printf("Cannot open input files.\n");
		return -1;
	}


	while(fread(&buf_mask,4,1,fmask)==1 && fread(&buf_in1,4,1,fin1)==1 && fread(&buf_in2,4,1,fin2)==1)
	{
		if( (buf_in1 & buf_mask) != (buf_in2 & buf_mask))
		{
			printf("Difference found at word address: 0x%08x (byte address: 0x%08x).\n",wc,wc*4);
			return -1;
		}
		wc++;
	}
	printf("Stream check finished. The 2 files are identical. Processed word num: %d (0x%08x).\n",wc,wc);

	fclose(fmask);
	fclose(fin1);
	fclose(fin2);
	return 0;
}