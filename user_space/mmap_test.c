#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <error.h>


// defines
#define IOCTL_RELEASE 			0
#define IOCTL_REQUIRE 			1
#define IOCTL_AXI_READ			20
#define IOCTL_AXI_WRITE			3
#define IOCTL_DMA_START			8
#define IOCTL_DMA_WAIT_FOR_RDY	9


// types
struct user_dma_buffer_desc
{
	uint32_t tx_offset;
	uint32_t rx_offset;
	uint32_t tx_length;
	uint32_t rx_length;
};


// globals
uint32_t pattern = 0x11223344;

int main(int argc, char **argv)
{
	int f;
	uint8_t *tx_buf = NULL;
	uint8_t *rx_buf = NULL;
	int i;
	struct user_dma_buffer_desc rqst;
	int ret;

	f = open("/dev/fpga_mgr", O_RDWR);

	// request accelerator
	ret = ioctl(f,1,1);
	printf("Accel request retval: %d\n",ret);


	tx_buf = mmap(NULL,1024,PROT_READ | PROT_WRITE, MAP_SHARED, f, 0);
	if(!tx_buf)
	{
		printf("MMAP failed\n");
		return -1;
	}
	memset(tx_buf,0,128);

	for(i=0;i<64;i+=4)
		memcpy(&tx_buf[i],&pattern,4);
	msync(tx_buf,64,MS_SYNC);

	// start dma transfer
	rqst.tx_offset = 0;
	rqst.tx_length = 64;
	rqst.rx_offset = 64;
	rqst.rx_length = 64;
	ret = ioctl(f,IOCTL_DMA_START,&rqst);
	printf("DMA start response: %d\n",ret);

	// wait for dma ready
	ret = ioctl(f,IOCTL_DMA_WAIT_FOR_RDY,0);
	if(ret)
		printf("Wait for dma error code: %s\n",strerror(ret));

	// read
	rx_buf = tx_buf + 64;

	printf("Sent data:\n");
	for(i=0;i<64;i++)
		printf("%02x",tx_buf[i]);

	printf("Received data:\n");
	for(i=0;i<64;i++)
		printf("%02x",rx_buf[i]);

	munmap(tx_buf,1024);
	close(f);

	return 0;
}