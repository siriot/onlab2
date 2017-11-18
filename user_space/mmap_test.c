#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>

uint32_t pattern = 0x12121212;

int main(int argc, char **argv)
{
	int f;
	uint8_t *tx_buf = NULL;
	uint8_t *rx_buf = NULL;
	int i;

	f = open("/dev/fpga_mgr", O_RDWR);

	ioctl(f,1,0);

	tx_buf = mmap(NULL,1024,PROT_READ | PROT_WRITE, MAP_SHARED, f, 0);

	memset(tx_buf,0,128);

	for(i=0;i<64;i+=4)
		memcpy(&tx_buf[i],&pattern,4);
	msync(tx_buf,64,MS_SYNC);

	sleep(1);

	ioctl(f,4,0);
	ioctl(f,5,64);
	ioctl(f,6,64);
	ioctl(f,7,64);
	ioctl(f,8,0);

	sleep(120);

	// read
	rx_buf = tx_buf + 64;
	msync(rx_buf,64,MS_SYNC);

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