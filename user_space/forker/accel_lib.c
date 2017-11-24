/*
 * accel_lib.c
 *
 *  Created on: 2017. febr. 27.
 *      Author: tibi
 */

#include "accel_lib.h"
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>


#define IOCTL_RELEASE 			0
#define IOCTL_REQUIRE 			1
#define IOCTL_AXI_READ			20
#define IOCTL_AXI_WRITE			3
#define IOCTL_DMA_START			8
#define IOCTL_DMA_WAIT_FOR_RDY	9

#define FPGA_MGR_FILE "/dev/fpga_mgr"


struct accel *accel_open(const char *acc_name)
{
	struct accel *acc;
	FILE  *sys_f;
	char path[128];
	int acc_id;
	int mgr_file;

// getting requested accelerator id

	// Determine the corresponding acc_id from the sysfs
	sprintf(path,"/sys/bus/fpga_virtual_bus/devices/%s/id",acc_name);

	sys_f = fopen(path,"r");
	if(!sys_f)
	{
		fprintf(stderr,"Cannot find accel id for %s.\n",acc_name);
		return NULL;
	}
	fscanf(sys_f,"%d",&acc_id);
	fclose(sys_f);


	acc = (struct accel*)malloc(sizeof(struct accel));
	if(!acc) 
	{
		return NULL;
	}

	mgr_file = open(FPGA_MGR_FILE,O_RDWR);
	if(mgr_file == -1)
	{
		free(acc);
		fprintf(stderr,"Cannot open fpga_mgr file.\n");
		return NULL;
	}

	acc->name = strdup(acc_name);
	acc->acc_id = acc_id;
	acc->mmap_buffer = NULL;
	acc->mmap_buffer_length = 0;
	acc->fpga_mgr_filedes = mgr_file;
	acc->slot_id = -1;

	return acc;

}

void accel_close(struct accel *acc)
{
	assert(acc);
	if(acc->name) 	free(acc->name);
	if(acc->mmap_buffer) 	accel_unmap(acc);
	if(acc->slot_id != -1) 	fpga_unlock(acc);
	close(acc->fpga_mgr_filedes);
	free(acc);
}

long fpga_getlock(struct accel *acc)
{
	assert(acc);
	long ret;
	if(acc->slot_id!=-1) return -1;

	ret = ioctl(acc->fpga_mgr_filedes,IOCTL_REQUIRE,acc->acc_id);

	if(ret < 0) 
	{
		acc->slot_id = -1;
	}	
	else 
		acc->slot_id = (int)ret;

	return ret;
}
long fpga_unlock(struct accel *acc)
{
	long ret;
	assert(acc);
	if(acc->slot_id == -1) return -1;
	ret = ioctl(acc->fpga_mgr_filedes,IOCTL_RELEASE,acc->slot_id);
	acc->slot_id = -1;
	return ret;
}


/*
 * Writes to the accelerator address space
 */
long  accel_write(struct accel *acc, uint32_t address, uint32_t data)
{
	assert(acc);
	// If no slot is owned, return
	if(acc->slot_id == -1) return -1;
 	return	ioctl(acc->fpga_mgr_filedes,(address<<16) | IOCTL_AXI_WRITE,data);
}

long accel_read(struct accel *acc, uint32_t address, uint32_t *buf)
{
	assert(acc);
	// If no slot is owned, return
	if(acc->slot_id == -1) return -1;
 	return	ioctl(acc->fpga_mgr_filedes,(address<<16) | IOCTL_AXI_READ,buf);
}

/*
 	Requeset DMA buffer from the driver
 	*/
uint8_t * accel_map(struct accel *acc, uint32_t buffer_size)
{
	assert(acc);
	uint32_t *b;
	b = (uint32_t *)mmap(NULL,buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, acc->fpga_mgr_filedes, 0);
	if(b == MAP_FAILED)
	{
		printf("MMAP operation failed. Error: %s\n",strerror(errno));
		return NULL;
	}
	acc->mmap_buffer = b;
	if(b)
		acc->mmap_buffer_length = 4096;
	else
		acc->mmap_buffer_length = 0;

	return (uint8_t*)acc->mmap_buffer;
}
void accel_unmap(struct accel *acc)
{
	assert(acc);
	if(acc->mmap_buffer == NULL) return;

	munmap(acc->mmap_buffer,acc->mmap_buffer_length);
	acc->mmap_buffer = NULL;
	acc->mmap_buffer_length = 0;
}


struct user_dma_buffer_desc
{
	uint32_t tx_offset;
	uint32_t rx_offset;
	uint32_t tx_length;
	uint32_t rx_length;
};

long accel_start_dma(struct accel *acc, unsigned int tx_offset, unsigned int tx_length, unsigned int rx_offset, unsigned int rx_length)
{
	assert(acc);
	struct user_dma_buffer_desc desc;

	if(acc->slot_id == -1)
		return -1;
	desc.tx_offset = tx_offset;
	desc.tx_length = tx_length;
	desc.rx_offset = rx_offset;
	desc.rx_length = rx_length;
	return ioctl(acc->fpga_mgr_filedes, IOCTL_DMA_START,&desc);
}
long accel_wait_dma_rdy(struct accel *acc)
{
	assert(acc);
	if(acc->slot_id == -1)
		return -1;
	return ioctl(acc->fpga_mgr_filedes, IOCTL_DMA_WAIT_FOR_RDY,0);
}
