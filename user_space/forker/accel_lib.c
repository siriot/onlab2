/*
 * accel_lib.c
 *
 *  Created on: 2017. febr. 27.
 *      Author: tibi
 */

#include "accel_lib.h"
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdio.h>

#define IOCTL_RELEASE 			0
#define IOCTL_REQUIRE 			1
#define IOCTL_AXI_READ			20
#define IOCTL_AXI_WRITE			3
#define IOCTL_DMA_SET_TX_BASE	4
#define IOCTL_DMA_SET_TX_LENGTH	5
#define IOCTL_DMA_SET_RX_BASE	6
#define IOCTL_DMA_SET_RX_LENGTH	7
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

// gettings requested accelerator id

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

	mgr_file = open(FPGA_MGR_FILE,O_RDONLY);
	if(mgr_file == -1)
	{
		free(acc);
		fprintf(stderr,"Cannot open fpga_mgr file.\n"),
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
	if(!acc)
		return;
	if(acc->name)
		free(acc->name);

	free(acc);
}

int fpga_getlock(struct accel *acc, struct timespec *time_array)
{
	if(!acc) return -1;
	if(acc->slot_id!=-1) return -1;

	clock_gettime(CLOCK_MONOTONIC_RAW,&(time_array[SLOT_GET_START]));
	acc->slot_id = ioctl(acc->fpga_mgr_filedes,SLOT_REQUIRE,acc->acc_id);
	clock_gettime(CLOCK_MONOTONIC_RAW,&(time_array[SLOT_GET_END]));

	if(acc->slot_id == -1)
	{
		fprintf(stderr,"Error at slot loading.\n");
		return -1;
	}
	return 0;
}
int fpga_unlock(struct accel *acc, struct timespec *time_array)
{
	int ret;
	if(!acc) return -1;
	if(acc->slot_id == -1) return -1;

	clock_gettime(CLOCK_MONOTONIC_RAW,&(time_array[SLOT_RELEASE_START]));
	ioctl(acc->fpga_mgr_filedes,SLOT_RELEASE,acc->slot_id);
	clock_gettime(CLOCK_MONOTONIC_RAW,&(time_array[SLOT_RELEASE_END]));
	acc->slot_id = -1;
	return ret;
}


/*
 * Writes to the
 */
int accel_write(struct accel *acc, uint32_t address, uint32_t data);
{
	uint32_t user_addr;
	// If no slot is owned, return
	if(acc->slot_id == -1) return -1;

 	return	ioctl(acc->fpga_mgr_filedes,(address<<16) | IOCTL_AXI_WRITE,data);
}

uint32_t accel_read(struct accel *acc, uint32_t address);
{
	uint32_t user_addr;
	// If no slot is owned, return
	if(acc->slot_id == -1) return -1;

 	return	ioctl(acc->fpga_mgr_filedes,(address<<16) | IOCTL_AXI_READ,0);
}


uint8_t * accel_map(struct accel *acc, uint32_t buffer_size);
void accel_unmap(struct accel *acc);


struct user_dma_buffer_desc
{
	u32 tx_offset;
	u32 rx_offset;
	u32 tx_len;
	u32 rx_len;
};

int accel_start_dma(struct accel *acc, unsigned int tx_offset, unsigned int tx_length, unsigned int rx_offset, unsigned int rx_length)
{
	struct user_dma_buffer_desc desc;
	if(acc->slot_id == -1)
		return -1;
	desc.tx_offset = tx_offset;
	desc.tx_length = tx_length;
	desc.rx_offset = rx_offset;
	desc.rx_length = rx_length;
	return ioctl(acc->fpga_mgr_filedes, IOCTL_DMA_WAIT_FOR_RDY,&desc);
}
int accel_wait_dma_rdy(struct accel *acc)
{
	if(acc->slot->id == -1)
		return -1;
	return ioctl(acc->fpga_mgr_filedes, IOCTL_DMA_WAIT_FOR_RDY,0);
}
