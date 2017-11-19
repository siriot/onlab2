/*
 * accel_lib.h
 *
 *  Created on: 2017. febr. 27.
 *      Author: tibi
 */
#ifndef ACCEL_LIB_H_
#define ACCEL_LIB_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <time.h>

#include <sys/mman.h>

struct accel
{
	char *name;
	int acc_id;

	uint32_t *mmap_buffer;
	unsigned int mmap_buffer_length;

	int slot_id;

	int fpga_mgr_filedes;
};

enum timestamps {ACCEL_OPEN_START,ACCEL_OPEN_END,SLOT_GET_START,SLOT_GET_END,SLOT_RELEASE_START,SLOT_RELEASE_END,MAX_INDEX};


struct accel *accel_open(const char *acc_name);
void accel_close(struct accel *acc);

int accel_write(struct accel *acc, uint32_t address, uint32_t data);
uint32_t accel_read(struct accel *acc, uint32_t address);

uint8_t * accel_map(struct accel *acc, uint32_t buffer_size);
void accel_unmap(struct accel *acc);

// ioctl operations
int fpga_getlock(struct accel *acc, struct timespec *time_array);
int fpga_unlock(struct accel *acc,  struct timespec *time_array);
int accel_start_dma(struct accel *acc, unsigned int tx_offset, unsigned int tx_length, unsigned int rx_offset, unsigned int rx_length);
int accel_wait_dma_rdy(struct accel *acc);

#endif /* ACCEL_LIB_H_ */
