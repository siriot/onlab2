/*
 * forker.c
 *
 *  Created on: 2017. márc. 5.
 *      Author: tibi
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>

#include <sys/wait.h>
#include <sys/types.h>
#include <stdint.h>

#include "accel_lib.h"


#define CONFIG_NUM 2
#define SEM_NAME "test_sem"
#define CYCLE_NUM 5


struct timing_data_t{
	struct timespec open_start;
	struct timespec open_end;

	struct timespec mmap_start;
	struct timespec mmap_end;
	
	struct timespec get_start[CYCLE_NUM];
	struct timespec get_end[CYCLE_NUM];

	struct timespec dma_start[CYCLE_NUM];
	struct timespec dma_end[CYCLE_NUM];

	struct timespec release_start[CYCLE_NUM];
	struct timespec release_end[CYCLE_NUM];
};

typedef int (*process_func_t)(struct accel *acc, struct timespec *dma_start,struct timespec *dma_end);
int process_copy(struct accel *acc, struct timespec *dma_start,struct timespec *dma_end);
int process_swapper(struct accel *acc, struct timespec *dma_start,struct timespec *dma_end);


// globals
char *accel_names[CONFIG_NUM] = {
		"accel_copy",
		"accel_swapper"
};

sem_t *sem;


int run_child(int no);

int main(int argc, char **argv)
{
	int i;
	pid_t ch_pid;
	int CHILD_NUM;
	int stat;
	int cntr;

	if(argc==1)
	{
		CHILD_NUM = 15;
	}
	else
		CHILD_NUM =atoi(argv[1]);


	// init timing semaphore
	sem_unlink(SEM_NAME);

	sem = sem_open(SEM_NAME,O_CREAT,S_IRUSR,0);
	if(!sem)
	{
		fprintf(stderr,"Cannot create process synchronizer semaphore.\n");
		exit(-1);
	}
	sem_close(sem);

	// step 1 - create processes
	for(i = 0;i<CHILD_NUM;i++)
	{
		ch_pid = fork();
		if(ch_pid == 0)
		{
			run_child(i);
			_exit(0);
		}
		else if(ch_pid < 0)
		{
			fprintf(stderr,"Forking error.\n");
			exit(-1);
		}
	}

	// step 2 start process execution
	printf("Forking ready.\n");
	// start children execution
	sem = sem_open(SEM_NAME,0);
	sleep(1);
	sem_post(sem);
	//wait until all children exit
	for(cntr = 0;cntr<CHILD_NUM;cntr++)
		waitpid(-1,&stat,0);

	printf("Children execution is ready.\n");
	sem_close(sem);
	sem_unlink(SEM_NAME);
	return 0;
}


#define WAIT_TIME 50000

int run_child(int no)
{
	int i;
	char log_file_name[100];
	FILE *log;
	struct timing_data_t timing_data;
	process_func_t func = NULL;
	int my_accel;
	char *config_name;

	my_accel = no % CONFIG_NUM;
	config_name = accel_names[my_accel];

	switch(my_accel)
	{
	case 0: func = process_copy; break;
	case 1: func = process_swapper; break;
	default: break;
	}

	memset(&timing_data,0,sizeof(struct timing_data_t));

	sprintf(log_file_name,"/home/root/measure/%d_log.txt",getpid());
	log = fopen(log_file_name,"w");
	if(!log)
	{
		fprintf(stderr,"CANNOT OPEN LOG FILE WITH PATH: %s.\n",log_file_name);
		return -1;
	}
	fprintf(log,"#pid%d\n#name%s\n",getpid(),accel_names[my_accel]);


	fprintf(stdout,"Starting child process with accelerator: %s.\n",accel_names[my_accel]);

	// child
	//wait for start signal
	sem = sem_open(SEM_NAME,0);
	if(!sem)
	{
		fprintf(stderr,"Child: cannot open semaphore.\n");
		_exit(-1);
	}
	sem_wait(sem);
	sem_post(sem);


/* ACCEL USAGE */
{
	struct accel *acc;
	long ret;
	uint8_t *mmap_addr;

	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data.open_start));
	acc = accel_open(config_name);
	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data.open_end));
	if(!acc)
	{
		printf("Cannot open the accelerator.\n");
		fclose(log);
		return -1;
	}

	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data.mmap_start));
	mmap_addr = accel_map(acc,4096);
	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data.mmap_end));
	if(mmap_addr==NULL)
	{
		printf("Cannot mmap the accelerator.\n");
		fclose(log);
		return -1;
	}
	
	for(i=0;i<CYCLE_NUM;i++)
	{
		clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data.get_start[i]));
		ret = fpga_getlock(acc);
		clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data.get_end[i]));
		if(ret)
		{
			fprintf(stderr,"Error at fpga locking.\n");
			break;
		}

// perform accel specific operation
		func(acc, &(timing_data.dma_start[i]),&(timing_data.dma_end[i]));
// operation END

		clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data.release_start[i]));
		fpga_unlock(acc);
		clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data.release_end[i]));
		
		usleep(WAIT_TIME);
	}
}


/* ACCEL USAGE END */

	//exit
	sem_close(sem);

	// save timing info to the log file
	fprintf(log,"%ld_%ld\n%ld_%ld\n%ld_%ld\n%ld_%ld\n", timing_data.open_start.tv_sec, 	timing_data.open_start.tv_nsec, \
														timing_data.open_end.tv_sec, 	timing_data.open_end.tv_nsec,    \
														timing_data.mmap_start.tv_sec, 	timing_data.mmap_start.tv_nsec,   \
														timing_data.mmap_end.tv_sec, 	timing_data.mmap_end.tv_nsec);

	for(i=0;i<CYCLE_NUM;i++)
			fprintf(log,"%ld_%ld ",timing_data.get_start[i].tv_sec, timing_data.get_start[i].tv_nsec);
		fprintf(log,"\n");

	for(i=0;i<CYCLE_NUM;i++)
			fprintf(log,"%ld_%ld ",timing_data.get_end[i].tv_sec, timing_data.get_end[i].tv_nsec);
		fprintf(log,"\n");

	for(i=0;i<CYCLE_NUM;i++)
			fprintf(log,"%ld_%ld ",timing_data.dma_start[i].tv_sec, timing_data.dma_start[i].tv_nsec);
		fprintf(log,"\n");

	for(i=0;i<CYCLE_NUM;i++)
			fprintf(log,"%ld_%ld ",timing_data.dma_end[i].tv_sec, timing_data.dma_end[i].tv_nsec);
		fprintf(log,"\n");

	for(i=0;i<CYCLE_NUM;i++)
			fprintf(log,"%ld_%ld ",timing_data.release_start[i].tv_sec, timing_data.release_start[i].tv_nsec);
		fprintf(log,"\n");

	for(i=0;i<CYCLE_NUM;i++)
			fprintf(log,"%ld_%ld ",timing_data.release_end[i].tv_sec, timing_data.release_end[i].tv_nsec);
		fprintf(log,"\n");
	
	fclose(log);

	return 0;
}


/**********************************************************
				ACCELERATOR HANDLER FUNCTIONS
***********************************************************/
#define BUFFER_LEN_BYTE (4096U)
#define BUFFER_LEN_WORD (BUFFER_LEN_BYTE/4)
int process_copy(struct accel *acc, struct timespec *dma_start,struct timespec *dma_end)
{
	uint32_t *tx_buffer = acc->mmap_buffer; // first half of the buffer
	uint32_t *rx_buffer = tx_buffer + BUFFER_LEN_WORD/2; // second half of the buffer
	uint8_t *tb = (uint8_t*) tx_buffer;
	uint8_t *rb = (uint8_t*) rx_buffer;
	uint32_t p = 0x11223344;
	int ret;
	int i;

	// fill tx buffer with sample data
	for(i=0;i<1024/4;i++)
	{
		tx_buffer[i] = p ^ 0x00AA5500;
		p<<=1;
	}
	clock_gettime(CLOCK_MONOTONIC_RAW,dma_start);
	ret = accel_start_dma(acc, 0, 1024, BUFFER_LEN_BYTE/2, 1024);
	if(ret) fprintf(stderr,"DMA START RETURN VALUE: %d.\n",ret);
	ret = accel_wait_dma_rdy(acc);
	clock_gettime(CLOCK_MONOTONIC_RAW,dma_end);
	if(ret) fprintf(stderr,"DMA START RETURN VALUE: %d.\n",ret);

	//check data correctness
	for(i=0;i<1024;i++)
	{
		if(rb[i] != tb[i&0xFFFFFFC0]) fprintf(stderr,"Data error at byte %d.\n",i);
	}
	return 0;

}
int process_swapper(struct accel *acc, struct timespec *dma_start,struct timespec *dma_end)
{
		uint32_t *tx_buffer = acc->mmap_buffer; // first half of the buffer
	uint32_t *rx_buffer = tx_buffer + BUFFER_LEN_WORD/2; // second half of the buffer
	uint8_t *tb = (uint8_t*) tx_buffer;
	uint8_t *rb = (uint8_t*) rx_buffer;
	uint32_t p = 0x11223344;
	int ret;
	int i;

	// fill tx buffer with sample data
	for(i=0;i<1024/4;i++)
	{
		tx_buffer[i] = p ^ 0x00AA5500;
		p<<=1;
	}
	clock_gettime(CLOCK_MONOTONIC_RAW,dma_start);
	ret = accel_start_dma(acc, 0, 1024, BUFFER_LEN_BYTE/2, 1024);
	if(ret) fprintf(stderr,"DMA START RETURN VALUE: %d.\n",ret);
	ret = accel_wait_dma_rdy(acc);
	clock_gettime(CLOCK_MONOTONIC_RAW,dma_end);
	if(ret) fprintf(stderr,"DMA START RETURN VALUE: %d.\n",ret);

	//check data correctness
	for(i=0;i<1024;i++)
	{
		uint32_t packet_base = i & 0xFFFFFFC0;
		uint32_t byte_index = i &  0x00000040;
		if(rb[i] != tb[packet_base + 63-byte_index]) fprintf(stderr,"Data error at byte %d.\n",i);
	}
	return 0;
}