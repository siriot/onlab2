#include "accel_lib.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <sys/types.h>


#include <time.h>

#define CYCLE_NUM 5
#define TERM_DELAY 1000000L

#define MATRIX_WAIT_TIME 300000
#define MATRIX_OP_TIME   200000
int process_matrix()
{
	int i,j;
	FILE *log;
	unsigned matrix_data[9] = {1,2,3,4,5,6,7,8,9};
	unsigned matrix_result[9];
	const char * config_name = "matrix_mul";
	char log_file_name[100];
	struct accel *my_acc;
	struct timespec timing_data[MAX_INDEX];

	// init logging system
	sprintf(log_file_name,"/home/root/%d_log.txt",getpid());
	log = fopen(log_file_name,"w");
	fprintf(log,"#pid%d\n#name%s\n",getpid(),config_name);

	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data[ACCEL_OPEN_START]));
	my_acc = accel_open(config_name);
	if(!my_acc)
	{
		printf("Cannot open the accelerator.\n");
		fclose(log);
		return -1;
	}
	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data[ACCEL_OPEN_END]));



	for(i=0;i<CYCLE_NUM;i++)
	{
		if(fpga_getlock(my_acc, timing_data))
		{
			printf("Error at fpga locking.\n");
			break;
		}

		usleep(MATRIX_WAIT_TIME);
		// perform matrix multiplication
		// fill matrix 1
		/*
		memcpy((void*)my_acc->axi_gp0_base+MATRIX_1_BASE_OFF,&matrix_data,sizeof(unsigned)*9);
		memcpy((void*)my_acc->axi_gp0_base+MATRIX_2_BASE_OFF,&matrix_data,sizeof(unsigned)*9);
		matrix_start(my_acc);
		while(!matrix_is_done(my_acc))
			usleep(10000);
		usleep(1000);
		memcpy((void*)&matrix_result,(void*)my_acc->axi_gp0_base+MATRIX_3_BASE_OFF,sizeof(int)*9);
		for(j=0;j<9;j++)
			printf("%d, ",matrix_result[j]);
		printf("\n");
		*/


		fpga_unlock(my_acc, timing_data);


		for(j=0;j<MAX_INDEX;j++)
			fprintf(log,"%ld %ld ",timing_data[j].tv_sec, timing_data[j].tv_nsec);
		fprintf(log,"\n");
		usleep(MATRIX_WAIT_TIME);
	}

	usleep(TERM_DELAY);
	accel_close(my_acc);

	fclose(log);

	return 0;
}

#define TIMER_OP_TIME 40000
#define TIMER_WAIT_TIME 20000
int process_timer()
{
	int i,j;
	FILE *log;
	const char *config_name = "timer";
	char log_file_name[100];
	struct accel *my_acc;
	struct timespec timing_data[MAX_INDEX];

	// init logging system
	sprintf(log_file_name,"/home/root/%d_log.txt",getpid());
	log = fopen(log_file_name,"w");
	fprintf(log,"#pid%d\n#name%s\n",getpid(),config_name);

	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data[ACCEL_OPEN_START]));
	my_acc = accel_open(config_name);
	if(!my_acc)
	{
		fprintf(stderr,"Cannot open the accelerator.\n");
		fclose(log);
	}
	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data[ACCEL_OPEN_END]));


	for(i=0;i<CYCLE_NUM;i++)
	{
		if(fpga_getlock(my_acc, timing_data))
		{
			printf("Error at fpga locking.\n");
			break;
		}
		usleep(TIMER_OP_TIME);
		fpga_unlock(my_acc, timing_data);
		usleep(TIMER_WAIT_TIME);

		for(j=0;j<MAX_INDEX;j++)
			fprintf(log,"%ld %ld ",timing_data[j].tv_sec, timing_data[j].tv_nsec);
		fprintf(log,"\n");
		usleep(TIMER_WAIT_TIME);
	}

	usleep(TERM_DELAY);
	accel_close(my_acc);

	fclose(log);

	return 0;
}

#define TIMER2_OP_TIME 25000
#define TIMER2_WAIT_TIME 20000
int process_timer2()
{
	int i,j;
	FILE *log;
	const char *config_name = "timer2";
	char log_file_name[100];
	struct accel *my_acc;
	struct timespec timing_data[MAX_INDEX];

	// init logging system
	sprintf(log_file_name,"/home/root/%d_log.txt",getpid());
	log = fopen(log_file_name,"w");
	fprintf(log,"#pid%d\n#name%s\n",getpid(),config_name);

	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data[ACCEL_OPEN_START]));
	my_acc = accel_open(config_name);
	if(!my_acc)
	{
		fprintf(stderr,"Cannot open the accelerator.\n");
		fclose(log);
	}
	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data[ACCEL_OPEN_END]));


	for(i=0;i<CYCLE_NUM;i++)
	{
		if(fpga_getlock(my_acc, timing_data))
		{
			printf("Error at fpga locking.\n");
			break;
		}
		usleep(TIMER2_OP_TIME);
		fpga_unlock(my_acc, timing_data);
		usleep(TIMER2_WAIT_TIME);

		for(j=0;j<MAX_INDEX;j++)
			fprintf(log,"%ld %ld ",timing_data[j].tv_sec, timing_data[j].tv_nsec);
		fprintf(log,"\n");

	}

	usleep(TERM_DELAY);
	accel_close(my_acc);

	fclose(log);

	return 0;
}
/*
// TODO
int process_fir()
{
	int i,j,k;
	FILE *log;
	unsigned matrix_data[9] = {1,2,3,4,5,6,7,8,9};
	unsigned matrix_result[9];
	const char * config_name = "matrix.bit";
	char log_file_name[100];
	struct accel *my_acc;
	struct timespec timing_data[MAX_INDEX];

	// init logging system
	sprintf(log_file_name,"/home/root/%d_log.txt",getpid());
	log = fopen(log_file_name,"w");
	fprintf(log,"#pid%d\n#name%s\n",getpid(),config_name);

	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data[ACCEL_OPEN_START]));
	my_acc = accel_open(config_name);
	if(!my_acc)
	{
		printf("Cannot open the accelerator.\n");
		fclose(log);
		return -1;
	}
	clock_gettime(CLOCK_MONOTONIC_RAW,&(timing_data[ACCEL_OPEN_END]));



	for(i=0;i<5;i++)
	{
		if(fpga_getlock(my_acc, timing_data))
		{
			printf("Error at fpga locking.\n");
			return -1;
		}
		// perform matrix multiplication
		// fill matrix 1
		memcpy((void*)my_acc->axi_gp0_base+MATRIX_1_BASE_OFF,&matrix_data,sizeof(unsigned)*9);
		memcpy((void*)my_acc->axi_gp0_base+MATRIX_2_BASE_OFF,&matrix_data,sizeof(unsigned)*9);
		matrix_start(my_acc);
		while(!matrix_is_done(my_acc))
			usleep(10000);
		usleep(1000);
		memcpy((void*)&matrix_result,(void*)my_acc->axi_gp0_base+MATRIX_3_BASE_OFF,sizeof(int)*9);
		for(j=0;j<9;j++)
			printf("%d, ",matrix_result[j]);
		printf("\n");


		fpga_unlock(my_acc, timing_data);


		for(j=0;j<MAX_INDEX;j++)
			fprintf(log,"%ld %ld ",timing_data[j].tv_sec, timing_data[j].tv_nsec);
		fprintf(log,"\n");
		usleep(10000);
	}

	accel_close(my_acc);

	fclose(log);

	return 0;
}
*/
/*
#define LENGTH 100
int test_fir()
{
	struct accel acc;
	uint32_t tx[LENGTH] = {1};
	uint32_t rx[LENGTH];
	int i;

	// open /dev/mem, and remap the axi gp regions into user space
	acc.dev_mem_filedes =open("/dev/mem", O_RDWR | O_SYNC);
	if(acc.dev_mem_filedes < 0)
	{
		fprintf(stderr,"Cannot open /dev/mem.\n");
		accel_close(acc);
		return NULL;
	}
	acc.axi_gp0_base =(void*)mmap(NULL,AXI_GP_LENGTH,PROT_READ|PROT_WRITE,MAP_SHARED,acc.dev_mem_filedes,AXI_GP_0_BASE);
	if(!acc->axi_gp0_base || !acc->axi_gp1_base)
	{
		fprintf(stderr,"Cannot mmap the axi gp memory regions.\n");
		return NULL;
	}


	// test the driver
	if(fifo_init(&acc)<0) return -1;
	if(!fir_is_ready(&acc))
	{
		fprintf(stderr,"Error: fir is not ready.\n");
		return -1;
	}
	// starting fir filter
	fir_start(&acc);

	printf("TX fifo vacancy: %d.\n",fifo_get_transmit_vacancy(&acc));
	if(fifo_transmit_packet(tx,LENGTH,&acc))
	{
		fprintf(stderr,"Error at tx fifo filling.\n");
		return -1;
	}
	sleep(2);
	if(!fir_is_done(&acc))
	{
		fprintf(stderr,"Fir filter timeout.\n");
		return -1;
	}

	// getting the filtered data back
	if(fifo_recv_len(&acc)==0)
	{
		fprintf(stderr,"Data is not in the recv fifo.\n");
		return -1;
	}

	fifo_recv(rx,&acc);

	// printing the data out
	for(i=0;i<LENGTH;i++)
		print("%ll")




	munmap(acc.axi_gp0_base,AXI_GP_LENGTH);
	close(acc.dev_mem_filedes);

}
*/
