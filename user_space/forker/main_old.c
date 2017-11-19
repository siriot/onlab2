/*
 * main_old.c
 *
 *  Created on: 2017. márc. 1.
 *      Author: tibi
 */


#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <semaphore.h>
#include <time.h>

int main(int argc, char**argv)
{

  sem_t *sem;
  int hw_mgr;
  int buffer_file;
  FILE *bit_file;
  char *mmap_buffer=NULL;

  char *buffer = NULL;
  unsigned long bitfile_length = 0;

  //time measurement
  struct timespec start;
  struct timespec memcpy_start;
  struct timespec memcpy_end;
  struct timespec ioctl_start;
  struct timespec ioctl_end;
  struct timespec end;


  //getting the bitfile
  buffer = (char*)malloc(2048*1024);
  if(!buffer)
  {
	  printf("Not enough memory.\n");
	  return -1;
  }
  bit_file = fopen("timer.bit","rb");
  if(!bit_file) {printf("Cannot find the bitfile.\n");return -1;}
  // copying the data to the user space buffer
  bitfile_length = fread(buffer,1,2048*1024,bit_file);
  printf("Bitfile size : %lu B\n",bitfile_length);
  if(bitfile_length == 20148*1024) {printf("Bitfile is too big.\n");return -1;}
  fclose(bit_file);


  // trying to get the semaphore, in order to access to the hardware manager
  sem = sem_open("tt",O_CREAT,S_IRUSR,1);
  if(sem == SEM_FAILED)
    {
      printf("Semaphore cannot be created.\n");
      exit(-1);
    }

  int i;
  sem_getvalue(sem,&i);
  printf("semaphore value: %d",i);

  fflush(stdout);

  if(i ==0 && argc >1 && !strcmp(argv[1],"force"))
  {
	  printf("Resetting semaphore.\n");
	  sem_post(sem);
  }

  // cycle
for(i=0;i<10;i++)
{
clock_gettime(CLOCK_MONOTONIC,&start);


	  sem_wait(sem);

 // HW manager can be used
 buffer_file = open("/proc/hw_mgr/config_buffer",O_RDWR);
  mmap_buffer = mmap(NULL,bitfile_length,PROT_READ|PROT_WRITE, MAP_SHARED,buffer_file,0);
 if(mmap_buffer == MAP_FAILED)
 {
	 printf("Mapping is not successful.\n");
 }
 else
 {
	 // copying bit file data to the kernel buffer
	 unsigned long i;
	 clock_gettime(CLOCK_MONOTONIC,&memcpy_start);
	 memcpy(mmap_buffer,buffer,bitfile_length);
	 /*
	 for(i=0;i<bitfile_length;i++)
		 mmap_buffer[i] = buffer[i];*/
	 clock_gettime(CLOCK_MONOTONIC,&memcpy_end);
 }
 munmap(mmap_buffer,bitfile_length);
 close(buffer_file);

 hw_mgr = open("/proc/hw_mgr/program",O_RDWR);
 // starting fpga config
 clock_gettime(CLOCK_MONOTONIC,&ioctl_start);
 ioctl(hw_mgr,0,bitfile_length);
 clock_gettime(CLOCK_MONOTONIC,&ioctl_end);
 close(hw_mgr);


 if(sem_post(sem)==-1)
 {
	 printf("Error at releasing the semaphore.\n");
	 return -1;
 }
clock_gettime(CLOCK_MONOTONIC,&end);


printf(" start:\t%lu sec - %lu nsec \n m_st:\t%lu sec - %lu nsec \n m_end:\t %lu sec - %lu nsec\n i_st:\t %lu sec - %lu nsec \n i_end:\t %lu sec - %lu nsec \n end:\t %lu sec - %lu nsec\n",
		start.tv_sec ,start.tv_nsec,
		memcpy_start.tv_sec - start.tv_sec,memcpy_start.tv_nsec - start.tv_nsec,
		memcpy_end.tv_sec - memcpy_start.tv_sec,memcpy_end.tv_nsec -memcpy_start.tv_nsec,
		ioctl_start.tv_sec - memcpy_end.tv_sec,ioctl_start.tv_nsec - memcpy_end.tv_nsec,
		ioctl_end.tv_sec - ioctl_start.tv_sec,ioctl_end.tv_nsec - ioctl_start.tv_nsec,
		end.tv_sec - ioctl_end.tv_sec,end.tv_nsec - ioctl_end.tv_nsec);

}

 free(buffer);

  return 0;
}
