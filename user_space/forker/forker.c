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


sem_t *sem;


#define CONFIG_NUM 3

#define SEM_NAME "test_sem"

extern int process_matrix();
extern int process_timer();
extern int process_timer2();

char *my_args[3][2] = {
		{"user_space","timer.bit"},
		{"user_space","timer2.bit"},
		{"user_space","matrix.bit"}
};


int run_child(int i);

int main(int argc, char **argv)
{
	int i;
	pid_t ch_pid;
	int CHILD_NUM;

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
	int stat;
	int cntr;
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

int run_child(int i)
{
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
	//execute program
	fprintf(stdout,"Starting child process with image: %s.\n",my_args[i%CONFIG_NUM][1]);

	switch(i%CONFIG_NUM)
	{
	case 0:process_timer(); break;
	case 1: process_timer2(); break;
	default: process_matrix(); break;
	}
	//exit
	sem_close(sem);
	return 0;
}


