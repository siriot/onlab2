#!/bin/bash

cp -f ../sched_2.0/main.c $PROJ/components/modules/sched/main.c

if [ -z $PETALINUX ]
then 
	source settings.sh
fi

petalinux-build -p $PROJ -c rootfs/sched -v  | tee sched_build.log

cp -f $PROJ/build/linux/rootfs/modules/sched/main.ko ../build/main.ko

echo "**********************************"
echo "**********************************"
echo "Uploading to tftp server over sftp"
echo "**********************************"
echo "**********************************"
sshpass -p 'redheaven' sftp  -oBatchMode=no -b /home/tibi/onlab2/scripts/sched_upload_cmds pi@192.168.137.100