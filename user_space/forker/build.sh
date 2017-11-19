#!/bin/bash

PROJECT_NAME=onlab
APP_NAME=sched_user
CURRENT_DIR=~/onlab/sched_user/sched_user
PROJ=/home/tibi/petalinux/onlab

echo -n "Current project:"
echo $PROJ
echo "Copy files to petalinux project folder" 
cp user_space.c  $PROJ/components/apps/$APP_NAME/user_space.c
cp accel_lib.c   $PROJ/components/apps/$APP_NAME/accel_lib.c
cp accel_lib.h   $PROJ/components/apps/$APP_NAME/accel_lib.h
cp forker.c      $PROJ/components/apps/$APP_NAME/forker.c


echo "Running settings.sh" 
cd  $PROJ
source /home/tibi/petalinux/petalinux-v2016.2-final/settings.sh 

echo "Rebuild rootfs" 
petalinux-build -c rootfs/$APP_NAME -v

echo "Get compiled file back"  
cp  $PROJ/build/linux/rootfs/apps/$APP_NAME/$APP_NAME  $CURRENT_DIR/user_main

echo "Uploading to tftp server over sftp"
sshpass -p 'redheaven' sftp  -oBatchMode=no -b $CURRENT_DIR/sftp_commands pi@192.168.137.100
