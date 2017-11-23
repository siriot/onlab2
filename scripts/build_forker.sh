#!/bin/bash

PROJECT_NAME=forker
APP_NAME=sched_user

echo -n "Current project:"
echo $PROJ
echo "Copy files to petalinux project folder" 
cp ../user_space/forker/accel_lib.c   $PROJ/components/apps/$APP_NAME/accel_lib.c
cp ../user_space/forker/accel_lib.h   $PROJ/components/apps/$APP_NAME/accel_lib.h
cp ../user_space/forker/forker.c      $PROJ/components/apps/$APP_NAME/forker.c


echo "Running settings.sh" 
source /home/tibi/petalinux/petalinux-v2016.2-final/settings.sh 

echo "Rebuild rootfs" 
petalinux-build -p $PROJ -c rootfs/$APP_NAME -v | tee forker_build.log

echo "Get compiled file back"  
cp  $PROJ/build/linux/rootfs/apps/$APP_NAME/$APP_NAME  ../build/$APP_NAME

echo "Uploading to tftp server over sftp"
sshpass -p 'redheaven' sftp  -oBatchMode=no -b ./forker_upload_commands pi@192.168.137.100
