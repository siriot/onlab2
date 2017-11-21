#!/bin/bash

cp -f ../user_space/mmap_test.c $PROJ/components/apps/reader/mmap_test.c

if [ -z $PETALINUX ]
then 
	source settings.sh
fi

petalinux-build -p $PROJ -c rootfs/reader -v  | tee reader_build.log

cp -f $PROJ/build/linux/rootfs/apps/reader/reader ../build/reader

echo "**********************************"
echo "**********************************"
echo "Uploading to tftp server over sftp"
echo "**********************************"
echo "**********************************"
sshpass -p 'redheaven' sftp  -oBatchMode=no -b /home/tibi/onlab2/scripts/reader_upload_cmds pi@192.168.137.100