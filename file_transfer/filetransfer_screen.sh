#!/bin/bash

# This is a simple script to send files to CROS.
# It assumes that you have screen session to CROS terminal open,
# e.g. screen /dev/ttyUSB0 57600
# It also requires you've already installed uudecode in /bin.
#
# Then just run: ./filetransfer_screen.sh myfile.r3
#
# Files will be dumped in the current directory on CROS side.

CRS_CON=crs_con 
TMP=`tempfile`
for var in "$@"
do
    echo uudecode > $TMP
    uuencode $var $var >> $TMP 
    screen -r $CRS_CON -X readreg p $TMP
    sleep 1
    screen -r $CRS_CON -X stuff ^M
    sleep 1
    screen -r $CRS_CON -X paste p
    sleep 1
    screen -r $CRS_CON -X stuff ^M
    sleep 1
done


