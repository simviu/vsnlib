#!/bin/bash


FRMS=$1
OUTF=$2

if [ -z "$FRMS" ] || [ -z "$OUTF" ]; then
    echo "Usage: ./run.sh <IMG_DIR> <OUT_YAML>"
    exit 1
else
    echo "Input frms: $FRMS" 
    echo "Out YAML: $OUTF" 
fi


SRC=-imgs=$FRMS
#SRC=-v=$1
#SRC=--ci=0    

#./charuco_calib -a=1 --sc --pc --ml=0.0205 --sl=0.0345 -d=10 -w=5 -h=7 --zt $SRC cam.yml 
./charuco_calib -a=1 --sc --pc --ml=0.021 --sl=0.035 -d=10 -w=5 -h=7 --zt $SRC OUTF 

