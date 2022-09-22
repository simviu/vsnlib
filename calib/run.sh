SRC=-v=$1
#SRC=--ci=0
./charuco_calib -a=1 --sc --pc --ml=0.0205 --sl=0.0345 -d=10 -w=5 -h=7 --zt $SRC cam.yml 
