SRC=-v=$1
#SRC=--ci=0
./calib -a=1 --sc --pc --ml=20.5 --sl=34.5 -d=0 -w=5 -h=7 --zt $SRC cam.yml 
