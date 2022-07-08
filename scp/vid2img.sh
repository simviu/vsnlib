
TSTART=04:00
TDUR=03:00
INFILE=yourvideo.mov
WDIR=imgs
FPA=1

ffmpeg -ss $TSTART  -t $TDUR -i $INFILE -r $FPS $WDIR/img%03d.png
