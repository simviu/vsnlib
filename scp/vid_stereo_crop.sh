
Y=$3
bin/vsntool video crop file=$1 filew=$2/L.mkv start=0,$Y sz=960,540 
bin/vsntool video crop file=$1 filew=$2/R.mkv start=960,$Y sz=960,540 


