if [ $# -ne 1 ]; then
    echo 'Specify the log-file to write to!'
    exit -1
fi
serial=/dev/ttyS2 
date="`date +%Y%m%d-%H%M`"
logfile="$1-$date.txt"
echo "Logging '$serial' --> '$logfile'"
date > "$logfile" 
stty -F /dev/ttyS2 19200 && tee -a "$logfile" < $serial
