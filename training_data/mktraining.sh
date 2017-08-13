#!/bin/bash

[ -f training.csv ] && mv training.csv training-old.csv
echo "file,throttle,steer" > training.csv
for i in *.yuv; do
    echo "$i"
    ../camcam/mkdetect $i 2>/tmp/train-out.txt
    steer=`grep steer= /tmp/train-out.txt | sed -e 's/.*=//'`
    drive=`grep drive= /tmp/train-out.txt | sed -e 's/.*=//'`
    if [ ! -z "$steer" ]; then
        echo "$i,$drive,$steer" >> training.csv
    else
        echo "$i,0,0" >> training.csv
    fi
done
