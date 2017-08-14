#!/bin/bash
WIDTH=324
HEIGHT=244
./camcam -p 0,0,$WIDTH,$HEIGHT -w $WIDTH -h $HEIGHT -fli 60hz -vs -ex antishake -awb off -mm spot -drc med -awbg 1.3,1.5 --verbose
