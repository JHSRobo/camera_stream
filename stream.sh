#!/bin/bash

# Script to be run at EVERY boot

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

grep -q "Setup" /etc/rc.local || ( echo "Please run setup!" && exit )
raspi-config nonint do_legacy 0

# Start up camera streamer
bash /home/jhsrobo/camera_stream/ping.sh &

# export ROTATION=0
# export WIDTH=1280
# export HEIGHT=720
# export FPS=45

# raspivid -n -cd MJPEG -awb auto -ifx none -b 25000000 -br 60 -t 0 -rot ${ROTATION} -w ${WIDTH} -h ${HEIGHT} -fps ${FPS} -o - | ncat -lkv4 5000
libcamera-vid -t 10000 --codec mjpeg -o | ncat -lkv4 5000
