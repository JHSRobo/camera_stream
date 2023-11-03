#!/bin/bash

# Script to be run at EVERY boot

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

grep -q "Setup" /etc/rc.local || ( echo "Please run setup!" && exit )
#raspi-config nonint do_legacy 0

# Start up camera streamer
bash /home/jhsrobo/camera_stream/ping.sh &

#export ROTATION=0
export WIDTH=1440
export HEIGHT=810
export FPS=50

libcamera-vid --framerate ${FPS} --width ${WIDTH} --height ${HEIGHT} -t 0 --codec mjpeg --inline --listen -o - | ncat -lkv4 5000
