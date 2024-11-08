!/bin/bash

# Script to be run at EVERY boot

# Give camera time to initialize
sleep 10

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Start up camera streamer
bash /home/jhsrobo/camera_stream/ping.sh &

export WIDTH=1440
export HEIGHT=810
export FPS=45

libcamera-vid --framerate ${FPS} --width ${WIDTH} --height ${HEIGHT} --codec mjpeg --inline 1 -g 1 -t 0 --listen -o - | ncat -lkv4 5000
