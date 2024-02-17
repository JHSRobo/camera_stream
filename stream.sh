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

#export ROTATION=0
export WIDTH=1440
export HEIGHT=810
export FPS=30

# Get the last 3 digits of the IP
export PORT=5$(hostname -I | cut -f1 -d' ' | cut -c 11-13)

libcamera-vid --framerate ${FPS} --width ${WIDTH} --height ${HEIGHT} --inline 1 -g 1 -t 0 --listen -o udp://192.168.1.100:${PORT}

