#!/bin/bash

# Script to be run ONCE at the first boot

# Checks sudo perms needed for installing libraries, editing files and changing passwords
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Pings google. if google breaks, we don't have access to start a new camera, sorry. Needs the internet and
# this was the best way I could think of. If you have a better way go for it
# TODO
if [[ "$(echo -e "GET http://google.com HTTP/1.0\n\n" | nc google.com 80 > /dev/null 2>&1)" == 0 ]]; then
    echo "Please connect to the internet to install libraries"
    exit
fi

# Makes sure the PI is up to date. This could cause things to break in the future
apt update -y
rpi-update -y

# Checks if ncat and nmap are installed and then installs them if they are not
# Needed for the raspivid function to stream the camera
dpkg -l | grep ncat || apt install ncat -y
dpkg -l | grep nmap || apt install nmap -y

# Make stream.sh launch on startup
crontab -l > crontab_new
echo "@reboot bash /home/jhsrobo/camera_stream/stream.sh" >> crontab_new
crontab crontab_new
rm crontab_new

# enable cameras
#raspi-config nonint do_legacy 0

# give more memory
echo -e "$(sed '/gpu_mem/d' /boot/config.txt)" > /boot/config.txt
echo "gpu_mem=256" >> /boot/config.txt

# increase max framebuffers
echo -e "$(sed '/max_framebuffers/d' /boot/config.txt)" > /boot.config.txt
echo "max_framebuffers=4" >> /boot/config.txt

# set dtoverlay
echo -e "$(sed '/dtoverlay/d' /boot/config.txt)" > /boot.config.txt
echo "dtoverlay=vc4-fkms-v3d" >> /boot/config.txt

# turn off the red light. if you leave it on, it reflects off the glass
echo "disable_camera_led=1" >> /boot/config.txt

reboot now
