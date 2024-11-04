![Image](./img/logo.png)

10/24/24 Version 1.0:

**Contributors:** James Randall '24, Alex Bertran '24, Jack Frings '26

**Editors:** Jack Frings '26

**Approved by:** Pending

---
This repository contains the software used to stream camera data from a Raspberry Pi Zero 2W to any ethernet address. In our case, the repository is meant to stream about four cameras to our [Pilot GUI](https://github.com/jhsrobo/pilot_gui).

This documentation is intended to be read and understood by other highschoolers like us, but this module will be effective for any RANGER, PIONEER, or EXPLORER team, or just anyone trying to build a low-cost ROV. If you've got questions, reach out to fringsj26@student.jhs.net. Feedback is always welcome.

## Setting up the Pi
When setting up the camera, flash a SD card with Bookworm 32 Bit Lite. To flash the SD card, we reccomend downloading the [RPi Imager](https://www.raspberrypi.com/software/). After downloading the imager, plug the SD card into your computer and select the neccesary options. When you are given the option to edit settings, make sure to allow for a ssh connection.

After flashing the SD, you can connect to the Pi over WiFi. However, this guide will access the Pi over our [ethernet hat](https://www.waveshare.com/eth-usb-hub-hat-b.htm). Identify the Pi on your router page and then ssh into the Pi to begin camera setup.

## Installing Streaming Software
After succesfully connecting to the Pi, you should begin installing the camera code. First, make sure your system is up to date. 
~~~bash
sudo apt update
sudo apt upgrade
~~~
Then, clone the ROVOTICS git repository.
~~~bash
git clone https://github.com/jhsrobo/camera_stream
~~~
Enter the new directory.
~~~bash
cd camera_stream
~~~
In this directory, ROVOTICS keeps three files.

The file ping.sh contains IP addresses that the Pi will stream to. These addresses can be configured to whatever IP you want to use for your own implementation. 

The stream.sh file is used to stream the camera footage to the IPs specified in the ping.sh file. 

Finally, the setup.sh installs some neccesary dependencies and uses crontab to automatically run stream.sh on reboot so that there is no need to manually run any file on the camera after the intiial setup. 

Before running any files, you need to specify what camera module you're planning to run. If you are running a default RPi Camera Module, there is no need to configure anything. 
Otherwise, if you're running an Arducam, you should alter the /boot/config.txt file to suit your camera according to the [official documentation](https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/5MP-OV5647/#selection-guide).

Finally, run the setup script by running the following bash command.
~~~bash
sudo bash setup.sh
~~~
