# Cameras
These are the steps for preparing our Pi Camera SD Cards. It's a part of our larger Software Documentation repository, and it was relevant for the creation of our Pi Cameras, so I excerpted it here.

Written by James Randall.

The obvious question here is: "Why not SD Card cloning?"

The answer is a sort of pseudo-superstition. We've corrupted many SD cards trying to perfect this process, so at this point it is faster to just automate the setup with a script or two.



## Setup
1. Flash a micro-SD card using the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) with **Raspbian Bullseye Lite** (32-bit).
	* Click the settings icon and adjust the settings
	* Create a user with username jhsrobo and password JHSRobo
	* Enable SSH with Password Authentication
	* Enable WiFi and configure with appropriate settings
	* Enable the locale settings
2. Install the SD card into an RPi 3B+ or 4. Finish installation if prompted.
	* We use 3B+ and 4 for setup because our cameras do not have WiFi access. If you are using raspberry Pi 2Ws for your cameras, you can use those for this step instead of the RPi 3B+ or 4.
3. Enable WiFi on the Raspberry Pi
	* You will need to enable it using the menu by running `sudo raspi-config`.
		* If you still can't connect, run`sudo nmtui` and connect to wifi.
			* If this gives you an error related to network manager, run `sudo systemctl start NetworkManager`
    * You may need to change your default gateway. Do this with `ip route add default via gateway dev wlan0 onlink`, where `gateway` is replaced with your network's default gateway.
	    * Please note that this command is temporary and only changes your IP for around 10 minutes, but this is plenty of time for the steps that come.
4. Run the following commands:
	* `sudo apt update`
	* `sudo apt install git`
	* `git clone https://github.com/JHSrobo/camera_stream`
	* `sudo bash ~/camera_stream/setup.sh`
You will be disconnected from the RPi, and you will get a video feed when you run topside.