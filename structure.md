# Camera Viewer Software

Written by James Randall '25

This document outlines the way that Jesuit High School's Robotics team handles camera feed.

TL;DR: On our raspberry pi cameras, we pipe video to port 5000. Our camera viewer software listens for these cameras, and captures the video from the camera's port 5000. See the diagram below. 
![Image](./img/camera_diagram.png)

NOTE: Topside refers to anything above the water's surface. There are 4 cameras in the above diagram, but that number can vary depending on implementation.

## Software on the Cameras
The software that runs on our cameras is microscopic and spartan. The big reason for this is that updating code on these cameras is a hassle. Our cameras do not have internet access, so once the code is on there, it is tough (not impossible, but not fun) to pull new code from GitHub. The cameras constantly broadcast video feed, even when nothing is listening. If the cameras are on, we're streaming. That's the point of this code, and any additional complexity is handled in the camera viewer.

There are 3 scripts:
* `stream.sh` is a program that runs whenever the pi turns on. It activates `ping.sh`, and streams the output of the RPi Camera to port 5000 on the camera.
  * For example, if the Camera's IP was 192.168.1.100, `stream.sh` would send the video feet to 192.168.1.100:5000
* `ping.sh` is a program that is run by `stream.sh` , and it's tiny. (The reason that it's a separate program is that we can treat it like a crude form of multithreading). It pings the "Topside IP" indefinitely on port 12345.
* `setup.sh` is a program that is run manually, once. It edits the crontab to launch `stream.sh` on startup, and configures the Pi for streaming.

Almost all of the heavy lifting is done by this single line of code in `stream.sh`.

`raspivid -n -cd MJPEG -awb auto -ifx none -b 25000000 -br 60 -t 0 -rot ${ROTATION} -w ${WIDTH} -h ${HEIGHT} -fps ${FPS} -o - | ncat -lkv4 5000`

This line grabs video from the camera with the `raspivid` command, and sends it to STDOUT, which we pipe to port 5000 using `ncat`.

#### `raspivid` Parameters Breakdown
* `-n, --nopreview` : Do not display a preview window
* `-cd CODEC` (we use MJPEG , motion jpeg)
* `-awb, --awb` : Set auto white balance mode
* `-ifx, --imxfx` : Set image effect
* `-b, --bitrate` : Set bitrate. Use bits per second
* `-br, --brightness` : Set image brightness (0 to 100)
* `-t, --timeout` : Time (in ms) before takes picture and shuts down. If not specified, set to 5s
* `-rot, --rotation` : Set image rotation (0-359)
* `-w, --width` : Set image width . Default 1920 (set in variable)
* `-h, --height` : Set image height . Default 1080
* `-fps, --framerate` : Specify the frames per second to record
* `-o, --output` : Output filename (to write to stdout, use '-o -')


#### `ncat` Parameters Breakdown
* `-l, --listen` : Bind and listen for incoming connections
* `-k, --keep-open` : Accept multiple connections in listen mode
* `-v, --verbose` : Set verbosity level (can be used several times)
* `-4` : Force the use of IPv4 only.
* `5000` : Push out to port 5000

## Software on Topside
There is only one relevant piece of software on Topside, and it is our [camera viewer](https://github.com/JHSRobo/camera_view/tree/main). Unfortunately, it is highly specific to our implementation. Feel free to take a look, but it likely will not be much good for you. Jesuit Robotics is looking forward to releasing a more open-source friendly version in the future.