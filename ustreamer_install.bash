apt install build-essential libevent-dev libjpeg-dev libbsd-dev
cd /home/jhsrobo
git clone --depth=1 https://github.com/pikvm/ustreamer
cd ustreamer
make
cp ustreamer /usr/local/bin
