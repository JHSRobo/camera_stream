from v4l2py.device import Device

path = "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0"

device = Device(path)
device.open()

while True:
    device.controls["brightness"].value = int(input())
