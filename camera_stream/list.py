import subprocess 
cmd = ["v4l2-ctl", "--list-devices"]
output = subprocess.check_output(cmd, text=True)
output = list(output.split("\n"))
output = list(line.strip("\t") for line in output)

available_cameras = []
for i, line in enumerate(output):
    if "usb" in line:
        cam_index = int(output[i+1].strip("/dev/video"))
        available_cameras.append(cam_index)
    
print(list(available_cameras))
