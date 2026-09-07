from camera import Camera

camera = Camera("front", "/home/jhsrobo/camera_stream/config")
camera.stream.start()
print("yes\n")


while True:
    input = input()
    if input == "5":
      camera.close()
      break
