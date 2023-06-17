# Raspberry Pi Zero - Digital Camera Overview and Assembly Instructions 2023
This is the build guide for Jesuit High School's Raspberry Pi Cameras. It includes the required parts, and directions for assembly. This documentation was written entirely by new team members (Koen M and Charlie K), to ensure that no knowledge is taken for granted. Edited by James R and Charlie K.

These cameras do not run much in the way of software. They ping an IP address (The IP of your camera viewer, by default 192.168.1.100), and they stream camera feed to their own IPs on port 5000. A good camera viewer program listens for the pings and tunes in to those IPs. Jesuit will be open sourcing its camera viewer software in the near future, as an example.

For more software information, check out the included flowchart.

# BOM/Components:

## For 1 Camera:
## Electrical Components
* Raspberry PI-Zero ($5 )
* Pi Pogo board ($12) 
* Raspberry Pi Camera V1 ($12)
* Raspberry Pi Camera Ribbon Cable (csi) UC- 665 ($2)
* RJ45 ethernet adapter ($13)
* Ethernet cables
* 4 pin jst wires. 
* Right angle 2 pin jst adapter
* 2 pin jst 22 gauge
* Heatshrink 

## Mechanical Components
* Camera Housing
* 3-D Printed Casing
* 3-D Printed Housing
* 3-D Printed Lid
* Denatured Alcohol
* Clear Silicone - Waterproof Sealant
* Testing Components
* TCU - (Topside Control Unit)
* USB to JST 5v power plug 
* Ethernet Cable

## Software Components

* Micro-SD Card (8gb+)
* Device with micro-SD card interface

## Tool Prep:

* Ethernet Crimper
* Snips
* Solder and soldering iron


# Build Guide:

1. Modify ethernet cables
2. Make the inner MEH ethernet cable.
3. Make Pi setup
4. Test the Pi setup
5. Pot Pi setup
6. Vaccum Test Units
7. Make the other end of the Camera ethernet cable
8. Put the cameras in the MEH.
9.   Connect ethernet cables to MEH

  ##  1. Modify Ethernet Cables
***

1.  Cut the ethernet cables in half

2.  On the terminated side (one with the plug) cut off the head so it is just cable. Strip about 2 inches off one end. 

3. Crimp stubby rj45 connector to all wires except for blue and brown pairs. **(Follow Guide)** The reason why we use the orange & green wires is because these wires are being used for Ethernet signals, and blue & brown are used for power and ground. We will use the blue & brown cables for  power later.

4.  Separate  out the blue and brown pairs.

5. Solder Blue pair to red and Brown pair to black of a 2 pin JST cable.*Make sure the JST wires are in the correct orientation because they sometimes are messed up.  *Try to use a 22 gauge wire JST but 26 gauge works too. *Also don’t forget to put on the heat shrink on wires before s soldering them together.  

6. Use the multimeter with a pointed object  and some alligator clips to test connectivity with the ethernet head. *The ethernet rj45 sometimes does not stick into the wires. Stick the other ends of the ethernet wires into a disassembled wall plug. 
   
## 2. Make the inner MEH ethernet cable.
***

1. Take an ethernet cable and cut off about 6 in. 

2. Strip of about 1.5 in and separate off the blue and brown pairs. Crimp the other wires like the other end to a rj45 stubby or non-stubby. 

3.  Take off the housing on the ethernet cable and take out the blue/brown wire.*The housing just kept the wires together during crimping. 

4. Then solder the rest of the wires to a 4 pin JST male cable in order using the following guide. Make sure to use a heat shrink for strain relief on the rj45 and also put a heatshrink on each of the wires before you solder. (I always messed up on it. - Charlie K)

## 3. Make Pi setup 
***
1.  Solder right angle 2 pin JST connector to 5v and ground on the Raspberry Pi, OR, use a pigtail cable and solder it correctly to the Pi using the diagram below. Then use sticky tack to stick it to the pi. 

2. Connect the pogo board to the Pi zero with the standoffs. 

3.  Plug the camera into the zero with the ribbon cable. 

## 4. Test the Pi setup
***

The following steps will differ greatly depending on your setup. The directions we have included below are specific to Jesuit's use cases. The gist is that we are obtaining the IP address of the camera, and testing the software through an ssh instance.

1.  Setup the TCU by plugging it into a wall outlet. Turn it on, and plug in a USB keyboard and mouse.
 
2. When the TCU turns on, navigate to the upper right panel. Click on the Power symbol, and click on Wired Connected. Turn it off, and then turn it back on.
 
3. Once you’ve turned the wired connection off and on again, click on the FireFox Web Browser in the upper left hand corner of the screen. Type 192.168.1.1 into the search bar, and if a website comes up then enter.

4. Log into the RouterOS by clicking Login. The password and username should already be there.

5. Click on IP, and then click on DHCP Server.

6. Click on Leases.

7. Plug the Ethernet to Ethernet cable into your Pi through the Pogo Board, and then plug in power through the 90 degree 2-pin JST connector, using the JST Power Connector. (The JST Power Connector has two sides - one side connects to the TCU’s USB ports, the other side connects to the Pi itself.) You should see a green light activate on the Pi.

8. Wait for a Camera to appear on the hosts screen. The Active Host Name should be JHSCamera.
  
9. Once a Camera host appears, click on the Terminal app. A terminal window should open.

10. Type “cd .ssh” into the terminal window. Then, type “ls” into the window. You should see the text “known_hosts” pop up below the command you just ran.

11. Type “rm *” into the terminal window. Then, type “ls” again. There should no longer be text saying “known_hosts”.

12. Type “cd ..” into the terminal window. Then, type “topside”. A bunch of commands should run, and you should get two windows to pop up: One, which is a window with a load of bars, and another window with your camera feed. If there’s a camera feed, then your camera and Pi work.

13. Navigate back to the terminal window, and hold down CTRL while pressing the C key. This will terminate the topside code lines.

14. Type “ssh” and then put a space. Next, navigate back to the FireFox Browser and find the “Active Address” of the JHSCamera. Type the number you see after “ssh”. P.S. An example of the correct number to type is “192.168.1.99”. *Note that the number can never be 100 or higher- the example used here is NOT A CAMERA. Funny story, once I turned off the entire TCU by accident by calling a .100 IP. - Koen M

15. It should ask whether or not you want to continue. Type “yes” into the terminal window. Then, it should ask for a password. The password you will have to type is “JHSRobo”, and the text will not show even if you type it properly. If you think you have the right password, simply enter and you should be ssh’d into the camera’s IP.

16.  Make sure the camera is focused for about 2 meters. 

17. Type “sudo shutdown now” and then watch your Pi and camera’s lights turn off. You can now safely unplug the Pi’s ethernet cable and power cable.

18. Shut off the TCU.
## 5. Pot Pi setup

***
1.  Screw the camera in first with m2*0.4mm by 8mm long or up to 12mm long nylon screws. *Make sure to find the right sized screw driver as the screws are easy to strip. **if they do not screw in well you may have to tap it. Don't tap completely through the standoffs as they can break. 

2. Tap the 3d print to screw in the pi. 

3. Screw in the Pi next and fit the 3d printed housing. 

4. Drill a hole in the pvc with the drill mill

5. Then clean it with alcohol. 

6. Thread the modified cable to the pvc and plug it into the Pi setup

7. Fit the 3d printed case inside the pvc

8. Use glue to close the gap in the hole. Use the pvc cement to attach the turret to the pvc. 

9. Clean a lens with a exacto knife.

10. Sandpaper the dege lightly

11. Use goop to seal the lens to the housing. Before adding goop use tape to cover the case. As you are adding goop use the q-tip to make sure the goop gets in the crack. It helps so there are no airbubbles. Smooth with a popsicle stick and make sure there are no holes. Work quickly as it dries fast. 

12. Next you can put the penetrator on the wire about 8 inches up and glue it with . 

## Vacuum Test the Pi setups

***
  7.1 Put penetrator in the test meh.  

  7.2 Pump pressure to 10 and come back and check in 20 min. If the pressure has not dropped then it has passed the vacuum test.

  7.3 Then put the camera into a bucket of water and weight it down. Wait about 20 minutes and if there is no leak then it is good. 
  
  7.4 Unscrew the penetrator from the test meh. 

## Make the other end of the Camera ethernet cable


***
  8.1 Solder a two pin JST male adapter the same way that it was soldered on the other side. 
  8.2 With the other 4 wires solder a 4 pin jst pin male in the same order. 

## Put the cameras in the MEH. 

***
  9.1 Unscrew the penetrator on the MEH that is being used. 
  9.2 Put an O-ring on the new penetrator with some pool lube. 
  9.3 Put in the camera penetrator into the MEH. Put on a washer and slip on a split nut. 
  9.4 Screw it on and tighten it with a wrench. 

## Connect ethernet cables to MEH

10.1 Connect a rj45 adapter to 4pin male jst connector. 
10.2 Plug it to the ethernet switch
10.3 Take the 2 pin jst and plug it into the power cable. 
10.4  Power the rov and test it. 
10.5 Assemble the MEH back together. 
DONE!

DOCUMENT TRACKING	DOCUMENT TRACKING	DOCUMENT TRACKING	DOCUMENT TRACKING
| VERSION |	EDITS COMPLETED BY	| DATE	| DESCRIPTION OF EDIT
| --- | --- | --- | ---|
|1.0	|"initial draft -Charlie Kim and Koen | 3/11/23	|initial draft
|1.1	|" Second draft - Charlie Kim and Koen "|	3/18/23 |	More steps and pics. 
 |1.2	|" tools needed, estimated duration to complete, tags to flag key things to watch for, and ways to check if did the correct."	| 4/3/23