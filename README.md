This Arduino sketch is the software for a pan/tilt/zoom controller.  It uses wired motor control
for panning and tilting, and uses LANC for sending zoom commands to compatible cameras.  It
supports both the Bescor MP-101 (with direct wiring) and the Vidpro MH-430 (with a Pololu
Dual MC33926 Motor Driver Shield).

The LANC circuit and the original version of the LANC driver code can be found on the web at:

    http://controlyourcamera.blogspot.com/2011/02/arduino-controlled-video-recording-over.html

Note that the record button in that circuit is unused, and does not need to be included.
The LANC portions of this code are verified to work with the Canon XH-A1 and the Panasonic
AG-CX350.  It is likely that it will work with any LANC-compatible (a.k.a. Control-L) camera.
