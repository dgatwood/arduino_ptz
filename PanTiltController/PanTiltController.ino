#include "DualMC33926MotorShield.h"

/*
 * This Arduino sketch is the software for a pan/tilt/zoom controller.  It uses wired motor control
 * for panning and tilting, and uses LANC for sending zoom commands to compatible cameras.  It
 * supports both the Bescor MP-101 (with direct wiring) and the Vidpro MH-430 (with a Pololu
 * Dual MC33926 Motor Driver Shield).  The LANC circuit can be found on the web at:
 *
 * http://controlyourcamera.blogspot.com/2011/02/arduino-controlled-video-recording-over.html
 *
 * Note that the record button in that circuit is unused, and does not need to be included.
 * The LANC portions of this code are verified to work with the Canon XH-A1 and the Panasonic
 * AG-CX350.  It is likely that it will work with any LANC-compatible (a.k.a. Control-L) camera.
 */
/*
 * PTZ Controller
 * © 2020 David A. Gatwood.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// For MP-101 hardware configuration, set to true (different outputs).
// For Vidpro MH-430 hardware configuration with a motor controller, set to false.
#define useMP101Mode 1

const int kMotorMax = 400;
const double kMotorSpeedPower = 2.0;
const int kMinimumMotorThreshold = 50;  // Ensure that values close to the center are treated as stopped.

const int kZoomMax = 9;
const double kZoomSpeedPower = 2.0;
const int kMinimumZoomThreshold = 50;  // Ensure that values close to the center are treated as stopped.

DualMC33926MotorShield md;

#define MP101AnalogPowerPin A0 

#define MP101AnalogGroundPin A1
#define horizontalSensePin A3
#define verticalSensePin A2
#define zoomSensePin A4

#if useMP101Mode
// The MP101 setup doesn't have the motor control on pin 8, so keep LANC on a single connector.
#define lancOutputPin 8
#else
// Use the only free pin we have.
#define lancOutputPin 6
#endif
#define lancInputPin 11

#define MP101UpPin 2
#define MP101DownPin 3
#define MP101LeftPin 4
#define MP101RightPin 5

typedef enum {
  kDebugModePan = 0x1,
  kDebugModeTilt = 0x2,
  kDebugModeZoom = 0x4,
  kDebugScaling = 0x8,
  kDebugLANC = 0x16,
} debugMode;

int debugPanAndTilt = 0; // kDebugModePan;// kDebugModeZoom;  // Bitmap from debugMode.

void setup() {
  // Initialize the motor controller
  Serial.begin(9600);
  if (!useMP101Mode) {
    Serial.write("Initializing motor controller.");
    md.init();
    Serial.write("Done.");
  }
  if (useMP101Mode) {
    pinMode(MP101AnalogPowerPin, OUTPUT);
    digitalWrite(MP101AnalogPowerPin, HIGH);
    pinMode(MP101AnalogGroundPin, OUTPUT);
    digitalWrite(MP101AnalogGroundPin, LOW);
    pinMode(MP101UpPin, OUTPUT);
    pinMode(MP101DownPin, OUTPUT);
    pinMode(MP101LeftPin, OUTPUT);
    pinMode(MP101RightPin, OUTPUT);
  }

  Serial.write("Initializing LANC.");
  lancSetup();
  Serial.write("Done.");
}

void stopIfFault()
{
  if (!useMP101Mode) {
    if (md.getFault()) {
      md.setM1Speed(0);
      md.setM2Speed(0);
      Serial.println("fault");
      while(1);
    }
  }
}

void loop() {
  handleMotorControl();
  handleLANC();
}

#pragma mark - Motor control

void handleMotorControl() {
  int rawHorizontalValue = analogRead(horizontalSensePin);
  int maxValue = useMP101Mode ? 255 : kMotorMax;
  int horizontalValue = computeScaledSpeedValue(1023 - rawHorizontalValue, kMotorSpeedPower, kMinimumMotorThreshold, maxValue);  // Reversed.
  logValues("horizontal: ", rawHorizontalValue, horizontalValue, kDebugModePan);
  if (useMP101Mode) {
    setMP101Horizontal(horizontalValue);
  } else {
    md.setM2Speed(horizontalValue);
  }

  int rawVerticalValue = analogRead(verticalSensePin);
  int verticalValue = computeScaledSpeedValue(rawVerticalValue, kMotorSpeedPower, kMinimumMotorThreshold, maxValue);
  logValues("vertical: ", rawVerticalValue, verticalValue, kDebugModeTilt);
  if (useMP101Mode) {
    setMP101Vertical(verticalValue);
  } else {
    md.setM1Speed(verticalValue);
  }
}

// By stripping the sign and scaling the value to the range 0..1, we can raise the input value to
// an arbitrary power to change the curve. We then multiply that by maxScale and provide the
// sign to get a value in the range [-maxScale .. maxScale].  (Positive and negative input values
// are not exactly balanced numerically, so we throw away the largest possible value on the positive
// side.)
int computeScaledSpeedValue(int bareValue, int power, int threshold, int maxScale) {
  int rawSpeed = abs(bareValue - 512);  // Range -512 .. 511
  bool direction = (bareValue >= 512);  // Equal number of positive and negative values.

  if (debugPanAndTilt & kDebugScaling) {
    Serial.print("bareValue: ");
    Serial.print(rawSpeed);
    Serial.print(" rawSpeed: ");
    Serial.print(rawSpeed);
    Serial.print(" direction: ");
    Serial.println(direction ? "true" : "false");
  }

  if (rawSpeed <= threshold) {
    return 0;
  }
  int maxSpeed = 511 - threshold;
  double speedPercentage = ((double)rawSpeed / (double)maxSpeed);
  if (debugPanAndTilt & kDebugScaling) {
    Serial.print("Percent: ");
    Serial.println(speedPercentage);
  }

  speedPercentage = pow(speedPercentage, power);
  
  float scaledSpeed = (speedPercentage * maxScale);
  int scaledSpeedInt = (int)scaledSpeed;
  if (scaledSpeed > maxScale) {
    scaledSpeedInt = maxScale;
  }
  return direction ? scaledSpeedInt : (0 - scaledSpeedInt);
}

#pragma mark - MP101 motor control

void setMP101Horizontal(int horizontalValue) {
  int leftPinValue = (horizontalValue < 0) ? -horizontalValue : 0;
  int rightPinValue = (horizontalValue > 0) ? horizontalValue : 0;
  analogWrite(MP101LeftPin, leftPinValue);
  analogWrite(MP101RightPin, rightPinValue);
  if (0) {
    Serial.print("Left: ");
    Serial.print(leftPinValue);
    Serial.print(" Right: ");
    Serial.println(rightPinValue);
  }
}

void setMP101Vertical(int verticalValue) {
  int upPinValue = (verticalValue > 0) ? verticalValue : 0;
  int downPinValue = (verticalValue < 0) ? -verticalValue : 0;

  analogWrite(MP101UpPin, upPinValue);
  analogWrite(MP101DownPin, downPinValue);
  if (0) {
    Serial.print("Up: ");
    Serial.print(upPinValue);
    Serial.print(" Down: ");
    Serial.println(downPinValue);
  }
}

#pragma mark - Logging

void logValues(char *label, int rawValue, int scaledValue, debugMode mode) {
  if (debugPanAndTilt & mode) {
    Serial.write(label);
    Serial.print(rawValue);
    Serial.write(" -> ");
    Serial.print(scaledValue);
    Serial.println("");
  }
}

#pragma mark - LANC

extern boolean ZOOM_IN_0[];
extern boolean ZOOM_IN_1[];
extern boolean ZOOM_IN_2[];
extern boolean ZOOM_IN_3[];
extern boolean ZOOM_IN_4[];
extern boolean ZOOM_IN_5[];
extern boolean ZOOM_IN_6[];
extern boolean ZOOM_IN_7[];
extern boolean ZOOM_OUT_0[];
extern boolean ZOOM_OUT_1[];
extern boolean ZOOM_OUT_2[];
extern boolean ZOOM_OUT_3[];
extern boolean ZOOM_OUT_4[];
extern boolean ZOOM_OUT_5[];
extern boolean ZOOM_OUT_6[];
extern boolean ZOOM_OUT_7[];
extern boolean IDLE_COMMAND[];

void handleLANC() {
  int rawZoomValue = analogRead(zoomSensePin);
  if (!useMP101Mode) {
    rawZoomValue = 1023 - rawZoomValue;
  }
  int zoomSpeed = computeScaledSpeedValue(rawZoomValue, kZoomSpeedPower, kMinimumZoomThreshold, kZoomMax);  // Reversed.
  logValues("zoom: ", rawZoomValue, zoomSpeed, kDebugModeZoom);
  
  switch (zoomSpeed) {
    case -9:  // Extremely unlikely
    case -8:
      lancCommand(ZOOM_OUT_7);
      break;
    case -7:
      lancCommand(ZOOM_OUT_6);
      break;
    case -6:
      lancCommand(ZOOM_OUT_5);
      break;
    case -5:
      lancCommand(ZOOM_OUT_4);
      break;
    case -4:
      lancCommand(ZOOM_OUT_3);
      break;
    case -3:
      lancCommand(ZOOM_OUT_2);
      break;
    case -2:
      lancCommand(ZOOM_OUT_1);
      break;
    case -1:
      lancCommand(ZOOM_OUT_0);
      break;
    case 0:
      lancCommand(IDLE_COMMAND);
      break;
    case 1:
      lancCommand(ZOOM_IN_0);
      break;
    case 2:
      lancCommand(ZOOM_IN_1);
      break;
    case 3:
      lancCommand(ZOOM_IN_2);
      break;
    case 4:
      lancCommand(ZOOM_IN_3);
      break;
    case 5:
      lancCommand(ZOOM_IN_4);
      break;
    case 6:
      lancCommand(ZOOM_IN_5);
      break;
    case 7:
      lancCommand(ZOOM_IN_6);
      break;
    case 8:
    case 9:  // Extremely unlikely
      lancCommand(ZOOM_IN_7);
      break;
    default:
      break;
  }
}

#pragma mark - LANC Library

/*******************************************************************************************
 * All code below this point is derived from an existing LANC example, with the loop()     *
 * method stripped out, the setup() method renamed to lancSetup(), and shorter timeouts    *
 * added so that a failed LANC connection won't prevent pan and tilt control from working. *
 * The copyright notice and licensing terms are included below.                            *
 *******************************************************************************************/

/*
 SIMPLE LANC REMOTE
 Version 1.0
 Sends LANC commands to the LANC port of a video camera.
 Tested with a Canon XF300 camcorder
 For the interface circuit interface see 
 http://controlyourcamera.blogspot.com/2011/02/arduino-controlled-video-recording-over.html
 Feel free to use this code in any way you want.
 2011, Martin Koch

 "LANC" is a registered trademark of SONY.
 CANON calls their LANC compatible port "REMOTE".
*/
int cmdRepeatCount;
int bitDuration = 104; //Duration of one LANC bit in microseconds. 


//LANC commands byte 0 + byte 1
//Tested with Canon XF300

//Start-stop video recording
boolean REC[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,LOW,HIGH,HIGH,LOW,LOW,HIGH,HIGH};          // 18 33

boolean IDLE_COMMAND[] = {LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW};               // 00 00

//Zoom in from slowest to fastest speed
boolean ZOOM_IN_0[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW};        // 28 00
boolean ZOOM_IN_1[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,LOW,LOW,HIGH,LOW};       // 28 02
boolean ZOOM_IN_2[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,LOW,HIGH,LOW,LOW};       // 28 04
boolean ZOOM_IN_3[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,LOW,HIGH,HIGH,LOW};      // 28 06
boolean ZOOM_IN_4[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,HIGH,LOW,LOW,LOW};       // 28 08
boolean ZOOM_IN_5[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,HIGH,LOW,HIGH,LOW};      // 28 0A
boolean ZOOM_IN_6[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,HIGH,HIGH,LOW,LOW};      // 28 0C
boolean ZOOM_IN_7[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,HIGH,HIGH,HIGH,LOW};     // 28 0E

//Zoom out from slowest to fastest speed
boolean ZOOM_OUT_0[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,LOW,LOW,LOW,LOW};      // 28 10
boolean ZOOM_OUT_1[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,LOW,LOW,HIGH,LOW};     // 28 12
boolean ZOOM_OUT_2[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,LOW,HIGH,LOW,LOW};     // 28 14
boolean ZOOM_OUT_3[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,LOW,HIGH,HIGH,LOW};    // 28 16
boolean ZOOM_OUT_4[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW};     // 28 18
boolean ZOOM_OUT_5[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,HIGH,LOW,HIGH,LOW};    // 28 1A
boolean ZOOM_OUT_6[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,HIGH,HIGH,LOW,LOW};    // 28 1C
boolean ZOOM_OUT_7[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,HIGH,HIGH,HIGH,LOW};   // 28 1E

//Focus control. Camera must be switched to manual focus
boolean FOCUS_NEAR[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,HIGH,HIGH,HIGH};   // 28 47
boolean FOCUS_FAR[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,HIGH,LOW,HIGH};     // 28 45

boolean FOCUS_AUTO[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,LOW,LOW,HIGH};     // 28 41

//boolean POWER_OFF[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,HIGH,HIGH,HIGH,HIGH,LOW}; // 18 5E
//boolean POWER_ON[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,HIGH,HIGH,HIGH,LOW,LOW};   // 18 5C  Doesn't work because there's no power supply from the LANC port when the camera is off
//boolean POWER_OFF2[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW};  // 18 2A Turns the XF300 off and then on again
//boolean POWER_SAVE[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,HIGH,HIGH,LOW,HIGH,HIGH,LOW,LOW}; // 18 6C Didn't work


void lancSetup() {
  pinMode(lancInputPin, INPUT); // listens to the LANC line
  pinMode(lancOutputPin, OUTPUT); // writes to the LANC line
  digitalWrite(lancOutputPin, LOW); // set LANC line to +5V
  delay(5000); // Wait for camera to power up completly
  bitDuration = bitDuration - 8; // Writing to the digital port takes about 8 microseconds so only 96 microseconds are left for each bit

  pinMode(zoomSensePin, INPUT); // listens to the LANC line
}

void lancCommand(boolean lancBit[]) {
  cmdRepeatCount = 0;

while (cmdRepeatCount < 5) {  // repeat 5 times to make sure the camera accepts the command
  // If this breaks, change the timeout (which didn't exist before, and thus defaulted to one second).
  // This gives us 40 adjustments per second, and is 5000 msec longer than a full LANC cycle, which
  // means this should kick in only if the LANC hardware is not attached.
  int missed = 0;
  unsigned long duration = 0;
  while (((duration = pulseIn(lancInputPin, HIGH, 25000))) < 5000) {   
    // "pulseIn, HIGH" catches any 0V TO +5V TRANSITION and waits until the LANC line goes back to 0V 
    // "pulseIn" also returns the pulse duration so we can check if the previous +5V duration was long enough (>5ms) to be the pause before a new 8 byte data packet
    // Loop till pulse duration is >5ms

    // Run the motor at least 100 times per second anyway while we're waiting.
    if (duration == 0) {
      if (debugPanAndTilt & kDebugLANC) {
        Serial.println("No pulse");
      }
      handleMotorControl();
    }
  }

  // LOW after long pause means the START bit of Byte 0 is here
  delayMicroseconds(bitDuration);  // wait START bit duration

  // Write the 8 bits of byte 0 
  // Note that the command bits have to be put out in reverse order with the least significant, right-most bit (bit 0) first
  for (int i=7; i>-1; i--) {
    digitalWrite(lancOutputPin, lancBit[i]);  //Write bits. 
    delayMicroseconds(bitDuration); 
  }

  // Byte 0 is written now put LANC line back to +5V
  digitalWrite(lancOutputPin, LOW);
  delayMicroseconds(10); //make sure to be in the stop bit before byte 1

  while (digitalRead(lancInputPin)) { 
    // Loop as long as the LANC line is +5V during the stop bit
  }

  // 0V after the previous stop bit means the START bit of Byte 1 is here
  delayMicroseconds(bitDuration);  // wait START bit duration

  // Write the 8 bits of Byte 1
  // Note that the command bits have to be put out in reverse order with the least significant, right-most bit (bit 0) first
  for (int i=15; i>7; i--) {
    digitalWrite(lancOutputPin,lancBit[i]);  // Write bits 
    delayMicroseconds(bitDuration);
  }

  // Byte 1 is written now put LANC line back to +5V
  digitalWrite(lancOutputPin, LOW); 

  cmdRepeatCount++;  // increase repeat count by 1

  /* Control bytes 0 and 1 are written, now don’t care what happens in Bytes 2 to 7
     and just wait for the next start bit after a long pause to send the first two command bytes again. */
  } // While cmdRepeatCount < 5
}
