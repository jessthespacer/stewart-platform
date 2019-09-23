/* =================================================================================================== 
 *  This code has been provided as an example to help you get started on your project. The objective 
 *  is to provide user input to the Arduino board and have the servo motors actuate. Several lines of 
 *  code are accredited to the Adafruit PWM Servo Driver Library example code. To use the Adafruit 
 *  PWM Servo Shield, be sure to add the Adafruit library to your Arduino IDE. 
 *  (Adafruit example: File menu > Examples > Adafruit PWM Servo Drivers Library > servo)
 *  
 *  Add Adafruit Library: In the Arduino IDE application select: Sketch menu > Include Libraries > 
 *  Manage Libraries. In the Library Manager window, search and install the "Adafruit PWM Servo 
 *  Driver Library".
 *  
 *  NOTE: Depending on your servo motor, the pulse width min/max may be different. Adjust to match 
 *  your servo motor.
 =================================================================================================== */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Geometry.h>
#include "revKin.h"

const float deg2rad = M_PI / 180;
const float rad2deg = 180 / M_PI;

const int SERVOMIN = 175; // 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 475; // 'maximum' pulse length count (out of 4096)
const int SERVOMID = floor((SERVOMAX+SERVOMIN)/2); // 'mid' pulse length count (out of 4096)
const int SERVOCHG = 50; // 'change' pulse length count

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

String valInput; // Serial input var.
int i = 0; // loop index var.
int val[6] = {SERVOMID, SERVOMID, SERVOMID, SERVOMID, SERVOMID, SERVOMID}; // PWM var

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {
  // --- PLATFORM DESIGN PARAMETERS ---
  // Use servo horns?
  bool horn = true;
  
  // Rod centre-to-centre length [mm]
  float s = 177;
  
  // Horn centre-to-centre length [mm]
  float a = 16;
  
  // Rod-platform joints (platform coords.) [mm]
  // These are precalculated, see Jin for source
  Point P[6];
  P[0].X() = 57.4337;
  P[0].Y() = -26.6658;
  P[0].Z() = 0;
  P[1].X() = 57.4337;
  P[1].Y() = 26.6658;
  P[1].Z() = 0;
  P[2].X() = 0;
  P[2].Y() = 60.96;
  P[2].Z() = 0;
  P[3].X() = -57.4337;
  P[3].Y() = 26.6658;
  P[3].Z() = 0;
  P[4].X() = -57.4337;
  P[4].Y() = -26.6658;
  P[4].Z() = 0;
  P[5].X() = 0;
  P[5].Y() = -60.96;
  P[5].Z() = 0;

  // Rod-platform joints (platform coords.) [mm]
  // These are precalculated, see Jin for source
  Point B[6];
  B[0].X() = 86.4235;
  B[0].Y() = -33.7820;
  B[0].Z() = 0;
  B[1].X() = 86.4235;
  B[1].Y() = 33.7820;
  B[1].Z() = 0;
  B[2].X() = -13.9557;
  B[2].Y() = 91.7359;
  B[2].Z() = 0;
  B[3].X() = -72.4678;
  B[3].Y() = 57.9539;
  B[3].Z() = 0;
  B[4].X() = -72.4678;
  B[4].Y() = -57.9539;
  B[4].Z() = 0;
  B[5].X() = -13.9557;
  B[5].Y() = -91.7359;
  B[5].Z() = 0;

  // Servo horn plane angles rel. to x-axis (following right-hand rule) [deg]
  // Use if servo horns point in
  // float beta[6] = {90, 270, 210, 30, 330, 150};

  // Use if servo horns point out
  float beta[6] = {270, 90, 30, 210, 150, 330};

  // Convert to rads
  for (int i = 0; i < 6; ++i) {
    beta[i] *= deg2rad;
  }

  // --- PLATFORM TRANSLATION AND ROTATION ---
  // Position of platform centroid (base coords) [mm]
  Point T;
  T.X() = 100;
  T.Y() = 0;
  T.Z() = 100;

  // Platform angles (phi, theta, psi) [deg]
  // phi: x-rotation
  // theta: y-rotation
  // psi: z-rotation
  float Pang[3] = {0, 5, 0};

  // --- SETUP CALCULATION VARIABLES ---
  // Convert to rads
  for (int i = 0; i < 3; ++i) {
    Pang[i] *= deg2rad;
  }

  // Initialize leg vector array [mm]
  Point L[6];
  for (int i = 0; i < 6; ++i) {
    L[i].X() = 0;
    L[i].Y() = 0;
    L[i].Z() = 0;
  }

  float alpha[6] = {0, 0, 0, 0, 0, 0};
  getAlpha(T, Pang, horn, alpha, beta, P, B, s, a);
}
