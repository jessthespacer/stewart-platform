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
#include <Math.h>
#include <Adafruit_PWMServoDriver.h>
#include <Geometry.h>
#include "revKin.h"

char choice = 'y';
const float deg2rad = M_PI / 180;
const float rad2deg = 180 / M_PI;
int count=0;
const int SERVOMIN[6] = {155, 155, 180, 170, 140, 160}; // 'minimum' pulse length count (out of 4096)
const int SERVOMAX[6] = {480, 475, 535, 524, 540, 495}; // 'maximum' pulse length count (out of 4096)
int SERVOMID[6] = {0, 0, 0, 0, 0, 0}; // 'mid' pulse length count (out of 4096)
const int SERVOCHG = 5; // 'change' pulse length count
String breaker; 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int delta = 16;
int platX = 0;
int platY = 0;
int joyX = 0;
int joyY = 0;
// --- PLATFORM DESIGN PARAMETERS ---
  // Use servo horns?
bool horn = true;

// Rod centre-to-centre length [mm]
float s = 174;
  
// Horn centre-to-centre length [mm]
float a = 44;

// Rod-platform joints (platform coords.) [mm]
// These are precalculated, see Jin for source
Point P[6];

// Rod-platform joints (platform coords.) [mm]
// These are precalculated, see Jin for source
Point B[6];

// Servo horn plane angles rel. to x-axis (following right-hand rule) [deg]
float beta[6] = {180, 0, 300, 120, 60, 240};

// --- PLATFORM TRANSLATION AND ROTATION ---
// Position of platform centroid (base coords) [mm]
Point T;

// Platform angles (phi, theta, psi) [deg]
// phi: x-rotation
// theta: y-rotation
// psi: z-rotation
float Pang[3] = {0, 0, 0};

// Initialize leg vector array [mm]
Point L[6];

float alpha[6] = {0, 0, 0, 0, 0, 0};

String valInput; // Serial input var.
int val[6];

void setup() {
for(int i=0; i<6; i++)
{
  SERVOMID[i] = floor((SERVOMIN[i] +SERVOMAX[i])/2);
}
for(int i= 0; i<6; i++)
{
  val[i] = SERVOMID[i];
}
  /* THESE ARE PRECALCULATED, SEE CALCULATION SHEET ON GDRIVE:         */
  /* Untitled.docx (3B)\Analysis\Stewart Platform Construction Vectors */
  /* IF YOU HAVE QUESTIONS, TALK TO JIN                                */
  // Rod-platform joints (SW model coords.) [mm]
  P[0].X() = -6.34995858;
  P[0].Y() = -89.93340186;

  P[1].X() = 6.34995858;
  P[1].Y() = -89.93340186;

  P[2].X() = 81.05958995;
  P[2].Y() = 39.46747548;

  P[3].X() = 74.70963137;
  P[3].Y() = 50.46592637;

  P[4].X() = -74.70963137;
  P[4].Y() = 50.46592637;

  P[5].X() = -81.05958995;
  P[5].Y() = 39.46747548;

  // Rod-base joints (SW model coords.) [mm]
  B[0].X() = -42.07005255;
  B[0].Y() = -77.13726744;

  B[1].X() = 42.07005255;
  B[1].Y() = -77.13726744;

  B[2].X() = 87.83785946;
  B[2].Y() = 2.134899473;

  B[3].X() = 45.76780691;
  B[3].Y() = 75.00236797;

  B[4].X() = -45.76780691;
  B[4].Y() = 75.00236797;

  B[5].X() = -87.83785946;
  B[5].Y() = 2.134899473;
  
  for (int i = 0; i < 6; ++i) {
    P[i].Z() = 0;
    B[i].Z() = 0;
  }
  // Convert to rads
  for (int i = 0; i < 6; ++i) {
    beta[i] *= deg2rad;
  }
  // Set platform translation position to neutral
  T.X() = 0;
  T.Y() = 0;
  T.Z() = 155;

  // --- SETUP CALCULATION VARIABLES ---
  // Convert to rads
  for (int i = 0; i < 3; ++i) {
    Pang[i] *= deg2rad;
  }

  for (int i = 0; i < 6; ++i) {
    L[i].X() = 0;
    L[i].Y() = 0;
    L[i].Z() = 0;
  }

  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Servo motor actuation using messaging");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {    
  if (Serial.available() > 0) {
  
  joyX = analogRead(A8);
  joyY = analogRead(A9);
  
  if (joyX < 512 - delta) {
    platX = joyX + delta;
  }
  else if (joyX < 512 + delta) {
    platX = 512;
  }
  else {
    platX = joyX - delta;
  }

  if (joyY < 512 - delta) {
    platY = joyY + delta;
  }
  else if (joyY < 512 + delta) {
    platY = 512;
  }
  else {
    platY = joyY - delta;
  }
  Pang[0] = map(platX,0,1007,-5,5)*deg2rad;
  Pang[1] = map(platY,0,1007,-5,5)*deg2rad;
  getAlpha(T, Pang, horn, alpha, beta, P, B, s, a);
    
    

  for (int i=0; i<6; i++){
    
    if (alpha[i]*rad2deg<90 or alpha[i]*rad2deg>-90){
    count++;
    }
    
  }

  if (count==6){
    Serial.print(" Servo values = [");
    for (int i = 0; i < 6; ++i) {
    float alphn = alpha[i] * rad2deg;
    Serial.println(alphn);
    }
    for(int i = 0; i<6; i++){
      if(i!=4){
        val[i] = map(alpha[i]*rad2deg,pow(-1,i)*90,pow(-1,i)*-90,SERVOMIN[i],SERVOMAX[i]);       
      }       
     if(i == 4)
     {
        val[i] = (-2.3846*((alpha[i]*rad2deg)-3)) + 322;
     }
    }
    Serial.println("]");
  
// Update servo commands:
  
  for (int i=0; i<6; i++) {
    pwm.setPWM(i+1, 0, val[i]); // added +1 to match PWM port numbering (pins 1..6 used)
  }
  }
  //
  else{
    Serial.print("Angles are outside of parametric range");
    Serial.println(" Servo values = [");
    for (int i = 0; i < 6; ++i) {
    float alphn = alpha[i] * rad2deg;
    Serial.println(alphn);
    }
  }
  count=0;
 /*for (short unsigned int i = 0; i < 6; ++i) {
  alpha[i] *= deg2rad;
  }*/
  }
}
