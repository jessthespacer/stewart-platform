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

const int SERVOMIN[6] = {155, 155, 175, 175, 160, 160}; // 'minimum' pulse length count (out of 4096)
const int SERVOMAX[6] = {480, 475, 530, 490, 545, 495}; // 'maximum' pulse length count (out of 4096)
int SERVOMID[6] = {0, 0, 0, 0, 0, 0}; // 'mid' pulse length count (out of 4096)

const int SERVOCHG = 5; // 'change' pulse length count

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
String yn;
String valInput; // Serial input var.
int i=0; // loop index var.
int val[6] = {0, 0, 0, 0, 0, 0}; // PWM var
/**/
  int valInp = 0;
void setup() {
  for(int i=0; i<6; i++)
{
  SERVOMID[i] = floor((SERVOMIN[i] +SERVOMAX[i])/2);
}
for(int i= 0; i<6; i++)
{
  val[i] = SERVOMID[i];
}
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Servo motor actuation using messaging");
  for(i= 0; i<6; i++)
{
  Serial.println(SERVOMID[i]);
  Serial.println(" ");
}
  for(i= 0; i<6; i++)
{
  Serial.print(val[i]);
  Serial.print(" ");
}
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {
  if (Serial.available() > 0) 
    {
      valInput = Serial.readString();
      Serial.println("I received: ");
      Serial.print(valInput);
      valInp = valInput.toInt();
      val[0] = map(valInp,90,-90,SERVOMIN[0],SERVOMAX[0]);
     
    Serial.print(" Servo values = [");
    for (i=0; i<6; i++) {
      Serial.print(val[i]);
      Serial.print(" ");
    }
    Serial.println("]");

    // Update servo commands:
    for (i=0; i<6; i++) {
      pwm.setPWM(i+1, 0, val[i]); // added +1 to match PWM port numbering (pins 1..6 used)
    }
    
  }
}
