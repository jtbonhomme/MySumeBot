#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <Wire.h>
#include <LSM303.h>
#include <stdarg.h>
#include <SimpleTimer.h>

#define DEBUG_MODE

/*
 * MySumoBot
 *
 * Inspired from many example of arduino programs
 * 
 * author:  jtbonhomme@gmail.com
 * version: Version 1.0
 *
 * Hardware pin configuration
 * --------------------------
 * Arduino analog pin 1   : battery level
 * Arduino digital pin 2  : bluetooth HC-06 module Tx (=> arduino Rx)
 * Arduino digital pin 3  : buzzer
 * Arduino digital pin 4  : servo signal pin
 * Arduino digital pin 5  : Parallax PING))) ultrasonic sensor out
 * Arduino digital pin 7  : Right Motor Direction
 * Arduino digital pin 8  : Left Motor Direction
 * Arduino digital pin 9  : Right Motor Speed
 * Arduino digital pin 10 : Left Motor Speed
 * Arduino digital pin 11 : bluetooth HC-06 module Rx (=> arduino Tx)
 * Arduino digital pin 12 : Zumo User Push Button
 * Arduino digital pin 13 : Zumo Yellow LED
 *
 */

// DEFINE
#define SERIAL_BAUD         9600
#define SPEED               300
#define BLUETOOTH_RX        2
#define BUZZER_PIN          3  
#define SERVO_PIN           4
#define SENSOR_PIN          5
#define BLUETOOTH_TX        11
#define YELLOW_LED_PIN      13   // Zumo shield yellow led

#define FWD_DURATION        250
#define TURN_DURATION       100

#define DISTANCE_TIMER      200 // max delay between two measurements

#define SERVO_LEFT_POS      700
#define SERVO_CENTRAL_POS   1500
#define SERVO_RIGHT_POS     2300
#define SERVO_DELAY         750

#define MAX_SERVO_POSITION  3000
#define MIN_DISTANCE        15

#define MOTOR_SPEED         200
#define SPIN_SPEED          150

#define MOTOR_STEP          5000
#define SPIN_STEP           500

#define CALIBRATION_SAMPLES 70   // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS  0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ     0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#define DEVIATION_THRESHOLD 5

// Global variables
LSM303 compass;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
unsigned char INBYTE;
unsigned char buffer[7];
int index;
unsigned int batterie           = 0;

SimpleTimer timer;
int motor_timer_id              = -1;
int distance_timer_id           = -1;

int leftDistance, rightDistance, frontDistance;
int distance;
unsigned char step              = 0;
boolean isMoving                = false;
boolean isTurning               = false;

// This is the time since the last rising edge in units of 0.5us.
uint16_t volatile servoTime     = 0;
// This is the pulse width we want in units of 0.5us.
uint16_t volatile servoHighTime = 3000;
// This is true if the servo pin is currently high.
boolean volatile servoHigh      = false;
 
/*
 * LOG function, send buffer both on Serial and bluetooth
 * usage: LOG("This string has a %d items in it - %s%c",2, "GREAT", '!');
 * Based on SerialPrint example. See http://www.utopiamechanicus.com/article/sprintf-arduino/
 */
void LOG(char *format,...)
{
#ifdef DEBUG_MODE
  char buff[128];
  va_list args;
  va_start (args,format);
  vsnprintf(buff,sizeof(buff),format,args);
  va_end (args);
  buff[sizeof(buff)/sizeof(buff[0])-1]='\0';
  Serial.println(buff);
#endif
}

/*
 * Setup function, run once, when the sketch starts
 * - Initialize serial port
 */
void setup() {
  // initialize serials comm port
#ifdef DEBUG_MODE
  Serial.begin(SERIAL_BAUD);
#endif

  button.waitForButton();

  //calibrate();

  LOG("[setup] Move servo");
  servoInit();
  servoSetPosition(MAX_SERVO_POSITION/2);
  LOG("[setup] Start distance timer");
  distance_timer_id = timer.setInterval(DISTANCE_TIMER, checkDistance);
}

/*
 * Loop function, run over and over again
 */
void loop() {
/*  float heading, relative_heading;
  int speed;
  static float target_heading = averageHeading();

  // Heading is given in degrees away from the magnetic vector, increasing clockwise
  heading = averageHeading();

  // This gives us the relative heading with respect to the target angle
  relative_heading = relativeHeading(heading, target_heading);

  LOG("[loop] Target heading: %d", target_heading);
  LOG("[loop] \t- Actual heading: %d", heading);
  LOG("[loop] \t- Difference: %d", relative_heading);
*/
  timer.run();

  if(step%5 == 0) {
    getDistanceLeft();
  }
  else if(step%5 == 1) {
    getDistanceRight();
  }
  else if(step%5 == 2) {
    lookForward();
  }
  else if(step%5 == 3 && isTurning == false) {
    startSpin();
  }
  else if(step%5 == 4 && isMoving == false) {
    startMove();
  }
}

void getDistanceLeft() {
  LOG("[getDistanceLeft]");
  isMoving = false;
  isTurning = false;
  // check distance to the left
  servoSetPosition(SERVO_LEFT_POS);
  delay(SERVO_DELAY);
  checkDistance();
  leftDistance = distance;
  step++;
}


void getDistanceRight() {
  LOG("[getDistanceRight]");
  isMoving = false;
  isTurning = false;
  servoSetPosition(SERVO_RIGHT_POS);
  delay(SERVO_DELAY);
  checkDistance();
  rightDistance = distance;
  step++;
}

void lookForward() {
  LOG("[lookForward]");
  isMoving = false;
  isTurning = false;
  servoSetPosition(SERVO_CENTRAL_POS);
  step++;
}

void startSpin() {
  LOG("[startSpin]");
  isMoving = false;
  isTurning = true;
  if(leftDistance > rightDistance) {
    LOG("[startSpin] Turn Left");
    go(SPIN_SPEED, -SPIN_SPEED, SPIN_STEP);
  }
  else if(rightDistance > leftDistance) {
    LOG("[startSpin] Turn Right");
    go(-SPIN_SPEED, SPIN_SPEED, SPIN_STEP);
  }
}

void startMove() {
  LOG("[startMove]");
  isMoving = true;
  isTurning = false;
  go(MOTOR_SPEED, MOTOR_SPEED, MOTOR_STEP);
}

/********************************
 * Motors command functions
 ********************************/

void stopMotorsOnTime() {
  LOG("[stopMotorsOnTime] Stop both motors");
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  if(isTurning == true) {
    step++;
  }
}

void stopMotorsAtDistance() {
  LOG("[stopMotorsAtDistance] Stop both motors");
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  if(isTurning == true) {
    step++;
  }
}

void go(int leftSpeed, int rightSpeed, long timer_duration) {
  motors.setLeftSpeed(leftSpeed);
  motors.setRightSpeed(rightSpeed);
  LOG("[go] SetTimeout");
  motor_timer_id = timer.setTimeout(timer_duration, stopMotorsOnTime);
}

void checkDistance() {
  distance = measureDistance();
//  LOG("[checkDistance] Distance : %d", distance);
  beep(distance*10, 1000/(distance+1));

  // prevent from case pulsein function returns 0 (no pulse before timeout ends)
  if((distance > 0) && (distance < MIN_DISTANCE) && (isMoving == true)) {
    stopMotorsAtDistance();
    step++;
  }
}

/*
 * beep
 */
void beep(int freq, int dur) {
  int i;
  //LOG("[beep] beep %d %d", freq, dur);
  for(i = 0; i< dur; i++) {
    digitalWrite(BUZZER_PIN, HIGH); // pin 3
    delayMicroseconds(freq);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(freq);
  }
}


// Setup will calibrate our compass by finding maximum/minimum magnetic readings
void calibrate()
{
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
  unsigned char index;

  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  LOG("[calibrate] starting calibration");

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(SPEED);
  motors.setRightSpeed(-SPEED);

  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    LOG("[calibrate] index: %d", index);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  LOG("[calibrate] max.x : %d ", running_max.x);
  LOG("[calibrate] max.y : %d ", running_max.y);
  LOG("[calibrate] min.x : %d ", running_min.x);
  LOG("[calibrate] max.y : %d ", running_min.y);

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;
}

/*
 * measureDistance
 *
 * return measure from the Parallax PING))) ultrasonic sensor in centimeter
 */
int measureDistance()
{
  unsigned long pulseduration = 0;
  int distance;

  // set pin as output so we can send a pulse
  pinMode(SENSOR_PIN, OUTPUT);
  // set output to LOW
  digitalWrite(SENSOR_PIN, LOW);
  delayMicroseconds(5);

  // now send the 5uS pulse out to activate Ping)))
  digitalWrite(SENSOR_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SENSOR_PIN, LOW);
   
  // now we need to change the digital pin
  // to input to read the incoming pulse
  pinMode(SENSOR_PIN, INPUT);

  // finally, measure the length of the incoming pulse
  pulseduration = pulseIn(SENSOR_PIN, HIGH);

  // divide the pulse length by half
  pulseduration = pulseduration/2; 

  // now convert to centimetres. We're metric here people...
  distance = int(pulseduration/29);
 
  return distance;
}

/************************************
 * SERVO DRIVER LIB
 ************************************/

// This ISR runs after Timer 2 reaches OCR2A and resets.
// In this ISR, we set OCR2A in order to schedule when the next
// interrupt will happen.
// Generally we will set OCR2A to 255 so that we have an
// interrupt every 128 us, but the first two interrupt intervals
// after the rising edge will be smaller so we can achieve
// the desired pulse width.
ISR(TIMER2_COMPA_vect)
{
  // The time that passed since the last interrupt is OCR2A + 1
  // because the timer value will equal OCR2A before going to 0.
  servoTime += OCR2A + 1;
   
  static uint16_t highTimeCopy = 3000;
  static uint8_t interruptCount = 0;
   
  if(servoHigh)
  {
    if(++interruptCount == 2)
    {
      OCR2A = 255;
    }
 
    // The servo pin is currently high.
    // Check to see if is time for a falling edge.
    // Note: We could == instead of >=.
    if(servoTime >= highTimeCopy)
    {
      // The pin has been high enough, so do a falling edge.
      digitalWrite(SERVO_PIN, LOW);
      servoHigh = false;
      interruptCount = 0;
    }
  } 
  else
  {
    // The servo pin is currently low.
     
    if(servoTime >= 40000)
    {
      // We've hit the end of the period (20 ms),
      // so do a rising edge.
      highTimeCopy = servoHighTime;
      digitalWrite(SERVO_PIN, HIGH);
      servoHigh = true;
      servoTime = 0;
      interruptCount = 0;
      OCR2A = ((highTimeCopy % 256) + 256)/2 - 1;
    }
  }
}
 
void servoInit()
{
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
   
  // Turn on CTC mode.  Timer 2 will count up to OCR2A, then
  // reset to 0 and cause an interrupt.
  TCCR2A = (1 << WGM21);
  // Set a 1:8 prescaler.  This gives us 0.5us resolution.
  TCCR2B = (1 << CS21);
   
  // Put the timer in a good default state.
  TCNT2 = 0;
  OCR2A = 255;
   
  TIMSK2 |= (1 << OCIE2A);  // Enable timer compare interrupt.
  sei();   // Enable interrupts.
}
 
void servoSetPosition(uint16_t highTimeMicroseconds)
{
  //LOG("[servoSetPosition] servoSetPosition: %d ms", highTimeMicroseconds);
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  servoHighTime = highTimeMicroseconds * 2;
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
}


// Converts x and y components of a vector to a heading in degrees.
// This function is used instead of LSM303::heading() because we don't
// want the acceleration of the Zumo to factor spuriously into the
// tilt compensation that LSM303::heading() performs. This calculation
// assumes that the Zumo is always level.
template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}
