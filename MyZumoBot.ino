#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <Wire.h>
#include <LSM303.h>
#include <stdarg.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <MemoryFree.h>

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
#define BATTERY_LEVEL       1
#define BLUETOOTH_RX        2
#define BUZZER_PIN          3  
#define SERVO_PIN           4
#define SENSOR_PIN          5
#define BLUETOOTH_TX        11
#define YELLOW_LED_PIN      13   // Zumo shield yellow led

#define CMD_LEN             4
#define FWD_DURATION        250
#define TURN_DURATION       100

#define EMIT_TIMOUT         250 // min delay between two internal status serial sending
#define DISTANCE_TIMER      200 // max delay between two measurements
#define CHECK_TIMER         500

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
unsigned char buffer[CMD_LEN];
int index;
SoftwareSerial SerialBluetooth( BLUETOOTH_RX, BLUETOOTH_TX);

SimpleTimer timer;
int timer_id              = -1;

int leftDistance, rightDistance, frontDistance;
unsigned char step              = 0;
boolean isMoving                = false;
boolean isTurning               = false;
boolean isSync                  = false;

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
 * publish data through bluetooth serial port
 */
void btPublish(char *key, int value) {
  char buff[128];
//  sprintf(buff, "{%c%s%c:%d}", 34, key, 34, value);
  sprintf(buff, "{\"key\":\"%s\", \"value\":%d}", key, value);
  SerialBluetooth.println(buff);
//  Serial.println(buff);
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
  SerialBluetooth.begin(SERIAL_BAUD);

/*  servoInit();
  servoSetPosition(MAX_SERVO_POSITION/2);*/

  button.waitForButton();
 
  timer_id = timer.setInterval(CHECK_TIMER, checkAll);

  // light Yellow LED
  LOG("Set yellow led start blink for 2 seconds = (20x(50+50)) ms, then remain up");
  digitalWrite(YELLOW_LED_PIN, HIGH);
  for(unsigned char index = 0; index < 10; index ++) {
    delay(50);
    digitalWrite(YELLOW_LED_PIN, LOW);
    delay(50);
    digitalWrite(YELLOW_LED_PIN, HIGH);
  }

  calibrate();
  LOG("[setup] go to loop");
}


/*
 * parse - Get command from bluetooth connection
 * Each command is a simple TLV encoded and always starts with a starting 
 * sync tag (0x47) and a command tag (1 byte)
 * It has a 'sign' field (1 byte) that indicates if the value is negative (1) or not (0)
 * Then, comes the value as a 16 bits integer.
 * command: 0x47 CC SS VV VV
 * 
 */
int parse(unsigned char *buf) {
  int           index    = 0;
  unsigned char command  = 0;
  unsigned char value    = 0;
  unsigned char checksum = 0;
  unsigned char verif    = 0;
  
  // check if buffer starts with a sync tag 0x47
  if( buf[index++] != 0x47 ) {
    LOG("! not a command");
    return -1;
  }
  // parses the command tag and sign
  command   = buf[index++];
  value     = buf[index++];
  checksum  = buf[index++];

  verif = 0x47 ^ command ^ value;
  
  if( verif != checksum ) {
    LOG("! checksum error");
    return -1;
  }
  
  LOG("Received command = 0x%02X - value = 0x%02X", command, value);
  
  if( command == 0x01 ) {        // up
    motors.setLeftSpeed(2*value);
    motors.setRightSpeed(2*value);
  } else if( command == 0x02 ) { // left
    motors.setLeftSpeed(-2*value);
    motors.setRightSpeed(2*value);
  } else if( command == 0x04 ) { // right
    motors.setLeftSpeed(2*value);
    motors.setRightSpeed(-2*value);
  } else if( command == 0x08 ) { // down
    motors.setLeftSpeed(-2*value);
    motors.setRightSpeed(-2*value);
  } /* else if( command == 0x10 ) { // servo 
    servoSetPosition(30*value);
  }*/
  return 0;
}

/*
 * Loop function, run over and over again
 */
void loop() {
  if (SerialBluetooth.available() > 0 ) {    // if COM port is not empty   
    INBYTE = SerialBluetooth.read();         // read next available byte
    if( isSync == true || INBYTE == 0x47 ) { // synced
      isSync = true;
      buffer[index++]=INBYTE;
      if(index == CMD_LEN) {
        index=0;
        isSync = false;
        parse(buffer);
      }
    }
  }
  timer.run();
}

void checkAll() {
  LOG("CHECK ALL");
  getHeading();
  getDistance();
  getMemory();
  getBattery();
}

/*
 * Data monitoring
 */

int getHeading() {
  float heading, relative_heading;
  int speed;
  static float target_heading = averageHeading();

  // Heading is given in degrees away from the magnetic vector, increasing clockwise
  heading = averageHeading();

  // This gives us the relative heading with respect to the target angle
  relative_heading = relativeHeading(heading, target_heading);
  
  btPublish("heading", relative_heading);
}

void getDistance() {
  int distance = measureDistance();
  btPublish("distance", distance);
  //beep(distance*10, 1000/(distance+1));
}

void getMemory() {
  int free = freeMemory();
  btPublish("ram", free);
}

void getBattery() {
  unsigned int batteryVoltage = analogRead(BATTERY_LEVEL) * 5000L * 3/2 / 1023;
  btPublish("battery", batteryVoltage);
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
