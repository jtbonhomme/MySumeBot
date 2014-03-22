# MySumoBot

This is simple program for Polulo Zumo robot (http://www.pololu.com)

# Install

Open MyZumoBot.ino into Arduino IDE and upload it in your Robot.

You can install the bluetooth remote server with the following commands:

    % git clone https://github.com/jtbonhomme/botRC.git
    % cd botRC
    % npm install
    % grunt
    % node api

Then open your chrome browser on http://localhost:3000

# ZumoBot Hardware pin configuration

    Arduino analog pin 1   : battery level
    Arduino digital pin 2  : bluetooth HC-06 module Tx (=> arduino Rx)
    Arduino digital pin 3  : buzzer
    Arduino digital pin 4  : servo signal pin
    Arduino digital pin 5  : Parallax PING))) ultrasonic sensor out
    Arduino digital pin 7  : Right Motor Direction
    Arduino digital pin 8  : Left Motor Direction
    Arduino digital pin 9  : Right Motor Speed
    Arduino digital pin 10 : Left Motor Speed
    Arduino digital pin 11 : bluetooth HC-06 module Rx (=> arduino Tx)
    Arduino digital pin 12 : Zumo User Push Button
    Arduino digital pin 13 : Zumo Yellow LED

# Dependencies

This program needs external librairies to work properly:

* Pololu libraries (https://github.com/pololu/zumo-shield)
    * ZumoMotors.h
    * Pushbutton.h
    * Wire.h

* LSM303 Arduino library (https://github.com/pololu/lsm303-arduino)
    * LSM303.h

* Simple Timer library (http://playground.arduino.cc/Code/SimpleTimer)
    * SimpleTimer.h
