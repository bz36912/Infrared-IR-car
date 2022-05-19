/*
 *  This program is write a for an infrared (IR) remote controlled robotics car. The instructions for this
 *  project can be found on https://www.instructables.com/How-to-Make-a-Infrared-IR-Remote-Controlled-Car-Wi/
 *  
 *  This program is free to use.
 ************************************************************************************
 * Information on the infrared sensor library, called 'IRremote.hpp'
 * 
 * MIT License 
 * Copyright (c) 2020-2022 Armin Joachimsmeyer
 * For more inforamtion: https://github.com/Arduino-IRremote/Arduino-IRremote.
 ************************************************************************************ 
  */
#include <Arduino.h>
#include "PinDefinitionsAndMore.h"

#include <IRremote.hpp>

//control pins for left and right motors
const int leftSpeed = 6; //means pin 6 on the Arduino controls the speed of left motor
const int left1 = 7; //left 1 and left 2 control the direction of rotation of left motor
const int left2 = 8;
const int rightSpeed = 9;
const int right1 = 10;
const int right2 = 11;

//Use Pin 2 for infrared signal, when building the circuit. 
//You don't need to define an infrared pin number, since the "PinDefinitionsAndMore.h" library has already done so

//setting the speed. In this case, more speed also means more torque due to the nature of PWM in Arduino
const int carSpeed = 255; //a whole number between 150 to 255. A larger value means more speed.
//how much the car turns left/right
const int turnTime = 100; //in millisec. Bigger value means bigger turn


void setup() {
    Serial.begin(115200);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.print(F("at pin "));
    Serial.println(IR_RECEIVE_PIN);

    pinMode(right1, OUTPUT);
    pinMode(right2, OUTPUT);
    pinMode(rightSpeed, OUTPUT);
    pinMode(left1, OUTPUT);
    pinMode(left2, OUTPUT);
    pinMode(leftSpeed, OUTPUT);
    
    analogWrite(rightSpeed, carSpeed);
    analogWrite(leftSpeed, carSpeed);
}

void loop() {
    /*
     * Check if received data is available and if yes, try to decode it.
     * Decoded result is in the IrReceiver.decodedIRData structure.
     *
     * E.g. command is in IrReceiver.decodedIRData.command
     * address is in command is in IrReceiver.decodedIRData.address
     * and up to 32 bit raw data in IrReceiver.decodedIRData.decodedRawData
     */
    if (IrReceiver.decode()) {

        // Print a short summary of received data
        IrReceiver.printIRResultShort(&Serial);
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            // We have an unknown protocol here, print more info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
        }
        Serial.println();

        /*
         * !!!Important!!! Enable receiving of the next value,
         * since receiving has stopped after the end of the current received data packet.
         */
        IrReceiver.resume(); // Enable receiving of the next value

        /*
         * Finally, check the received data and perform actions according to the received command
         */
        if (IrReceiver.decodedIRData.command == 0x44) { //drive forward
            Serial.println("forward");
            digitalWrite(right1, HIGH); //the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
            digitalWrite(right2, LOW);
            digitalWrite(left1, HIGH);
            digitalWrite(left2, LOW);
        } else if (IrReceiver.decodedIRData.command == 0x1D) { //drive backward
            Serial.println("backward");
            digitalWrite(right1, LOW); //the right motor rotates BACKWARDS when right1 is LOW and right2 is HIGH
            digitalWrite(right2, HIGH);
            digitalWrite(left1, LOW);
            digitalWrite(left2, HIGH);
        } else if (IrReceiver.decodedIRData.command == 0x48) { //turn right
            Serial.println("right");
            digitalWrite(right1, LOW);
            digitalWrite(right2, HIGH);
            digitalWrite(left1, HIGH);
            digitalWrite(left2, LOW);
            delay(turnTime);
            stopCar(); //scroll to the bottom to see definition of this function
        } else if (IrReceiver.decodedIRData.command == 0x1C) { //turn left
            Serial.println("left");
            digitalWrite(right1, HIGH);
            digitalWrite(right2, LOW);
            digitalWrite(left1, LOW);
            digitalWrite(left2, HIGH);
            delay(turnTime);
            stopCar();
        } else if (IrReceiver.decodedIRData.command == 0x5C) { //stop or brake
            Serial.println("stop");
            stopCar();
        } 
    }
}

void stopCar(){
        digitalWrite(right1, LOW);
        digitalWrite(right2, LOW);
        digitalWrite(left1, LOW);
        digitalWrite(left2, LOW);  
}
