/*
 * Razorimu.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: Chen Bainian
 */

#include "Arduino.h"
#include "Config.h"
#include "Razorimu.h"

Razor_imu::Razor_imu() {
}

void Razor_imu::init(){
	RAZOR_IMU_SERIAL.begin(RAZOR_IMU_BAUD);
	RAZOR_IMU_SERIAL.println("#o1");
}

bool Razor_imu::update(){
	char incomingByte;
	String incomingString;
	int counter = 0;

    // send data only when you receive data:
    if (RAZOR_IMU_SERIAL.available() > 0 && RAZOR_IMU_SERIAL.read() == '#') {
        while (RAZOR_IMU_SERIAL.available() > 0) {
            incomingByte = RAZOR_IMU_SERIAL.read();
            incomingString += incomingByte;
       }
			Serial.println(incomingString);
 
        String word = "";
        for(uint i = 1; i < incomingString.length() - 1; i++){
            if(incomingString[i] == ','){
				if(counter == 0){
                	row = word.toFloat();
				}
				else if(counter == 1){
					pitch = word.toFloat();
				}
				counter += 1;
                word = "";
            }
            else{
                word += incomingString[i];
            }
        }
        yaw = word.toFloat();
        incomingString = "";
		return true;
    }
	else{
		return false;
	}

}

Razor_imu::~Razor_imu() {
}

