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

}

bool Razor_imu::update(){
	char incomingByte;
	String incomingString;
	int counter = 0;
	RAZOR_IMU_SERIAL.println("#f");
    // send data only when you receive data:
    if (RAZOR_IMU_SERIAL.available() > 0 && RAZOR_IMU_SERIAL.read() == '#') {
        while (RAZOR_IMU_SERIAL.available() > 0) {
            incomingByte = RAZOR_IMU_SERIAL.read();
            incomingString += incomingByte;
       }
 
        String word = "";
        for(uint i = 1; i < incomingString.length() - 1; i++){
            if(incomingString[i] == ','){
				switch(counter)
                {
                case 0:                
                	yaw = word.toFloat();
                    break;
				case 1:
					pitch = word.toFloat();
                    break;
                case 2:
					roll = word.toFloat();
                    break;
				case 3:
					ax = word.toFloat();
                    break;
				case 4:
					ay = word.toFloat();
                    break;
				case 5:
					az = word.toFloat();
                    break;
				case 6:
					gx = word.toFloat();
                    break;
				case 7:
					gy = word.toFloat();
                    break;
                  
				}
                
				counter += 1;
                word = "";
            }
            else{
                word += incomingString[i];
            }
        }
        gz = word.toFloat();
        incomingString = "";
		return true;
    }
	else{
		return false;
	}

}

Razor_imu::~Razor_imu() {
}

