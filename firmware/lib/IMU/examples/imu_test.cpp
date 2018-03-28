#include <Arduino.h>
#include "Config.h"
#include "Razorimu.h"

Razor_imu razor_imu;
elapsedMillis imu_timer;


void setup(){
    pinMode(13, OUTPUT);
    Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
    razor_imu.init();
    while(razor_imu.update()){
    }
    imu_timer = 0;
    digitalWrite(13, HIGH);
}


void loop(){
    if(imu_timer > 1000 * IMU_UPDATE_RATE){
        imu_timer = 0;
        razor_imu.update();
        Serial.print(razor_imu.row);
        Serial.print('\t');
        Serial.print(razor_imu.pitch);
        Serial.print('\t');
        Serial.println(razor_imu.yaw);
    }

}