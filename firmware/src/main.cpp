#include <Arduino.h>

#include "Fin.h"
#include "PID.h"
#include "Config.h"
#include "PWMServo.h"
#include "Razorimu.h"

int pins[] = {L1_PIN, L2_PIN, L3_PIN, R1_PIN, R2_PIN, R3_PIN};
int offset[] = {L1_OFFSET, L2_OFFSET, L3_OFFSET, R1_OFFSET, R2_OFFSET, R3_OFFSET};

float desired_angle = 0;
float err;

PWMServo servos[6];
Fin robot(6);
elapsedMillis cpg_timer;


Razor_imu razor_imu;
elapsedMillis imu_timer;

PID pid(KP, KI, KD);

int desired_linear_speed, desired_angular_speed;

void setup() {
    for(int i = 0; i < 6; i++){
        servos[i].attach(pins[i]);
    }
    // put your setup code here, to run once:
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    robot.init(servos, offset);
    delay(1000);   

    robot.write_angle_deg(0);
    
    delay(1000);    
    
    Serial.begin(115200);

    razor_imu.init();
    while(!razor_imu.update() || millis() < 5000){
        delay(30);
    }
    desired_angle = razor_imu.yaw;
    imu_timer = 0;
    cpg_timer = 0;
    digitalWrite(13, HIGH);


}

float scale_angle(float angle){
    while(angle > 180 || angle <= -180){
        if(angle > 180){
            angle -= 360;
        }
        else if(angle <= -180){
            angle += 360;
        }
    }
    return angle;
}

void loop() {
    if(imu_timer > 1000 * IMU_UPDATE_RATE){
        imu_timer = 0;
        razor_imu.update();
    }

    if(cpg_timer > 1000 * UPDATE_INTERVAL){
        cpg_timer = 0;
        desired_angular_speed = 0;
        desired_linear_speed = 255;
        desired_angle += RAD_TO_DEG * desired_angular_speed * UPDATE_INTERVAL;
        float err = scale_angle(razor_imu.yaw - desired_angle);
        pid.load_err(err);
//        robot_write(0,0);
//        L_fin.write(150, LEFT_PROPEL_DIRECTION, 0, true);
        robot.write(desired_linear_speed, pid.get_angular(), 0);
    }
}