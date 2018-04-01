#include <Arduino.h>

#include "Fin.h"
#include "PID.h"
#include "Config.h"
#include "PWMServo.h"
#include "Razorimu.h"

int L_pins[] = {L1_PIN, L2_PIN, L3_PIN};
int L_offset[] = {L1_OFFSET, L2_OFFSET, L3_OFFSET};

int R_pins[] = {R1_PIN, R2_PIN, R3_PIN};
int R_offset[] = {R1_OFFSET, R2_OFFSET, R3_OFFSET};

float desired_angle = 0;
float err;

PWMServo L_servos[3];
PWMServo R_servos[3];
Fin L_fin(3), R_fin(3);
elapsedMillis cpg_timer;


Razor_imu razor_imu;
elapsedMillis imu_timer;

PID pid(KP, KI, KD);

int desired_linear_speed, desired_angular_speed;

void robot_write(int linear, int angular){
    int left_vel = linear - angular / 2;
    int right_vel = linear + angular / 2;
    if(left_vel > 0){
        L_fin.write(left_vel, LEFT_PROPEL_DIRECTION, 0, true);
    }
    else{
        L_fin.write(-left_vel, !LEFT_PROPEL_DIRECTION, 0, true);
    }
    if(right_vel > 0){
        R_fin.write(right_vel, RIGHT_PROPEL_DIRECTION, 0, true);
    }
    else{
        R_fin.write(-right_vel, !RIGHT_PROPEL_DIRECTION, 0, true);
    }
}

void setup() {
    for(int i = 0; i < 3; i++){
        L_servos[i].attach(L_pins[i]);
    }
    for(int i = 0; i < 3; i++){
        R_servos[i].attach(R_pins[i]);
    }
    // put your setup code here, to run once:
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    L_fin.init(L_servos, L_offset);
    R_fin.init(R_servos, R_offset);
    delay(1000);   

    L_fin.write_angle_deg(90);
    R_fin.write_angle_deg(-90);
    
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

int counter = 0;
int direction = 1;
int angular = 0;
void loop() {
    if(imu_timer > 1000 * IMU_UPDATE_RATE){
        imu_timer = 0;
        razor_imu.update();
    }

    if(cpg_timer > 1000 * UPDATE_INTERVAL){
        counter++;
        if(counter % 10 == 0){
            angular += 10 * direction;
            if (angular >= 150 || angular <= -150){
                direction *= -1;
            }
        }
        //Serial.println(direction);
        cpg_timer = 0;
        desired_angular_speed = 0;
        desired_linear_speed = 255;
        desired_angle += RAD_TO_DEG * desired_angular_speed * UPDATE_INTERVAL;
        float err = scale_angle(desired_angle - razor_imu.yaw);
        Serial.print(err);
        Serial.print("   ");
        Serial.print(desired_angle);
        Serial.print("   ");
        Serial.println(razor_imu.yaw);
        pid.load_err(err); 
//        robot_write(0,0);
//        L_fin.write(150, LEFT_PROPEL_DIRECTION, 0, true);
        robot_write(desired_linear_speed, pid.get_angular());
    }
}