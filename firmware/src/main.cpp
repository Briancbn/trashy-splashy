#include <Arduino.h>

#include "Fin.h"
#include "PID.h"
#include "Config.h"
#include "PWMServo.h"
#include "Razorimu.h"
#include "Command.h"

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

bool start_flag = false;
int input_linear_speed = 0, input_angular_speed = 0;

long baud = 115200;
char cmd;
char argv1[256];
char argv2[256];
float arg1;
float arg2;
int arg = 0;
int index0 = 0;

void reset_command(){
    cmd = ' ';
    memset(argv1, 0, sizeof(argv1));
    memset(argv2, 0, sizeof(argv2));
    arg1 = 0;
    arg2 = 0;
    arg = 0;
    index0 = 0;
}

void run_command()
{
    arg1 = atof(argv1);
    arg2 = atof(argv2);
    switch (cmd)
    {
    case GET_BAUDRATE:
        Serial.println(baud);
        break;
    case GET_DEVICE:
        Serial.println("Teensy3.2");
        break;
    case START:
        Serial.println("OK");
        start_flag = true;
        desired_angle = razor_imu.yaw;
        robot.write_angle_deg(0);
        break;
    case STOP:
        Serial.println("OK");
        start_flag = false;
        robot.write_angle_deg(90);
        input_linear_speed = 0;
        input_angular_speed = 0;
        break;
    case LINEAR_INPUT:
        input_linear_speed = arg1;
        Serial.println("OK");
        break;
    case ANGULAR_INPUT:
        input_angular_speed = arg1;
        Serial.println("OK");
        break;
    case GET_YAW:
        Serial.println(razor_imu.yaw);
        break;
    case GET_PITCH:
        Serial.println(razor_imu.pitch);
        break;
    case GET_ROLL:
        Serial.println(razor_imu.roll);
        break;
    case GET_AX:
        Serial.println(razor_imu.ax);
        break;
    case GET_AY:
        Serial.println(razor_imu.ay);
        break;
    case GET_AZ:
        Serial.println(razor_imu.az);
        break;
    case GET_GX:
        Serial.println(razor_imu.gx);
        break;
    case GET_GY:
        Serial.println(razor_imu.gy);
        break;
    case GET_GZ:
        Serial.println(razor_imu.gz);
        break;

    }
}

uint32_t lastBlink = 0;
void blinkLED()
{
  static bool ledState = false;
  digitalWrite(13, ledState);
  ledState = !ledState;
}

void print_imu(){
        Serial.print(razor_imu.yaw);
        Serial.print('\t');
        Serial.print(razor_imu.pitch);
        Serial.print('\t');
        Serial.print(razor_imu.roll);
        Serial.print('\t');
        Serial.print(razor_imu.ax);
        Serial.print('\t');
        Serial.print(razor_imu.ay);
        Serial.print('\t');
        Serial.print(razor_imu.az);
        Serial.print('\t');
        Serial.print(razor_imu.gx);
        Serial.print('\t');
        Serial.print(razor_imu.gy);
        Serial.print('\t');
        Serial.print(razor_imu.gz);
        Serial.print('\n');    
}

void setup() {
    for(int i = 0; i < 6; i++){
        servos[i].attach(pins[i]);
    }
    // put your setup code here, to run once:
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    robot.init(servos, offset);
    delay(1000);   

//    robot.write_angle_deg(0);
    
    delay(1000);    
    
    Serial.begin(baud);

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


    while(Serial.available() > 0){
        char chr = Serial.read();
        if (chr == 13){
            if (arg == 1) argv1[index0] = ' ';
            else if (arg == 2) argv2[index0] = ' ';
            run_command();
            reset_command();
        }
        else if (chr == ' '){
            if (arg == 0) arg = 1;
            else if (arg == 1){
                argv1[index0] = ' ';
                arg = 2;
                index0 = 0;
            }
        }
        else{
            if (arg == 0){
                cmd = chr;
            }
            else if (arg == 1){
                argv1[index0] = chr;
                index0++;
            }
            else if (arg == 2){
                argv2[index0] = chr;
                index0++;
            }
        }
    }
    


    if(imu_timer > 1000 * IMU_UPDATE_RATE){
        imu_timer = 0;
        razor_imu.update();
        //print_imu();
    }

    if(cpg_timer > 1000 * UPDATE_INTERVAL){
        cpg_timer = 0;

        if (start_flag)
        {
            if (millis() > lastBlink + 200)
            {
                blinkLED();
                lastBlink = millis();
            }

            desired_angle += RAD_TO_DEG * input_angular_speed * UPDATE_INTERVAL;
            float err = scale_angle(razor_imu.yaw - desired_angle);
            pid.load_err(err);
            robot.write(input_linear_speed, pid.get_angular(), 0);
        }
    }
}