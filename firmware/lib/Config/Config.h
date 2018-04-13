/*
 * Config.h
 *
 *  Created on: 20 Mar 2018
 *      Author: Chen Bainian
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define ROBOT_RADIUS 0.3


/******** Basic Tuning Parameters ********/

// max linear power
#define MAX_LINEAR_POWER 255

// max power for a single motor in an angular motion 
#define MAX_ANGULAR_POWER 255

// PID values
#define KP 1
#define KI 0
#define KD 0


/******** Advance Tuning Parameters ********/

// according to history angular command to determine whether it is a straight line or a slow turn
#define LINE_STATE_DETERMINATION false
#define ANGULAR_HISTORY_LENGTH 70
#define STRAIGHT_THRESHOLD 30
#define GRADUAL_TURN_THRESHOLD 70

// Servo Pins and Offset
#define R1_PIN 5
#define R2_PIN 4
#define R3_PIN 3
#define L1_PIN 22
#define L2_PIN 21
#define L3_PIN 20

#define R1_OFFSET -15
#define R2_OFFSET -10
#define R3_OFFSET -5
#define L1_OFFSET 7
#define L2_OFFSET 5
#define L3_OFFSET 7



#define RAZOR_IMU_BAUD 115200
#define RAZOR_IMU_SERIAL Serial1
#define IMU_UPDATE_RATE 0.03

#define SERVO_ANGLE_RANGE 250

#define DEFAULT_FREQUENCY 1
#define PHASE_SHIFT 60

#define ALPHA 11.68
#define BETA 11.68
#define MU 5.84
#define UPDATE_INTERVAL 0.006

#define LEFT_PROPEL_DIRECTION false
#define RIGHT_PROPEL_DIRECTION true

//#define ENABLE_INCLINE
#define LEFT_INCLINE_DIRECTION true
#define RIGHT_INCLINE_DIRECTION true

#define MAX_AMPLITUDE 40 //In deg
#define MIN_AMPLITUDE 0 //In deg
#define AMPLITUDE_RESOLUTION 255

#define ENABLE_FREQ_CHANGE 
#define MAX_FREQUENCY 2.5
#define MIN_FREQUENCY 1





#endif /* CONFIG_H_ */
