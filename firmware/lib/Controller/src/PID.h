#ifndef PID_H
#define PID_H

#include "Arduino.h"
#include "Config.h"

class PID
{
  private:
    float kp, ki, kd;
    float p_gain, i_gain, d_gain;
    float err;
    int angular, linear;
    float angular_h[ANGULAR_HISTORY_LENGTH];
    int index;

    const void record_angular(){
        angular_h[index] = angular;
        index += 1;
        if(index == ANGULAR_HISTORY_LENGTH){
            index = 0;
        }
    }

    const void classify_angular_type(){
        int angular_sum = sum_angular();
        if(abs(angular_sum) < GRADUAL_TURN_THRESHOLD){
            angular = angular_sum / ANGULAR_HISTORY_LENGTH;
        }
        else if(abs(angular_sum) < STRAIGHT_THRESHOLD){
            angular = 0;
        }
    }

    const int16_t sum_angular(){
        int sum = 0;
        for(int i  = 0; i < ANGULAR_HISTORY_LENGTH; i++){
            sum += angular_h[i];
        }
        return sum;
    }


  public:
    PID(float _kp, float _ki, float _kd)
    : kp(_kp), ki(_ki), kd(_kd), 
      p_gain(0), i_gain(0), d_gain(0),
      err(0), index(0)
    {
        for(int i = 0; i < ANGULAR_HISTORY_LENGTH; i++){
            angular_h[i] = 0;
        }
    }

    const void load_err(float new_err){
        p_gain = kp * new_err; // calculate p gain
        i_gain += ki * new_err; // integrate i gain
        d_gain = kd * (new_err - err); // calculate the d gain
        err = new_err; // update the error
        i_gain = constrain(i_gain, -255, 255);
        float total_gain = p_gain + i_gain + d_gain;
        angular = total_gain;

        if(LINE_STATE_DETERMINATION){
            record_angular();
            classify_angular_type();
        }

        if(angular != 0){
            linear = MAX_ANGULAR_POWER - abs(angular) / 2;
        }
        else
        {
            linear = MAX_LINEAR_POWER;
        }
    }

    const int16_t get_angular() const{
        return angular;
    }

    const int16_t get_linear() const{
        return linear;
    }

};

#endif