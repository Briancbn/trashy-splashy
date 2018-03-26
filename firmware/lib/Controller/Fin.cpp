/*
 * Fin.cpp
 *
 *  Created on: 20 Mar 2018
 *      Author: Chen Bainian
 */

#include "Fin.h"


Fin::Fin(int8_t _num_fin_servos) 
: num_fin_servos(_num_fin_servos), cpg(_num_fin_servos, DEFAULT_FREQUENCY, ALPHA, BETA, MU, UPDATE_INTERVAL)
{
}

void Fin::init(PWMServo* _servos){

  servos = _servos;
  write(0, true, 0, true);
  ::delay(600);

}

void Fin::write(uint16_t propel, bool propel_direction, uint16_t incline, bool incline_direction)
{
  std::vector<int> input_amplitude_deg, input_neutral_position_deg, input_phase_shift_deg;

  int amplitude_deg;
  if(propel == 0){
    amplitude_deg = 0;
  }
  else{
    if(propel > AMPLITUDE_RESOLUTION){
      propel = AMPLITUDE_RESOLUTION;
    }
    amplitude_deg = MIN_AMPLITUDE + (MAX_AMPLITUDE - MIN_AMPLITUDE) * propel / AMPLITUDE_RESOLUTION;
  }
  for(int i = 0; i < num_fin_servos; i++){
    input_amplitude_deg.push_back(amplitude_deg);

#if defined(ENABLE_INCLINE)
    if(incline_direction){
      input_neutral_position_deg.push_back(incline * (i - (int)(num_fin_servos / 2)) / (num_fin_servos - 1) );
    }
    else{
      input_neutral_position_deg.push_back(incline * ((int)(num_fin_servos / 2) - i) / (num_fin_servos - 1) );
    }
#else
    input_neutral_position_deg.push_back(0);
#endif
    if(propel_direction){
      input_phase_shift_deg.push_back(PHASE_SHIFT * (i - (int)(num_fin_servos / 2)));
    }
    else{
      input_phase_shift_deg.push_back(PHASE_SHIFT * ((int)(num_fin_servos / 2) - i));
    }
  }

  std::vector<float> output_rad = cpg.generate_new_pose(DEFAULT_FREQUENCY, deg_to_rad(input_amplitude_deg), deg_to_rad(input_neutral_position_deg), deg_to_rad(input_phase_shift_deg));
  write_angle_rad(output_rad);
}



void Fin::write_angle_rad(const std::vector<float>& angle_rad){
  write_angle_deg(rad_to_deg(angle_rad));
}

void Fin::write_angle_deg(const std::vector<int>& angle_deg){
  for(int i = 0; i < num_fin_servos; i++){
    servos[i].write(90 + angle_deg[i] * 180 / ANGLE_RANGE);
  }
}

void Fin::write_angle_rad(float angle_rad){
  write_angle_deg(rad_to_deg(angle_rad));
}

void Fin::write_angle_deg(int angle_deg){
  for(int i = 0; i < num_fin_servos; i++){
    servos[i].write(90 + angle_deg * 180 / ANGLE_RANGE);
  }
}

Fin::~Fin() {
	// TODO Auto-generated destructor stub
}

