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

void Fin::init(PWMServo* _servos, int* _offset_angle_deg){

  servos = _servos;
  offset_angle_deg = _offset_angle_deg;
  write_angle_deg(0);

}


void Fin::write(int linear_speed, int angular_speed, int incline)
{
  std::vector<int> input_amplitude_deg, input_neutral_position_deg, input_phase_shift_deg;
  int propel_L = linear_speed - angular_speed / 2 / ROBOT_RADIUS;
  int propel_R = linear_speed + angular_speed / 2 / ROBOT_RADIUS;
  int amplitude_deg_L, amplitude_deg_R;
  if(propel_L == 0){
    amplitude_deg_L = 0;
  }
  else{
    if(propel_L > AMPLITUDE_RESOLUTION){
      propel_L = AMPLITUDE_RESOLUTION;
    }
    else if(propel_L < -AMPLITUDE_RESOLUTION){
      propel_L = -AMPLITUDE_RESOLUTION;
    }
    amplitude_deg_L = MIN_AMPLITUDE + (MAX_AMPLITUDE - MIN_AMPLITUDE) * abs(propel_L) / AMPLITUDE_RESOLUTION;
  }

  if(propel_R == 0){
    amplitude_deg_R = 0;
  }
  else{
    if(propel_R > AMPLITUDE_RESOLUTION){
      propel_R = AMPLITUDE_RESOLUTION;
    }
    else if(propel_R < -AMPLITUDE_RESOLUTION){
      propel_R = -AMPLITUDE_RESOLUTION;
    }
    amplitude_deg_R = MIN_AMPLITUDE + (MAX_AMPLITUDE - MIN_AMPLITUDE) * abs(propel_R) / AMPLITUDE_RESOLUTION;
  }

  for(int i = 0; i < num_fin_servos / 2; i++){
    input_amplitude_deg.push_back(amplitude_deg_L);

    if(incline > 0){
      input_neutral_position_deg.push_back(incline * (i - (int)(num_fin_servos / 4)) / (num_fin_servos / 2 - 1) );
    }
    else{
      input_neutral_position_deg.push_back(incline * ((int)(num_fin_servos / 4) - i) / (num_fin_servos / 2 - 1) );
    }

    if(propel_L > 0){
      input_phase_shift_deg.push_back(PHASE_SHIFT * (i - (int)(num_fin_servos / 4)));
    }
    else{
      input_phase_shift_deg.push_back(PHASE_SHIFT * ((int)(num_fin_servos / 4) - i));
    }
  }

  for(int i = 0; i < num_fin_servos / 2; i++){
    input_amplitude_deg.push_back(amplitude_deg_R);

    if(incline > 0){
      input_neutral_position_deg.push_back(incline * (i - (int)(num_fin_servos / 4)) / (num_fin_servos / 2 - 1) );
    }
    else{
      input_neutral_position_deg.push_back(incline * ((int)(num_fin_servos / 4) - i) / (num_fin_servos / 2 - 1) );
    }

    if(propel_R > 0){
      input_phase_shift_deg.push_back(PHASE_SHIFT * (i - (int)(num_fin_servos / 4)));
    }
    else{
      input_phase_shift_deg.push_back(PHASE_SHIFT * ((int)(num_fin_servos / 4) - i));
    }
  }


  std::vector<float> output_rad = cpg.generate_new_pose(DEFAULT_FREQUENCY, deg_to_rad(input_amplitude_deg), deg_to_rad(input_neutral_position_deg), deg_to_rad(input_phase_shift_deg));
  write_angle_rad(output_rad);
}

void Fin::write_L(uint16_t propel, bool propel_direction, uint16_t incline, bool incline_direction)
{
  int num_fin = num_fin_servos / 2;
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
  for(int i = 0; i < num_fin; i++){
    input_amplitude_deg.push_back(amplitude_deg);

    if(incline_direction){
      input_neutral_position_deg.push_back(incline * (i - (int)(num_fin / 2)) / (num_fin - 1) );
    }
    else{
      input_neutral_position_deg.push_back(incline * ((int)(num_fin / 2) - i) / (num_fin - 1) );
    }

    if(propel_direction){
      input_phase_shift_deg.push_back(PHASE_SHIFT * (i - (int)(num_fin / 2)));
    }
    else{
      input_phase_shift_deg.push_back(PHASE_SHIFT * ((int)(num_fin / 2) - i));
    }
  }

  std::vector<float> output_rad = cpg.generate_new_pose(DEFAULT_FREQUENCY, deg_to_rad(input_amplitude_deg), deg_to_rad(input_neutral_position_deg), deg_to_rad(input_phase_shift_deg));
  write_L_angle_rad(output_rad);
}


void Fin::write_R(uint16_t propel, bool propel_direction, uint16_t incline, bool incline_direction)
{
  int num_fin = num_fin_servos / 2;
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
  for(int i = 0; i < num_fin; i++){
    input_amplitude_deg.push_back(amplitude_deg);

    if(incline_direction){
      input_neutral_position_deg.push_back(incline * (i - (int)(num_fin / 2)) / (num_fin - 1) );
    }
    else{
      input_neutral_position_deg.push_back(incline * ((int)(num_fin / 2) - i) / (num_fin - 1) );
    }

    if(propel_direction){
      input_phase_shift_deg.push_back(PHASE_SHIFT * (i - (int)(num_fin / 2)));
    }
    else{
      input_phase_shift_deg.push_back(PHASE_SHIFT * ((int)(num_fin / 2) - i));
    }
  }

  std::vector<float> output_rad = cpg.generate_new_pose(DEFAULT_FREQUENCY, deg_to_rad(input_amplitude_deg), deg_to_rad(input_neutral_position_deg), deg_to_rad(input_phase_shift_deg));
  write_R_angle_rad(output_rad);
}

void Fin::write_angle_rad(const std::vector<float>& angle_rad){
  write_angle_deg(rad_to_deg(angle_rad));
}

void Fin::write_angle_deg(const std::vector<int>& angle_deg){
  for(int i = 0; i < num_fin_servos / 2; i++){
    servos[i].write(90 + (offset_angle_deg[i] + angle_deg[i]) * 180 / SERVO_ANGLE_RANGE);
  }
  for(int i = 0; i < num_fin_servos / 2; i++){
    servos[i + num_fin_servos / 2].write(90 + (offset_angle_deg[i + num_fin_servos / 2] - angle_deg[i + num_fin_servos / 2]) * 180 / SERVO_ANGLE_RANGE);
  }
}

void Fin::write_angle_rad(float angle_rad){
  write_L_angle_rad(angle_rad);
  write_R_angle_rad(angle_rad);
}

void Fin::write_angle_deg(int angle_deg){
  write_L_angle_deg(angle_deg);
  write_R_angle_deg(angle_deg);
}


void Fin::write_L_angle_rad(const std::vector<float>& angle_rad){
  write_L_angle_deg(rad_to_deg(angle_rad));
}

void Fin::write_L_angle_deg(const std::vector<int>& angle_deg){
  for(int i = 0; i < num_fin_servos / 2; i++){
    servos[i].write(90 + (offset_angle_deg[i] + angle_deg[i]) * 180 / SERVO_ANGLE_RANGE);
  }
}

void Fin::write_L_angle_rad(float angle_rad){
  write_L_angle_deg(rad_to_deg(angle_rad));
}

void Fin::write_L_angle_deg(int angle_deg){
  for(int i = 0; i < num_fin_servos / 2; i++){
    servos[i].write(90 + (offset_angle_deg[i] + angle_deg) * 180 / SERVO_ANGLE_RANGE);
  }
}



void Fin::write_R_angle_rad(const std::vector<float>& angle_rad){
  write_L_angle_deg(rad_to_deg(angle_rad));
}

void Fin::write_R_angle_deg(const std::vector<int>& angle_deg){
  for(int i = 0; i < num_fin_servos / 2; i++){
    servos[i + num_fin_servos / 2].write(90 + (offset_angle_deg[i + num_fin_servos / 2] - angle_deg[i]) * 180 / SERVO_ANGLE_RANGE);
  }
}

void Fin::write_R_angle_rad(float angle_rad){
  write_L_angle_deg(rad_to_deg(angle_rad));
}

void Fin::write_R_angle_deg(int angle_deg){
  for(int i = 0; i < num_fin_servos / 2; i++){
    servos[i + num_fin_servos / 2].write(90 + (offset_angle_deg[i + num_fin_servos / 2] - angle_deg) * 180 / SERVO_ANGLE_RANGE);
  }
}


Fin::~Fin() {
	// TODO Auto-generated destructor stub
}

