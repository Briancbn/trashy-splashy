/*
 * Joint.cpp
 *
 *  Created on: 17 Mar 2018
 *      Author: Chen Bainian
 */

#include "Joint.h"
#include <iostream>

namespace CPG {

Joint::Joint(float _frequency, float _alpha, float _beta, float _update_interval, float initial_amplitude = 0, float initial_neutral_position = 0, float initial_phase_shift = 0)
:   amplitude_t(0), neutral_position_t(0), phase_shift_t(0),
	amplitude_t2(0), neutral_position_t2(0), phase_shift_t2(0),
	update_interval(_update_interval), 
    frequency(_frequency),
    alpha(_alpha), beta(_beta),
	output(0)
{
	// initialize all the values
	for(int i = 0; i < 2; i++){
		amplitude[i] = initial_amplitude;
		neutral_position[i] = initial_neutral_position;
		phase_shift[i] = initial_phase_shift;
	}

	// Record the number of total joint
	total_joint++;
}

Joint::Joint(float _frequency, float _alpha, float _beta, float _update_interval)
:   amplitude_t(0), neutral_position_t(0), phase_shift_t(0),
	amplitude_t2(0), neutral_position_t2(0), phase_shift_t2(0),
	update_interval(_update_interval), 
    frequency(_frequency),
    alpha(_alpha), beta(_beta),
	output(0)
{
	// initialize all the values as zero
	for(int i = 0; i < 2; i++){
		amplitude[i] = 0;
		neutral_position[i] = 0;
		phase_shift[i] = 0;
	}

	// Record the number of total joint
	total_joint++;
}



const void Joint::prepare_joint(float _frequency){
    frequency = _frequency;
	update_current_state();
}

const void Joint::joint_CPG(float input_amplitude, float input_neutral_position){

	// Implement CPG formulas on the amplitude and neutral_position
	amplitude_t2  = (-1) * pow(alpha * frequency, 2) * (amplitude[0] - input_amplitude) - 2 * alpha * frequency * amplitude_t;
	neutral_position_t2  = (-1) * pow(beta * frequency, 2) * (neutral_position[0] - input_neutral_position) - 2 * beta * frequency * neutral_position_t;

}

const void Joint::update_joint_state(float _phase_shift_t2){
	phase_shift_t2 = _phase_shift_t2;
	integration();
	output = neutral_position[1] + amplitude[1] * sin(phase_shift[1]);
}




float Joint::get_phase_shift() const{
	return phase_shift[0];
}

float Joint::get_phase_shift_t() const{
	return phase_shift_t;
}

float Joint::get_output() const{
	return output;
}

int Joint::total_joint = 0;

Joint::~Joint() {
	total_joint--;
}

} /* namespace CPG */
