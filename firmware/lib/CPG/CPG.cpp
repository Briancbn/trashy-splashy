/*
 * CPG.cpp
 *
 *  Created on: 17 Mar 2018
 *      Author: Chen Bainian
 */

#include "CPG.h"

namespace CPG {

CPG::CPG(int8_t _joint_num, float _frequency, float _alpha, float _beta, float _mu, float _update_interval, const std::vector<float>& initial_amplitude, const std::vector<float>& initial_neutral_position, const std::vector<float>& initial_phase_shift)
: joint_num(_joint_num),
  mu(_mu),
  frequency(_frequency)
{
	// Initialize the Joint vector with initial condition
	for(int i = 0; i < joint_num; i++){
		joint.push_back(Joint(frequency, _alpha, _beta, _update_interval, initial_amplitude[i], initial_neutral_position[i], initial_phase_shift[i]));
	}
}


CPG::CPG(int8_t _joint_num, float _frequency, float _alpha, float _beta, float _mu, float _update_interval)
: joint_num(_joint_num),
  mu(_mu),
  frequency(_frequency)
{
	// Initialize the Joint vector without initial condition
	for(int i = 0; i < joint_num; i++){
		joint.push_back(Joint(frequency, _alpha, _beta, _update_interval));
	}


}

const std::vector<float> CPG::generate_new_pose(float _frequency, const std::vector<float>& input_amplitude, const std::vector<float>& input_neutral_position, const std::vector<float>& input_phase_shift)
{
    frequency = _frequency;
	std::vector<float> output, phase_shift, phase_shift_t, phase_shift_t2;


	// Prepare the joint data to be updated first
	for(int i = 0; i < joint_num; i++){
		joint[i].prepare_joint(frequency);

		// Get the current phase shift and the change of phase shift
		phase_shift.push_back(joint[i].get_phase_shift());
		phase_shift_t.push_back(joint[i].get_phase_shift_t());
		
	}
	

	// Run CPG on all three parameters
	for(int i = 0; i < joint_num; i++){
		// Run CPG on amplitude and neutral position
		joint[i].joint_CPG(input_amplitude[i], input_neutral_position[i]);

		// Run CPG on phase shift
		// Calculate relative error
		float err_sum = 0;
		for(int j = 0; j < joint_num; j++){
			float err = (phase_shift[i] - phase_shift[j]) - (input_phase_shift[i] - input_phase_shift[j]);
			err_sum += err;
		}

		// calculate second order change of the phase shift
		phase_shift_t2.push_back((-1) * pow(mu * frequency, 2) * err_sum - 2 * (joint_num - 1) * mu * frequency * ((phase_shift_t[i]) - 2 * M_PI * frequency));

		joint[i].update_joint_state(phase_shift_t2[i]);
		output.push_back(joint[i].get_output());
	}
	
	return output;
}

CPG::~CPG() {
	// TODO Auto-generated destructor stub
}

} /* namespace CPG */
