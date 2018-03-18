/*
 * CPG.h
 *
 *  Created on: 17 Mar 2018
 *      Author: Chen Bainian
 */

#ifndef CPG_H_
#define CPG_H_

#include "Joint.h"
#include <cmath>
#include <vector>
#include <cstdint>

namespace CPG {

class CPG {
public:
	// Constructor with initial condition
	CPG(int8_t _joint_num, float _frequency, float _alpha, float _beta, float _mu, float _update_interval, const std::vector<float>& initial_amplitude, const std::vector<float>& initial_neutral_position, const std::vector<float>& initial_phase_shift);

	// Constructor without initial condition
	CPG(int8_t _joint_num, float _frequency, float _alpha, float _beta, float _mu, float _update_interval);

	// Generate the output position using CPG
	const std::vector<float>generate_new_pose(const std::vector<float>& input_amplitude, const std::vector<float>& input_neutral_position, const std::vector<float>& input_phase_shift);

	virtual ~CPG();

private:
	int8_t joint_num;
	float mu;
	float frequency;
	std::vector<Joint> joint;
};

} /* namespace CPG */

#endif /* CPG_H_ */
