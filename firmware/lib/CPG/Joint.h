/*
 * Joint.h
 *
 *  Created on: 17 Mar 2018
 *      Author: Chen Bainian
 * 
 */

#ifndef JOINT_H_
#define JOINT_H_

#include "Arduino.h"

namespace CPG {

class Joint {

public:
	Joint(float _frequency, float _alpha, float _beta, float _update_interval, float initial_amplitude, float initial_neutral_position, float initial_phase_shift);
	Joint(float _frequency, float _alpha, float _beta, float _update_interval);

	const void prepare_joint(float _frequency);
	// Do CPG on amplitude and neutral position inside joint
	const void joint_CPG(float input_amplitude, float input_neutral_position);

	// Calculate the output by integrating
	const void update_joint_state(float _phase_shift_t2);

	// return the value of the output
	float get_output() const;

	// return the value of the phase shift and its change over time
	float get_phase_shift() const;
	float get_phase_shift_t() const;

	// The total number of joints
	static int total_joint;

	virtual ~Joint();

private:

	// Construct the sine wave parameters and their first order time derivatives: [0] is past values, [1] is present values.
	float amplitude[2], neutral_position[2], phase_shift[2];
	float amplitude_t, neutral_position_t, phase_shift_t;

	// Calculated values based on CPG
	float amplitude_t2, neutral_position_t2, phase_shift_t2;

	// The update_interval with respect to second
	float update_interval;

    // Frequency
    float frequency;

	// Parameters for the joint
	float alpha, beta;

	float output;




	const void update_current_state(){
		/*
		 * Simple explanation on update_current_state
		 *
		 * Below are the meaning of the value stored after integration
		 * ________ __________________________________________________________________
		 *         |							|					|				  |
		 *         | 		past amplitude 		| current amplitude | output amplitude|
		 *         | 		amplitude[0]      	| amplitude[1]      |				  |
		 * ________|____________________________|___________________|_________________|
		 *									 \		 /
		 *            						  \	    /
		 *  							_______\___/___________________
		 *  						   |							   |
		 * 							   | current first order amp change|
		 * 							   |		amplitude_t 		   |
		 * 							   |_______________________________|
		 *
		 * Let amplitude[0] = amplitude[1], to store the current amplitude
		 */
		amplitude[0] = amplitude[1];

		neutral_position[0] = neutral_position[1];

		phase_shift[0] = phase_shift[1];

	}




	const void integration(){
		/*
		 * Simple explanation on integration
		 * ________ ______________________________________________________________________
		 *         |							|					    |				  |
		 *         | 		past amplitude 		|   current amplitude   | output amplitude|
		 *         | 					      	|   amplitude[0]        | amplitude[1]    |
		 * ________|____________________________|_______________________|_________________|
		 *									 \		 /               \      /
		 *            						  \	    /                 \    /
		 *  					 ______________\___/___________ _______\__/________________
		 *  					|							   |					        |
		 * 						|current first order amp change| new first order amp change |
		 * 						|		amplitude_t 		   | amplitude_t += amplitude_t2|
		 * 						|______________________________|____________________________|
		 * 						                           \        /
		 * 						                            \      /
		 * 						                 ____________\____/________________
		 * 						                |                                  |
		 * 						                |calculated second order amp change|
		 * 						                |         amplitude_t2             |
		 * 						                |__________________________________|
		 *
		 *
		 */

		amplitude_t += amplitude_t2 * update_interval;
		amplitude[1] = amplitude[0] + amplitude_t * update_interval;

		neutral_position_t += neutral_position_t2 * update_interval;
		neutral_position[1] = neutral_position[0] + neutral_position_t * update_interval;

		phase_shift_t += phase_shift_t2 * update_interval;
		phase_shift[1] = phase_shift[0] + phase_shift_t * update_interval;

	}

};

} /* namespace CPG */

#endif /* JOINT_H_ */
