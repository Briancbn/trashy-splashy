/*
 * Fin.h
 *
 *  Created on: 20 Mar 2018
 *      Author: Chen Bainian
 */

#ifndef FIN_H_
#define FIN_H_


#include "CPG.h"
#include "Config.h"
#include "PWMServo.h"

#include "Arduino.h"
#include "stdint.h"
#include <vector>

class Fin {
public:
	Fin(int8_t _num_fin_servos); // _num_fin_servos must be even
	void init(PWMServo* _servos, int* _offset_angle_deg); // PWM servos needs to define outside of the object
	void write(int linear_speed, int angular_speed, int incline);
	void write_L(uint16_t propel, bool propel_direction, uint16_t incline, bool incline_direction);
	void write_R(uint16_t propel, bool propel_direction, uint16_t incline, bool incline_direction);
	void write_angle_rad(const std::vector<float>& angle_rad);
	void write_angle_deg(const std::vector<int>& angle_deg);
	void write_angle_rad(float angle_rad);
	void write_angle_deg(int angle_deg);
	void write_L_angle_rad(const std::vector<float>& angle_rad);
	void write_L_angle_deg(const std::vector<int>& angle_deg);
	void write_L_angle_rad(float angle_rad);
	void write_L_angle_deg(int angle_deg);
	void write_R_angle_rad(const std::vector<float>& angle_rad);
	void write_R_angle_deg(const std::vector<int>& angle_deg);
	void write_R_angle_rad(float angle_rad);
	void write_R_angle_deg(int angle_deg);
	virtual ~Fin();

private:
	int8_t num_fin_servos;
	PWMServo* servos;
	int* offset_angle_deg;
	CPG::CPG cpg;

	const int rad_to_deg(float rad){
		return (int)(rad / M_PI * 180);
	}

	const float deg_to_rad(int deg){
		return (float)deg * M_PI / 180;
	}


	const std::vector<int> rad_to_deg(const std::vector<float>& rad){
		std::vector<int> deg;
		for(int i = 0; i < num_fin_servos; i++){
			deg.push_back(rad_to_deg(rad[i]));
		}
		return deg;
	}

	const std::vector<float> deg_to_rad(const std::vector<int>& deg){
		std::vector<float> rad;
		for(int i = 0; i < num_fin_servos; i++){
			rad.push_back(deg_to_rad(deg[i]));
		}
		return rad;
	}


};

#endif /* FIN_H_ */
