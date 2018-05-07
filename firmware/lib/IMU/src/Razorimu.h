/*
 * Razorimu.h
 *
 *  Created on: 26 Mar 2018
 *      Author: Chen Bainian
 */

#ifndef RAZORIMU_H_
#define RAZORIMU_H_

class Razor_imu {
public:
	Razor_imu();
	void init();
	bool update();

	float yaw, roll, pitch, ax, ay, az, gx, gy, gz;

	virtual ~Razor_imu();

};

#endif /* RAZORIMU_H_ */
