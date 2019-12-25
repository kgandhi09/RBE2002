/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */

#ifndef PID_H_
#define PID_H_

class RBEPID {

public:
	RBEPID();
	float kp = 0.00015;
	float ki = 0;
	float kd = 0;
	float last_error = 0;
	float sum_error = 0;
	int sampleRateMs = 5;
	float sum = 0;
	float threshold = 100000;
	/**
	 * setpid set PID constants
	 */
	void setpid(float P, float I, float D);
	/**
	 * calc the PID control signal
	 *
	 * @param setPoint is the setpoint of the PID system
	 * @param curPosition the current position of the plan
	 * @return a value from -1.0 to 1.0 representing the PID control signel
	 */
	float calc(double setPoint, double curPosition);
	/**
	 * Clear the internal representation for the integral term.
	 *
	 */
	void clearIntegralBuffer();
private:

};

#endif
