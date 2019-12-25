/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "Arduino.h"
#include "RBEPID.h"

//Class constructor
RBEPID::RBEPID() {

}

//Function to set PID gain values
void RBEPID::setpid(float P, float I, float D) {
	kp = P;
	ki = I;
	kd = D;
}

/**
 * calc the PID control signel
 *
 * @param setPoint is the setpoint of the PID system
 * @param curPosition the current position of the plan
 * @return a value from -1.0 to 1.0 representing the PID control signal
 */
float RBEPID::calc(double setPoint, double curPosition) {

	// calculate error
	float err = setPoint - curPosition;
	// calculate derivative of error
	//TODO
	// calculate integral error. Running average is best but hard to implement
	//TODO
	//Serial.begin(115200);
	sum = sum + err;
	//Serial.println(sum);
	if(sum > threshold){
		sum = threshold;
	}
	else if (sum < -threshold){
		sum = -threshold;
	}
	// sum up the error value to send to the motor based off gain values.
	//TODO

	float out = err * kp + sum * ki;	// simple P controller
	//return the control signal from -1 to 1
	if (out > 1)
		out = 1;
	if (out < -1)
		out = -1;
	return out;
}

/**
 * Clear the internal representation for the integral term.
 *
 */
void RBEPID::clearIntegralBuffer() {
	//TODO implement this when implementing the integral term
	sum = 0;
}
