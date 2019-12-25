/*
 * SensorArray.h
 *
 *  Created on: Dec 7, 2019
 *      Author: ocblaufuss
 */

#ifndef SENSORARRAY_H_
#define SENSORARRAY_H_

#include <stdlib.h>
#include <string.h>
#include "HardwareSerial.h"

// Sensor Pins
#define ANALOG_SENSE_FRONT		36
#define ANALOG_SENSE_TWO		39
#define ANALOG_SENSE_THREE		34
#define ANALOG_SENSE_FOUR		35


class SensorArray{

private:
	int frontTriggerDistance;//in Cm

public:
	SensorArray();

	//@return true if something is sensed within trigger distance of the front
	bool isFrontObstacle();

	//@return value from front sensor converted to CM
	float frontValueCM();

};



#endif /* SENSORARRAY_H_ */
