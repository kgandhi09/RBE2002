/*
 * SensorArray.cpp
 *
 *  Created on: Dec 7, 2019
 *      Author: ocblaufuss
 */

#include "SensorArray.h"

SensorArray::SensorArray(){
	// Set up the Analog sensors
	pinMode(ANALOG_SENSE_FRONT, ANALOG);
	pinMode(ANALOG_SENSE_TWO, ANALOG);
	pinMode(ANALOG_SENSE_THREE, ANALOG);
	pinMode(ANALOG_SENSE_FOUR, ANALOG);
	frontTriggerDistance = 10;
}

//@return true if something is sensed within trigger distance of the front
bool SensorArray::isFrontObstacle(){
	fflush(stdout);
	bool output = frontValueCM() < frontTriggerDistance;
//	Serial.println(frontValueCM());

	if (output){//get actual distance here later
//		Serial.print("Front Values of Sensor : ");
		Serial.print(frontValueCM());
		Serial.print("Trigger Value : ");
		Serial.print(frontTriggerDistance);
		return true;
	}
	return false;
}

//@return value from front sensor in Cm (only works reliably above 10cm)
float SensorArray::frontValueCM(){
	pinMode(ANALOG_SENSE_FRONT, INPUT);
	float rawADC = (analogRead(ANALOG_SENSE_FRONT)*0.02);//sensor reading * (5/256)
	if (rawADC == 0){
		return 1000;//prevent infinity values with a cap
	}
	//Serial.println(rawADC);
	//float ADCtoCM = 5461/(rawADC - 17)-2;
	float tempDist = 27.726*pow(rawADC,-1.2045)*25;
	float ADCtoCM = 1.25*(tempDist - 26) + 40;
	return ADCtoCM;
}
