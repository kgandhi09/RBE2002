/*
 * Pose.h
 *
 *  Created on: Nov 13, 2019
 *      Author: kgandhi
 */

#ifndef POSE_H_
#define POSE_H_

class Pose{
private:
public:
	Pose();
	float x = 0;
	float y = 0;
	float theta = 0;

	//field position on grid, with starting position being 0,0
	int fieldX = 0;
	int fieldY = 0;
	int currNode = 0;

	double fieldScaleX = 45;//size of a grid x in cm
	double fieldScaleY = 50;//size of a grid y in cm

	//map that keeps track of which spots are open
	enum spot{
		EMPTY = 0,//uninteresting
		OBUIL = 1,//building?
		BUILD = 2,//confirmed building
		OSCAN = 3,//scan location
		SCAND = 4 //scan complete
		};

	int fieldMap[6][6] = {
				{OSCAN, OBUIL, OSCAN, OBUIL, OSCAN, OBUIL},//bottom row (y=0)
				{EMPTY, OSCAN, EMPTY, OSCAN, EMPTY, OSCAN},
				{OSCAN, OBUIL, OSCAN, OBUIL, OSCAN, OBUIL},
				{EMPTY, OSCAN, EMPTY, OSCAN, EMPTY, OSCAN},
				{OSCAN, OBUIL, OSCAN, OBUIL, OSCAN, OBUIL},
				{EMPTY, OSCAN, EMPTY, OSCAN, EMPTY, OSCAN},//top row (y=5)
				};
	};




	/**
	 * Report back address on the field
	 *
	 * @return array of chars which forms the string of the address on the field according to the map
	 */
	//char* fieldAddress();




#endif /* POSE_H_ */
