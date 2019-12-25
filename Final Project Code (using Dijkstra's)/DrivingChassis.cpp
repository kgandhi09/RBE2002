/*
 * DrivingChassis.cpp
 *
 *  Created on: Jan 31, 2019
 *      Author: hephaestus
 */

#include "DrivingChassis.h"

/**
 * Compute a delta in wheel angle to traverse a specific distance
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param distance a distance for this wheel to travel in MM
 * @return the wheel angle delta in degrees
 */
float DrivingChassis::distanceToWheelAngle(float distance) {
	return 0;
}

/**
 * Compute the arch length distance the wheel needs to travel through to rotate the base
 * through a given number of degrees.
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param angle is the angle the base should be rotated by
 * @return is the linear distance the wheel needs to travel given the this CHassis's wheel track
 */
float DrivingChassis::chassisRotationToWheelDistance(float angle) {
	return 0;
}

DrivingChassis::~DrivingChassis() {
	// do nothing
}

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * @param left the left motor
 * @param right the right motor
 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
 * @param imu The object that is used to access the IMU data
 *
 */
DrivingChassis::DrivingChassis(PIDMotor * left, PIDMotor * right,
		float wheelTrackMM, float wheelRadiusMM,GetIMU * imu)
: pose(){
	myleft = left;
	myright = right;
	mywheelTrackMM = wheelTrackMM;
	mywheelRadiusMM = wheelRadiusMM;
	IMU = imu;
	setTargetNode(0);
}

/**
 * Start a drive forward action
 *
 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 * @note this function is fast-return and should not block
 * @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		 allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::driveForward(float mmDistanceFromCurrent, int msDuration) {
}

/**
 * Drive straight in a direction
 *
 * @param targetHeading is the heading to point towards while running
 */
void DrivingChassis::driveStraight(int targetHeading) {
	int Kp = 18;
	int error = targetHeading - pose.theta;
	if (error > 24){
		//cap on error to prevent overshoot/overtaxing motors
		error = 24;
	}
	if (pose.theta > targetHeading){
		myleft->setVelocityDegreesPerSecond(400 - Kp*error);
		myright->setVelocityDegreesPerSecond(-400);
	}else if(pose.theta < targetHeading){
		myleft->setVelocityDegreesPerSecond(400);
		myright->setVelocityDegreesPerSecond(-400 - Kp*error);
	}else{
		myleft->setVelocityDegreesPerSecond(400);
		myright->setVelocityDegreesPerSecond(-400);
	}
}


 //Stops robot motion
void DrivingChassis::stop(){
	myleft->setVelocityDegreesPerSecond(0);
	myright->setVelocityDegreesPerSecond(0);
}

/**
 * Drive to a target Y-coordinate on the field
 *
 * @param targetY is the desired ending Y-coordinate (pose.x, targetY)
 * @param targetTheta is the desired ending pose
 *
 * @return true if robot is still driving
 */
bool DrivingChassis::driveToY(float targetY, float targetTheta) {
	if (targetY - moveTolerance > pose.y){
		if(!turnDegrees(0)){
			driveStraight(0);
		}
	}
	else if(targetY + moveTolerance < pose.y){
		if(!turnDegrees(180)){
			driveStraight(180);
		}
	}else{
		return turnDegrees(targetTheta);
	}
	return true;
}

/**
 * Drive to a target X-coordinate on the field
 *
 * @param targetX is the desired ending X-coordinate (targetX, pose.y)
 * @param targetTheta is the desired ending pose
 *
 * @return true if robot is still driving
 */
bool DrivingChassis::driveToX(float targetX, float targetTheta) {
	if (targetX - moveTolerance > pose.x){
		if(!turnDegrees(-90)){
			driveStraight(-90);
		}
	}
	else if(targetX + moveTolerance < pose.x){
		if(!turnDegrees(90)){
			driveStraight(90);
		}
	}else{
		return turnDegrees(targetTheta);
	}
	return true;
}

/**
 * Drive to a target Y-coordinate on the field from an address
 *
 * @param targetY is the desired ending Y-coordinate (pose.fieldX, targetAddressY)
 * @param targetTheta is the desired ending pose
 *
 * @return true if robot is still driving
 * @return false if robot is finished OR if robot was given a garbage value not on the field
 */
bool DrivingChassis::driveToFieldY(int targetAddressY, float targetTheta){
	if (targetAddressY >=0 && targetAddressY <= 5){
			double convertedY = targetAddressY * pose.fieldScaleY;
			return driveToY( convertedY , targetTheta);
	}else{
		Serial.print("INVALID ADDRESS provided to driveToFieldY: ");
		Serial.print(targetAddressY);
		Serial.println();
		return false;
	}
}

/**
 * Drive to a target X-coordinate on the field from an address
 *
 * @param targetX is the desired ending X-coordinate (targetAddress, pose.fieldY)
 * @param targetTheta is the desired ending pose
 *
 * @return true if robot is still driving
 * @return false if robot is finished OR if robot was given a garbage value not on the field
 */
bool DrivingChassis::driveToFieldX(int targetAddressX, float targetTheta){
	if (targetAddressX >=0 && targetAddressX <= 5){
			double convertedX = targetAddressX * pose.fieldScaleX;
			return driveToX( convertedX, targetTheta);
	}else{
		Serial.print("INVALID ADDRESS provided to driveToFieldX: ");
		Serial.print(targetAddressX);
		Serial.println();
		return false;
	}
}

/**
 * Drive to a target X and Y coordinates on the field from an address
 *
 * @param targetX is the desired ending X-coordinate
 * @param targetY is the desired ending Y-coordinate (targetAddressX, targetAddressY)
 * @param targetTheta is the desired ending pose
 *
 * @return true if robot is still driving
 */
/*
 bool DrivingChassis::driveToFieldXY(int targetAddressX, int targetAddressY, float targetTheta){
	 //sanity check coordinates
	 if (targetAddressX <=0 && targetAddressX >= 5){
		 Serial.print("INVALID ADDRESS provided to driveToFieldX: ");
		 Serial.print(targetAddressX);
		 Serial.println();
		 return false;
	 }
	 if (targetAddressY <=0 && targetAddressY >= 5){
		 Serial.print("INVALID ADDRESS provided to driveToFieldY: ");
		 Serial.print(targetAddressY);
		 Serial.println();
		 return false;
	 }

	 //check on a large square scale before going more specific
	 if (targetAddressX != pose.fieldX){
		 double convertedX = targetAddressX * pose.fieldSquare;
		 convertedX += pose.fieldSquare/2;
		 if(driveToX(convertedX, targetTheta)){
			 return true;
		 }
	 }else if (targetAddressY != pose.fieldY){
		 double convertedY = targetAddressY * pose.fieldSquare;
		convertedY += pose.fieldSquare/2;
		if (driveToY(convertedY , targetTheta)){
			return true;
		}
	 }
	 return false;
 }

 */

/**
 * This action rotates the robot around the center line made up by the contact points of the left and right wheels.
 * Positive angles should rotate to the left around the Z axis.
 *
 * @param targetHeading the heading to adjust to
 * @return whether move is still in progress or not ("false" when done)
 *
 *  @note this function is fast-return and should not block
 *  @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		  allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
bool DrivingChassis::turnDegrees(float targetHeading) {
	int error = pose.theta - targetHeading;
	if (error > 50){
		//cap on error to prevent overshoot/overtaxing motors
		error = 50;
	}
	//Proportional control
	int Kp = 5;
	if (pose.theta > targetHeading+turnTolerance){
		//negative turn
		myleft->setVelocityDegreesPerSecond(Kp*error);
		myright->setVelocityDegreesPerSecond(Kp*error);
	}else if(pose.theta < targetHeading-turnTolerance){
		//positive turn
		myleft->setVelocityDegreesPerSecond(Kp*error);
		myright->setVelocityDegreesPerSecond(Kp*error);
	}else{
		myleft->setVelocityDegreesPerSecond(0);
		myright->setVelocityDegreesPerSecond(0);
		return false;
	}
	return true;
}

/**
 * Check to see if the chassis is performing an action
 *
 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
 *
 * @note this function is fast-return and should not block
 */
bool DrivingChassis::isChassisDoneDriving() {
	return false;
}

/**
 * This function updates the X, Y, and theta coordinates as the robot moves around the field.
 * This version does not currently work but is left in case we want to reintegrate these methods
 */
void DrivingChassis::updatePoseICC(){
	double leftEnc = myleft->getAngleDegrees();    //getAngleDegrees()
	double rightEnc = myright->getAngleDegrees();	//getAngleDegrees()
	long imuData = IMU->getEULER_azimuth();

	double deltaLeftEnc = leftEnc - lastLeftEnc;
	double deltaRightEnc = rightEnc - lastRightEnc;
	long deltaIMU = imuData - lastIMUData;

	float uR = - (deltaRightEnc) / (deltaTimeS);
	float uL = (deltaLeftEnc) / (deltaTimeS);
	float u = (uL + uR)/2;

	if((uR == uL) || ((uR == 0) && (uL == 0))){
		R = 0;
	}
	else{
		R = abs(11.5 * ((uR+uL)/(uR-uL)));
	}
	float ICCx = pose.x - (R*sin(deltaIMU));
	float ICCy = pose.y  + (R*cos(deltaIMU));

	//every time omega (the velocity) is called, it is multiplied by deltaTime, so...
	///using deltaIMU in place of omega*deltaTime for simplicity
	float omega = deltaIMU / (deltaTimeS);


	pose.x = (pose.x - ICCx)*cos(deltaIMU) + (ICCy - pose.y)*sin(deltaIMU) + ICCx;
	pose.y = (pose.x - ICCx)*sin(deltaIMU) + (pose.y - ICCy)*cos(deltaIMU) + ICCy;

	lastLeftEnc = leftEnc;
	lastRightEnc = rightEnc;
	lastIMUData = imuData;
	prevTheta = pose.theta;

}

/**
 * This function updates the X, Y, and theta coordinates as the robot moves around the field.
 * It uses the second order equations provided to do so.
 */
void DrivingChassis::updatePose(){
	double leftEnc = myleft->getAngleDegrees();    //getAngleDegrees()
	double rightEnc = myright->getAngleDegrees();	//getAngleDegrees()
	long imuData = IMU->getEULER_azimuth();

	double deltaLeftEnc = leftEnc - lastLeftEnc;
	double deltaRightEnc = rightEnc - lastRightEnc;
	long deltaIMU = imuData - lastIMUData;

	float uR = (-(deltaRightEnc) / (deltaTimeS)) * ((2*3.1415*mywheelRadiusMM)/3600);
	float uL = ((deltaLeftEnc) / (deltaTimeS)) * ((2*3.1415*mywheelRadiusMM)/3600);
	float u = (uL + uR)/2;

	//float omega = ((uR - uL)*10)/(mywheelTrackMM);

	pose.theta = pose.theta - (deltaIMU);

	float avgTheta = (prevTheta + ((deltaIMU)/2))*(3.1415/180);
	pose.x = pose.x - (deltaTimeS*u*sin(avgTheta));
	pose.y = pose.y + (deltaTimeS*u*cos(avgTheta));
	pose.fieldX = pose.x/pose.fieldScaleX;
	pose.fieldY = pose.y/pose.fieldScaleY;
	pose.currNode = convertToNode(pose.fieldX, pose.fieldY);


//	Serial.print(" Coordinates: ");
//	Serial.print(pose.fieldX);
//	Serial.print(" , ");
//	Serial.print(pose.fieldY);
//	Serial.print(" , ");
//	Serial.println(pose.theta);


	lastLeftEnc = leftEnc;
	lastRightEnc = rightEnc;
	lastIMUData = imuData;
	prevTheta = pose.theta;

}

//return which node is being faced
int DrivingChassis::facingNode(){
	int result = -1;
	int temp = 15;
	if(-90 - temp <= pose.theta && pose.theta <= -90 + temp){
		if(pose.currNode%6 == 5){
			result = -1;
		}else{
			result = pose.currNode + 1;
		}

	}
	else if(-temp <= pose.theta && pose.theta <= temp){
		if(pose.currNode <= 35 && pose.currNode >= 30){
			result = -1;
		}else{
			result = pose.currNode + 6;

		}
	}
	else if (90 - temp <=pose.theta && pose.theta <= 90 + temp){
		if(pose.currNode%6 == 0){
			result = -1;
		}else{
			result = pose.currNode - 1;
		}

	}
	else if (180 - temp <= pose.theta && pose.theta <= 180 + temp){
		if(pose.currNode <= 5 && pose.currNode >= 0){
			result = -1;
		}else{
			result = pose.currNode - 6;
		}

	}

	return result;
}

int DrivingChassis::convertToNode(int x, int y){
	return (y*6)+x;
}

/*
 * setTargetNode()
 *
 * @param int node label to drive to next
 */
void DrivingChassis::setTargetNode(int targetNode){
//	Serial.print("Start Node : ");
//	Serial.println(pose.currNode);
//	Serial.println(d->nodeCoordinateX(0));
	path = d->findPath(pose.currNode, targetNode);
	progress = path.size() - 1;
}

//returns next node in the path
int DrivingChassis::nextNode(){
	return path[progress];
}

//sets the next node in the path to being blocked
void DrivingChassis::blockNode(int node){
	d->blockNode(node);
}

/*
 * move()
 * moves through list of djsktra-provided nodes to the predefined target
 */
bool DrivingChassis::move(){
	//Serial.println(d->nodeCoordinateX(path[progress]));
	//Serial.println(d->nodeCoordinateY(path[progress]));
	if (progress < 0){
		stop(); //if at end of list: you have arrived
		return false;
	}else if(
			!driveToFieldX(d->nodeCoordinateX(path[progress]), pose.theta) && !driveToFieldY(d->nodeCoordinateY(path[progress]), pose.theta)
			){
		progress--;

	}
	return true;
}


/**
 * loop()
 *
 * a fast loop function that will update states of the motors based on the information from the
 * imu.
 */
bool DrivingChassis::loop(){
	if(nextTimeDC <= millis()){
		nextTimeDC = nextTimeDC + deltaTimeS*100;
		updatePose();
	}
	return false;
}


