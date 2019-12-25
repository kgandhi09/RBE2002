/*
 * DrivingChassis.h
 *
 *  Created on: Jan 12, 2019
 *      Author: hephaestus
 */

#ifndef DRIVINGCHASSIS_H_
#define DRIVINGCHASSIS_H_
#include "src/pid/PIDMotor.h"
#include "src/commands/GetIMU.h"
#include "dijkstra.h"
#include "config.h"
#include "Pose.h"
#include <stdlib.h>
#include <string.h>
/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * The 0,0,0 center of the robot is on the ground, half way between the left and right wheel contact points.
 *
 * The +X axis is the positive direction of travel
 *
 * The +Y axis goes from the center of the robot to the left wheel
 *
 * The +Z axis goes from the center up through the robot towards the ceiling.
 *
 * This object should manage the setting of motor setpoints to enable driving
 */
class DrivingChassis {
private:
	PIDMotor * myleft;
	PIDMotor * myright;
	GetIMU * IMU;
	float mywheelTrackMM;
	float mywheelRadiusMM;
	unsigned long int nextTimeDC = 0;
	float deltaTimeS = 0.02;
	double lastLeftEnc = 0;
	double lastRightEnc = 0;
	double lastIMUData = 0;
	float R = 0;
	float prevPoseX = 0;
	float prevPoseY = 0;
	float prevTheta = 0;
	//tolerance allows small errors without oscillation
	float turnTolerance = 3;
	float moveTolerance = 1.5;
	int count = 0;
	dijkstra* d = new dijkstra();
	vector<int> path = {};
	int progress;
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
	float distanceToWheelAngle(float distance);
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
	float chassisRotationToWheelDistance(float angle);


	// Weighted Adjacency matrix that represents the field layout in gaph data structure


public:
	Pose pose; // Pose Object
	virtual ~DrivingChassis();

	/**
	 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
	 *
	 * @param left the left motor
	 * @param right the right motor
	 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
	 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
	 * @param imu The object that is used to access the IMU data
	 */
	DrivingChassis(PIDMotor * left, PIDMotor * right, float wheelTrackMM,
			float wheelRadiusMM,GetIMU * imu);

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
	void driveForward(float mmDistanceFromCurrent, int msDuration);

	 //Stops robot motion
	void stop();

	/**
	 * Drive straight in a direction
	 *
	 * @param targetHeading is the heading to point towards while running
	 */
	void driveStraight(int targetHeading);

	/**
	 * Drive to a target X-coordinate on the field
	 *
	 * @param targetX is the desired ending X-coordinate (targetX, pose.y)
	 * @param targetTheta is the desired ending pose
	 *
	 * @return true if robot is still driving
	 */
	bool driveToX(float targetX, float targetTheta);

	/**
	 * Drive to a target Y-coordinate on the field
	 *
	 * @param targetY is the desired ending Y-coordinate (pose.x, targetY)
	 * @param targetTheta is the desired ending pose
	 *
	 * @return true if robot is still driving
	 */
	bool driveToY(float targetY, float targetTheta);

	/**
			 * Drive to a target X-coordinate on the field from an address
			 *
			 * @param targetX is the desired ending X-coordinate (targetAddress, pose.fieldY)
			 * @param targetTheta is the desired ending pose
			 *
			 * @return true if robot is still driving
			 */
			bool driveToFieldX(int targetAddressX, float targetTheta);

			/**
			 * Drive to a target Y-coordinate on the field from an address
			 *
			 * @param targetY is the desired ending Y-coordinate (pose.fieldX, targetAddressY)
			 * @param targetTheta is the desired ending pose
			 *
			 * @return true if robot is still driving
			 */
			bool driveToFieldY(int targetAddressY, float targetTheta);

			/**
			 * Drive to a target X and Y coordinates on the field from an address
			 *
			 * @param targetX is the desired ending X-coordinate
			 * @param targetY is the desired ending Y-coordinate (targetAddressX, targetAddressY)
			 * @param targetTheta is the desired ending pose
			 *
			 * @return true if robot is still driving
			 */
			 bool driveToFieldXY(int targetAddressX, int targetAddressY, float targetTheta);



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
	bool turnDegrees(float degreesToRotateBase);

	/**
	 * Check to see if the chassis is performing an action
	 *
	 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
	 *
	 * @note this function is fast-return and should not block
	 */
	bool isChassisDoneDriving();
	/**
	 * loop()
	 *
	 * a fast loop function that will update states of the motors based on the information from the
	 * imu.
	 *
	 * @note this function is fast-return and should not block
	 */
	bool loop();

	/**
	 * This function updates the X, Y, and theta coordinates as the robot moves around the field.
	 * This version does not currently work but is left in case we want to reintegrate these methods
	 */
	void updatePoseICC();

	/**
	 * This function updates the X, Y, and theta coordinates as the robot moves around the field.
	 * It uses the second order equations provided to do so.
	 */
	void updatePose(); // Updates the pose object from encoders


	int facingNode();

	/*
	 * setTargetNode()
	 *
	 * @param int node label to drive to next
	 */
	void setTargetNode(int targetNode);
	//sets the next node in the path to being blocked
	void blockNode(int node);

	//returns next node in the path
	int nextNode();

	/*
	 * move()
	 * moves through list of djsktra-provided nodes to the predefined target
	 */
	bool move();

	int convertToNode(int x, int y);

};

#endif /* DRIVINGCHASSIS_H_ */
