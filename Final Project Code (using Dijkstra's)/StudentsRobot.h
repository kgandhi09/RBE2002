/*
 * StudentsRobot.h
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#ifndef STUDENTSROBOT_H_
#define STUDENTSROBOT_H_
#include "config.h"
#include <Arduino.h>
#include "src/pid/ServoEncoderPIDMotor.h"
#include "src/pid/HBridgeEncoderPIDMotor.h"
#include "src/pid/ServoAnalogPIDMotor.h"
#include <ESP32Servo.h>
#include <string.h>

#include "DrivingChassis.h"
#include "SensorArray.h"
#include "src/commands/IRCamSimplePacketComsServer.h"
#include "src/commands/GetIMU.h"

/**
 * @enum RobotStateMachine
 * These are sample values for a sample state machine.
 * Feel free to add ot remove values from here
 */
enum RobotStateMachine {
	StartupRobot = 0, StartRunning = 1, Running = 2, Halting = 3, Halt = 4, WAIT_FOR_TIME=5,
	Pick_New_Location = 6, Wait_For_Drive = 7, Handle_Obstacle = 8, Scan_Left = 9, Scan_Right = 10,
	ROBIN = 12, Deploy_Device = 13, Return_Home = 14
};
/**
 * @enum ComStackStatusState
 * These are values for the communications stack
 * Don't add any more or change these. This is how you tell the GUI
 * what state your robot is in.
 */
enum ComStackStatusState {
	Ready_for_new_task = 0,
	Heading_to_pickup = 1,
	Waiting_for_approval_to_pickup = 2,
	Picking_up = 3,
	Heading_to_Dropoff = 4,
	Waiting_for_approval_to_dropoff = 5,
	Dropping_off = 6,
	Heading_to_safe_zone = 7,
	Fault_failed_pickup = 8,
	Fault_failed_dropoff = 9,
	Fault_excessive_load = 10,
	Fault_obstructed_path = 11,
	Fault_E_Stop_pressed = 12
};
/**
 * @class StudentsRobot
 */
class StudentsRobot {
private:
	PIDMotor * motor1;
	PIDMotor * motor2;
	PIDMotor * motor3;
	Servo * servo;
	float lsensorVal=0;
	float rsensorVal=0;
	long nextTime =0;
	int timeInterval = 1000;//interval between loops
  long startTime =0;
	RobotStateMachine nextStatus = StartupRobot;
	IRCamSimplePacketComsServer * IRCamera;
	GetIMU * IMU;
	DrivingChassis drivingChassis;
	SensorArray sensors;
	float targetDist = 1400; // (a) - 1147

	//direction going through the field
	int backForth = 1;
	int newX;
	int newY;

	//keep track of where Robin was seen
	int finalNode;
	int finalTheta;

public:
	/**
	 * Constructor for StudentsRobot
	 *
	 * attach the 4 actuators
	 *
	 * these are the 4 actuators you need to use for this lab
	 * all 4 must be attached at this time
	 * DO NOT reuse pins or fail to attach any of the objects
	 *
	 */
	StudentsRobot(PIDMotor * motor1,
			PIDMotor * motor2, PIDMotor * motor3,
			Servo * servo,IRCamSimplePacketComsServer * IRCam,GetIMU * imu);
	/**
	 * Command status
	 *
	 * this is sent upstream to the Java GUI to notify it of current state
	 */
	ComStackStatusState myCommandsStatus = Ready_for_new_task;
	/**
	 * This is internal data representing the runtime status of the robot for use in its state machine
	 */
	RobotStateMachine status = StartupRobot;


	/**
	 * pidLoop This functoion is called to let the StudentsRobot controll the running of the PID loop functions
	 *
	 * The loop function on all motors needs to be run when this function is called and return fast
	 */
	void pidLoop();
	/**
	 * updateStateMachine use the stub state machine as a starting point.
	 *
	 * the students state machine can be updated with this function
	 */
	void updateStateMachine();
};

#endif /* STUDENTSROBOT_H_ */
