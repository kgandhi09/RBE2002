/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

//  //
#include "StudentsRobot.h"

StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * motor3, Servo * servo, IRCamSimplePacketComsServer * IRCam,
		GetIMU * imu)
: drivingChassis(motor1, motor2, 230, 30, imu), sensors()


{
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");
	this->servo = servo;
	this->motor1 = motor1;
	this->motor2 = motor2;
	this->motor3 = motor3;
	IRCamera = IRCam;
	IMU = imu;

#if defined(USE_IMU)
	IMU->setXPosition(200);
	IMU->setYPosition(0);
	IMU->setZPosition(0);
#endif
	// Set the PID Clock gating rate. The PID must be 10 times slower than the motors update rate
	motor1->myPID.sampleRateMs = 5; //
	motor2->myPID.sampleRateMs = 5; //
	motor3->myPID.sampleRateMs = 5;  // 10khz H-Bridge, 0.1ms update, 1 ms PID

	// Set default P.I.D gains
	motor1->myPID.setpid(0.004, 0.0001, 0);
	motor2->myPID.setpid(0.004, 0.0001, 0);
	motor3->myPID.setpid(0.004, 0.0001, 0);

	motor1->velocityPID.setpid(0.004, 0.0001, 0);
	motor2->velocityPID.setpid(0.004, 0.0001, 0);
	motor3->velocityPID.setpid(0.1, 0, 0);
	// compute ratios and bounding
	double motorToWheel = 3;
	motor1->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
			50.0 * // Motor Gear box ratio
			motorToWheel * // motor to wheel stage ratio
			(1.0 / 360.0) * // degrees per revolution
			2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
	);
	motor2->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
			50.0 * // Motor Gear box ratio
			motorToWheel * // motor to wheel stage ratio
			(1.0 / 360.0) * // degrees per revolution
			2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150	// the speed in degrees per second that the motor spins when the hardware output is at creep forwards
	);
	motor3->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
			50.0 * // Motor Gear box ratio
			1.0 * // motor to arm stage ratio
			(1.0 / 360.0) * // degrees per revolution
			2, // Number of edges that are used to increment the value
			1400, // measured max degrees per second
			50 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
	);
	// H-Bridge enable pin
	pinMode(H_BRIDGE_ENABLE, OUTPUT);
	// Stepper pins
	pinMode(STEPPER_DIRECTION, OUTPUT);
	pinMode(STEPPER_STEP, OUTPUT);
	// User button
	pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
	//Test IO
	pinMode(WII_CONTROLLER_DETECT, OUTPUT);

	//where to first? where to next?

}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */
void StudentsRobot::updateStateMachine() {
	digitalWrite(WII_CONTROLLER_DETECT, 1);
	long now = millis();
	drivingChassis.loop();

	switch (status) {
	case StartupRobot:
		//Do this once at startup
		status = StartRunning;
		Serial.println("StudentsRobot::updateStateMachine StartupRobot here ");
		break;
	case StartRunning:
		Serial.println("Start Running Final with IR");

		digitalWrite(H_BRIDGE_ENABLE, 1);

		//Pick location to drive to;
		newX = drivingChassis.pose.fieldX;
		newY = drivingChassis.pose.fieldY;
		status = Pick_New_Location;

		nextTime = startTime + timeInterval; // the next timer loop should be (timeInterval)ms after the motors stop
		break;
	case Running:
		// Set up a non-blocking 1000 ms delay
		status = WAIT_FOR_TIME;
		nextTime = nextTime + timeInterval; // ensure no timer drift by incremeting the target
		// After (timeInterval) ms, come back to this state

		nextStatus = Running;
		if (!digitalRead(BOOT_FLAG_PIN)) {
			status = WAIT_FOR_TIME;
			Serial.println(
					" Running State Machine " + String((now - startTime)));
#if defined(USE_IMU)
			IMU->print();
#endif
#if defined(USE_IR_CAM)
			IRCamera->print();
#endif

		}
		break;
	case WAIT_FOR_TIME:
		// Check to see if enough time has elapsed
		if (nextTime <= millis()) {
			// if the time is up, move on to the next state
			status = nextStatus;
		}
		break;
	case Pick_New_Location:
		Serial.println(drivingChassis.pose.fieldMap[newY][newX+backForth]);
		if(drivingChassis.pose.fieldMap[newY][newX+backForth] == 3){ //if a spot that needs to be scanned is next (OSCAN = 3)
			Serial.print(newX+backForth);
			Serial.print(", ");
			Serial.println(newY);
			drivingChassis.setTargetNode(drivingChassis.convertToNode(newX+backForth, newY));
			status = Wait_For_Drive;
		}else{
			newX = newX + backForth;
			if (newX >= 5 || newX <= 0){ //if X overflows, go to next Y
				Serial.println("Going back the other way");
				backForth = backForth*-1; //flip sign (now going the other way)
				newY++;
				Serial.println("line overflow");
				if(newY >= 6 || newY < 0){//if y overflows reset to bottom of field
					newY = 0;
					drivingChassis.setTargetNode(0);//return to home
					status = Wait_For_Drive;
				}
			}
			status = Pick_New_Location; //and we'll try again next time around.
		}
		break;
	case Wait_For_Drive:
		if (!drivingChassis.move()){
			finalTheta = drivingChassis.pose.theta;
			status = Scan_Left;
		}
		if(sensors.isFrontObstacle()){ //if obstacle is in front of robot
			status = Handle_Obstacle;
		}
		break;
	case Handle_Obstacle:
		Serial.println("Obstacle encountered!");
		drivingChassis.blockNode(drivingChassis.nextNode());//block the node we were about to drive to (should we move this to a drivingChassis function?)
		//		if (drivingChassis.nextNode()==1 || drivingChassis.nextNode()==3 || drivingChassis.nextNode()==5||
		//				drivingChassis.nextNode()== 13 || drivingChassis.nextNode()== 15 || drivingChassis.nextNode()== 17||
		//				drivingChassis.nextNode()== 25 || drivingChassis.nextNode()== 27 || drivingChassis.nextNode()== 29){//if is at a building address, we investigate.
		//			nextDestination = destination;//we remember where we were going, but save it for later.
		//			//status = Building_Scan;
		//			drivingChassis.setTargetNode(destination);
		//			status = Wait_For_Drive;
		//		}else{//otherwise we just go right back to driving after our recalculation.
		//			drivingChassis.setTargetNode(destination);
		//			status = Wait_For_Drive;
		//		}
		newX = drivingChassis.pose.fieldX;
		newY = drivingChassis.pose.fieldY;
		status = Pick_New_Location;
		break;
	case Scan_Left:
		//pan sensors left
		Serial.println("Scanning Left");
		if (!drivingChassis.turnDegrees(finalTheta - 90)){//when the turret is pointing left and has taken its measurement
			finalTheta = drivingChassis.pose.theta;
			status = Scan_Right; //scan the other way
		}
		break;
	case Scan_Right:
		//pan sensors right
		Serial.println("Scanning Right");
		if (!drivingChassis.turnDegrees(finalTheta + 180)){//when turret is pointing right and has taken its measurement and everything is updated
			finalTheta = drivingChassis.pose.theta;
			newX = drivingChassis.pose.fieldX;
			newY = drivingChassis.pose.fieldY;
			status = Pick_New_Location; //go to next location
		}
		break;
	case ROBIN:
		finalNode = drivingChassis.pose.currNode;
		finalTheta = drivingChassis.pose.theta;
		status = Deploy_Device;
		break;
	case Deploy_Device:
		if (true){ //after device is successfully deployed
			drivingChassis.setTargetNode(0);
			status = Return_Home;
		}
	break;
	case Return_Home:
		if (!drivingChassis.move()){
			status = Halting;
		}
	break;
	case Halting:
		// save state and enter safe mode
		Serial.println("Halting State machine");
		digitalWrite(H_BRIDGE_ENABLE, 0);
		motor3->stop();
		motor2->stop();
		motor1->stop();

		status = Halt;
		break;
	case Halt:
		// in safe mode
		break;

	}
	digitalWrite(WII_CONTROLLER_DETECT, 0);
}

/**
 * This is run fast and should return fast
 *
 * You call the PIDMotor's loop function. This will update the whole motor control system
 * This will read from the concoder and write to the motors and handle the hardware interface.
 */
void StudentsRobot::pidLoop() {
	motor1->loop();
	motor2->loop();
	motor3->loop();
}

