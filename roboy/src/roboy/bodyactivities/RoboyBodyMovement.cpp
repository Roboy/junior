/*Copyright (c) 2014, University of Zurich, Department of Informatics, Artificial Intelligence Laboratory
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors 
   may be used to endorse or promote products derived from this software without 
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/


#include "RoboyBodyMovement.h"
// BodyMovement should be applicable for a any Folder containing Motor Position Commands for the whole body (except head, under Arm and Hand?)



namespace Roboy
{

RoboyBodyMovement::RoboyBodyMovement(Robot *canBus_given) : RoboyBodyActivity()
{
	std::cout << "RoboyBodyMovement Constructor with passing the canBus" << std::endl;
	canBus = canBus_given;

  canBus->setMaxVelocityAndAcceleration(LEFT_KNEE_MOTOR_ID, DEFAULT_LEG_SWING_VELOCITY, DEFAULT_LEG_SWING_ACCELERATION);
  canBus->setMaxVelocityAndAcceleration(RIGHT_KNEE_MOTOR_ID, DEFAULT_LEG_SWING_VELOCITY, DEFAULT_LEG_SWING_ACCELERATION);

  legSwingAcceleration = DEFAULT_LEG_SWING_ACCELERATION;
  legSwingMagnitude = NO_LEG_SWING;
  legSwingVelocity = DEFAULT_LEG_SWING_VELOCITY;
  legSwingTimePeriodUS = NO_LEG_SWING;
  legSwingPhaseShift = DEFAULT_LEG_PHASE_SHIFT;
  legSwingSets = NO_LEG_SWING;

  isLegSwinging = false;

  legSwingThread = new boost::thread( boost::bind(&RoboyBodyMovement::legSwingExecuteThread, this) );
}


RoboyBodyMovement::~RoboyBodyMovement() 
{

}

void
RoboyBodyMovement::setFolderToPlay(std::string folderName_passed)
{
	folderName = folderName_passed;
	std::cout << "RoboyBodyMovement Initialization, this->filetoplay (actually a folder) is set to " << folderName_passed << std::endl;

}

void
RoboyBodyMovement::init(std::string filetoplay)
{
	
}

void 
RoboyBodyMovement::execute()
{          
	
	while (1) {

		while (!isBodyActivityActive){
			usleep(10000);
		}

  // FIXME: Put the initialization stuff which has to be done only once in initializeRobot(). This function is called in the constructor.
 
		std::cout << "RoboyBodyMovement Executing with the file (actually a folder) " << folderName << std::endl;
	
		playTrajectory();

		isBodyActivityFinished = true;
		isBodyActivityActive = false;
		std::cout << "RoboyBodyMovement Executing the file (actually a folder) " << folderName << " finished" << std::endl;

	} // end while (1)
}

void
RoboyBodyMovement::terminate()
{
	std::cout << "RoboyBodyMovement Terminating" << std::endl;
  legSwingThread->interrupt();
  canBus->setControlValue(LEFT_KNEE_MOTOR_ID, LEFT_KNEE_REST_POSITION, POSITION_CONTROL);
  canBus->setControlValue(RIGHT_KNEE_MOTOR_ID, RIGHT_KNEE_REST_POSITION, POSITION_CONTROL);
	// terminating a bodyMovement: Simply wait until it's finished!
  while(!isFinished())
		{
		usleep(10000);
		}
	// setting back the isFinished and isActive Values happens in the stop() function.
}

void
RoboyBodyMovement::playTrajectory() {
	std::string filePaths[TOTAL_MOTORS_IN_ROBOT];

	int i, j;
	std::stringstream ss, folderPathStream;
	vector<trajectoryFileInfo> tFileInfo;
	int *motorID;
	int *maxVelocity;
		
	PauseTimer waitTimer(100000000);
	dataArrayWrapper testFileExists;
	trajectoryFileInfo tmp;
	
	folderPathStream.str("");	
	folderPathStream << canBus->pathToTrajectories.str().c_str() << folderName << "/";

	std::cerr << "Relative Folder Path: " << folderPathStream.str() << std::endl;
	
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		ss.str("");
		ss << folderPathStream.str().c_str();
		if(i < 10) {
			ss << "Motor0" << i << "Out.txt";
		} else {
			ss << "Motor" << i << "Out.txt";
		}
		testFileExists = loadDataFileArray(ss.str().c_str(), 3);
		if(testFileExists.row > 0) {
			tmp.motorID = i;
			tmp.numWayPoints = testFileExists.row;
			tmp.totalTimeMS = 0;
			for(j = 0; j < tmp.numWayPoints; j++) {
				tmp.totalTimeMS += testFileExists.dataArray[3*j];
			}
			filePaths[tFileInfo.size()] = ss.str();
			tFileInfo.push_back(tmp);
			free(testFileExists.dataArray);
		}
	}

	if(tFileInfo.size() <= 0) {
		TRACE_RED_BOLD("No Files Found in Relative Path: %s", folderPathStream.str().c_str()); TRACE("\n");
		return;
	}
	
	motorID = (int *) malloc(tFileInfo.size()*sizeof(int));
	maxVelocity = (int *) malloc(tFileInfo.size()*sizeof(int));
		
	for(i = 0; i < tFileInfo.size(); i++) {
		motorID[i] = tFileInfo[i].motorID;
		if((motorID[i] >= 2 && motorID[i] <= 4) || motorID[i] == 23 || motorID[i] == 24 || (motorID[i] >= 26 && motorID[i] <= 30) || (motorID[i] >= 45 && motorID[i] <= 48)) {
			maxVelocity[i] = 11000;
		} else {
			if(motorID[i] == 1 || motorID[i] == 25) {
				maxVelocity[i] = 5000;
			} else {
				maxVelocity[i] = 7000;
			}
		}	
	}
	
	for(i = 0; i < tFileInfo.size(); i++) {
		canBus->clearFault(motorID[i]);
		canBus->resetInterpolatedBuffer(motorID[i]);

    //May not be needed, but has to be here for failsafe. May reduce delay if removed. 
    //Can cause problems if the values are not set correctly or if the motors are not in the proper CONTROL state (not related to OPERATION state).
		canBus->initializeMotorForControl(motorID[i]);
		// Increase the max velocity by 1000 RPM so that there will be no errors when computing the 
		// interconnecting trajectory
		canBus->setMaxVelocityAndAcceleration(motorID[i], maxVelocity[i] + 1000, 320000);
	}	
	
	canBus->startNode();
	for(i = 0; i < tFileInfo.size(); i++) {
		canBus->setMotorControlModePDO(motorID[i], OPERATION_MODE_INTERPOLATED_POSITION);
	
		TRACE_PURP_BOLD("To Load Motor [%d] (%d RPM) Num waypoints: [%d] Total time (ms): [%d]", motorID[i], maxVelocity[i], tFileInfo[i].numWayPoints, tFileInfo[i].totalTimeMS);TRACE("\n");		
	}	

	canBus->readTrajectoryMultipleInterpolate(motorID, filePaths, maxVelocity, tFileInfo.size(), true);

	while(canBus->isIPMBufferEmpty(motorID, tFileInfo.size()) == false) {
		waitTimer.wait();
	}
	
	
	free(motorID);
	free(maxVelocity);
}

bool
RoboyBodyMovement::moveForeArm(bool isLeftArm, int value, int maxVelocity, int maxAcceleration, bool blocking) {
  bool usePDO;
	int targetMotor, targetValue;


	if(isLeftArm){
		targetMotor = LEFT_FOREARM_MOTOR_ID;
	  targetValue = bound(value, LEFT_OUTWARD_FOREARM_VALUE, LEFT_INWARD_FOREARM_VALUE);
	} else {
		targetMotor = RIGHT_FOREARM_MOTOR_ID;
  	targetValue = bound(value, RIGHT_INWARD_FOREARM_VALUE, RIGHT_OUTWARD_FOREARM_VALUE);
	}

  if(canBus->setMaxVelocityAndAcceleration(targetMotor, maxVelocity, maxAcceleration) != SEND_SUCCESSFUL) return false;
  if(canBus->setMotorControl(targetMotor, targetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return false;

	if(blocking) {
    if(canBus->allMotors[targetMotor].getState() == OPERATIONAL) {
      usePDO = true;
    } else {
      usePDO = false;
    }
    canBus->isMotorReachedBlocking(targetMotor, targetValue, MOTOR_POSITION_ERROR_ALLOWANCE, usePDO, POSITION_CHECKING_TIME_PERIOD_NS,  INFINITE_CYCLES);
  }
  return true;
}


bool
RoboyBodyMovement::moveHand(bool isLeftHand, bool isClose, bool blocking)
{

	if(isClose){
		return moveHand(isLeftHand, CLOSE_HAND_VALUE, blocking);
	} else {
		return moveHand(isLeftHand, OPEN_HAND_VALUE, blocking);
	}
}

bool
RoboyBodyMovement::moveHand(bool isLeftHand, int value, bool blocking)
{
  bool usePDO;
	int targetMotor, targetValue;

	targetValue = bound(value, CLOSE_HAND_VALUE, OPEN_HAND_VALUE);

	if(isLeftHand){
		targetMotor = LEFT_HAND_MOTOR_ID;
	} else {
		targetMotor = RIGHT_HAND_MOTOR_ID;
	}
  if(canBus->setMotorControl(targetMotor, targetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return false;

	if(blocking) {
    if(canBus->allMotors[targetMotor].getState() == OPERATIONAL) {
      usePDO = true;
    } else {
      usePDO = false;
    }
    canBus->isMotorReachedBlocking(targetMotor, targetValue, MOTOR_POSITION_ERROR_ALLOWANCE, usePDO, POSITION_CHECKING_TIME_PERIOD_NS,  INFINITE_CYCLES);
  }
  return true;
}

std::string
RoboyBodyMovement::getInfo()
{
	std::string info = "Body Movement";
	return info;      
}

bool
RoboyBodyMovement::isHandTouched(bool isLeftHand) {
  if(isLeftHand) {
    return canBus->readDigitalInput(LEFT_HAND_MOTOR_ID, DIGITAL_INPUT_1);
  } else {
    return canBus->readDigitalInput(RIGHT_HAND_MOTOR_ID, DIGITAL_INPUT_1);
  }
}

void
RoboyBodyMovement::isHandTouched(bool isLeftHand, bool isTouched) {
  bool output;

  output = isHandTouched(isLeftHand);
	while(output == !isTouched) {
    output = isHandTouched(isLeftHand);
    usleep(DIGITAL_INPUT_CHECKING_TIME_PERIOD_US);
	}
}

bool
RoboyBodyMovement::moveHead(int roll, int pitch, int yaw, bool blocking) {

	neckMotionCommand c;
  c.neckRollAngle = roll;
  c.neckPitchAngle = pitch;
  c.neckYawAngle = yaw;

	c.rollVelocity = DEFAULT_NECK_VELOCITY;
	c.pitchVelocity = DEFAULT_NECK_VELOCITY;
	c.yawVelocity = DEFAULT_NECK_VELOCITY;

	c.rollAcceleration = DEFAULT_NECK_ACCELERATION;
	c.pitchAcceleration = DEFAULT_NECK_ACCELERATION;
	c.yawAcceleration = DEFAULT_NECK_ACCELERATION;
  return moveHead(c, blocking);
}

bool
RoboyBodyMovement::moveHead(neckMotionCommand nMC, bool blocking) {
	neckMotorInfo a;
  bool usePDO;

  a = canBus->setNeckAngle(nMC);
  if(a.commandSent == false) return false;
    
	if(blocking) {
    if(canBus->allMotors[LEFT_NECK_ROLL_MOTOR_ID].getState() == OPERATIONAL) {
      usePDO = true;
    } else {
      usePDO = false;
    }
    canBus->isMotorReachedBlocking(LEFT_NECK_ROLL_MOTOR_ID, a.leftNeckRollMotorTargetValue, MOTOR_POSITION_ERROR_ALLOWANCE, usePDO, POSITION_CHECKING_TIME_PERIOD_NS,  INFINITE_CYCLES);
    if(canBus->allMotors[RIGHT_NECK_ROLL_MOTOR_ID].getState() == OPERATIONAL) {
      usePDO = true;
    } else {
      usePDO = false;
    }
    canBus->isMotorReachedBlocking(RIGHT_NECK_ROLL_MOTOR_ID, a.rightNeckRollMotorTargetValue, MOTOR_POSITION_ERROR_ALLOWANCE, usePDO, POSITION_CHECKING_TIME_PERIOD_NS,  INFINITE_CYCLES);
    if(canBus->allMotors[NECK_PITCH_MOTOR_ID].getState() == OPERATIONAL) {
      usePDO = true;
    } else {
      usePDO = false;
    }
    canBus->isMotorReachedBlocking(NECK_PITCH_MOTOR_ID, a.neckPitchMotorTargetValue, MOTOR_POSITION_ERROR_ALLOWANCE, usePDO, POSITION_CHECKING_TIME_PERIOD_NS,  INFINITE_CYCLES);
    if(canBus->allMotors[NECK_YAW_MOTOR_ID].getState() == OPERATIONAL) {
      usePDO = true;
    } else {
      usePDO = false;
    }
    canBus->isMotorReachedBlocking(NECK_YAW_MOTOR_ID, a.neckYawMotorTargetValue, MOTOR_POSITION_ERROR_ALLOWANCE, usePDO, POSITION_CHECKING_TIME_PERIOD_NS,  INFINITE_CYCLES);
  }
  return true;
}

void
RoboyBodyMovement::setLegSwingBehavior(int timePeriodUS, int velocity, int acceleration, int magnitude, int phaseShift, int swingSets) {

  legSwingTimePeriodUS = timePeriodUS;
  legSwingVelocity = velocity;
  legSwingAcceleration = acceleration;
  legSwingMagnitude = magnitude;
  legSwingPhaseShift = bound(phaseShift, MAX_PHASE_SHIFT, MIN_PHASE_SHIFT);
  legSwingSets = swingSets;
}

void 
RoboyBodyMovement::performLegSwing(int lSTP, int lSV, int lSA, int lSM, int lSPS, int lSS) {
  int swingPeriodUS, distanceCheck, dist, i, waitPeriod;
  double timeCheckS;

  if(lSTP == NO_LEG_SWING || lSM == NO_LEG_SWING || legSwingSets == NO_LEG_SWING) return;
  lSPS = bound(lSPS, MAX_PHASE_SHIFT, MIN_PHASE_SHIFT);

  if(lSA != canBus->allMotors[LEFT_KNEE_MOTOR_ID].readParameter(MAX_ACCELERATION) || lSV != canBus->allMotors[LEFT_KNEE_MOTOR_ID].readParameter(MAX_PROFILE_VELOCITY)) canBus->setMaxVelocityAndAcceleration(LEFT_KNEE_MOTOR_ID, lSV, lSA);

  if(lSA != canBus->allMotors[RIGHT_KNEE_MOTOR_ID].readParameter(MAX_ACCELERATION) || lSV != canBus->allMotors[RIGHT_KNEE_MOTOR_ID].readParameter(MAX_PROFILE_VELOCITY)) canBus->setMaxVelocityAndAcceleration(RIGHT_KNEE_MOTOR_ID, lSV, lSA);

  if(lSA != canBus->allMotors[RIGHT_HIP_REAR_MOTOR_ID].readParameter(MAX_ACCELERATION) || lSV != canBus->allMotors[RIGHT_HIP_REAR_MOTOR_ID].readParameter(MAX_PROFILE_VELOCITY)) canBus->setMaxVelocityAndAcceleration(RIGHT_HIP_REAR_MOTOR_ID, lSV, lSA);

  if(lSA != canBus->allMotors[RIGHT_HIP_FRONT_MOTOR_ID].readParameter(MAX_ACCELERATION) || lSV != canBus->allMotors[RIGHT_HIP_FRONT_MOTOR_ID].readParameter(MAX_PROFILE_VELOCITY)) canBus->setMaxVelocityAndAcceleration(RIGHT_HIP_FRONT_MOTOR_ID, lSV, lSA);

  if(lSA != canBus->allMotors[LEFT_HIP_FRONT_MOTOR_ID].readParameter(MAX_ACCELERATION) || lSV != canBus->allMotors[LEFT_HIP_FRONT_MOTOR_ID].readParameter(MAX_PROFILE_VELOCITY)) canBus->setMaxVelocityAndAcceleration(LEFT_HIP_FRONT_MOTOR_ID, lSV, lSA);

  if(lSA != canBus->allMotors[LEFT_HIP_REAR_MOTOR_ID].readParameter(MAX_ACCELERATION) || lSV != canBus->allMotors[LEFT_HIP_REAR_MOTOR_ID].readParameter(MAX_PROFILE_VELOCITY)) canBus->setMaxVelocityAndAcceleration(LEFT_HIP_REAR_MOTOR_ID, lSV, lSA);
  
  // Converting time from minutes to seconds
  timeCheckS = (double)(2.0*60.0)*((double)lSV/(double)lSA);
  
  dist = 0.5*lSV*CONVERT_RPM_TO_COUNTS_PER_SEC*timeCheckS;

  if(dist < lSM) {
    swingPeriodUS = (int)(timeCheckS*SECONDS_TO_MICRO_SECONDS + ((lSM - dist)*SECONDS_TO_MICRO_SECONDS)/(lSV*CONVERT_RPM_TO_COUNTS_PER_SEC));
  } else {
    swingPeriodUS = 2*sqrt(((float)lSM)/((float)lSA*CONVERT_RPM_TO_COUNTS_PER_SEC))*SECONDS_TO_MICRO_SECONDS;
  }

  canBus->setMotorControl(RIGHT_HIP_REAR_MOTOR_ID, RIGHT_HIP_REAR_REST_POSITION - lSM, POSITION_CONTROL);
  canBus->setMotorControl(RIGHT_HIP_FRONT_MOTOR_ID, RIGHT_HIP_FRONT_REST_POSITION - lSM, POSITION_CONTROL);
  canBus->setMotorControl(LEFT_HIP_FRONT_MOTOR_ID, LEFT_HIP_FRONT_REST_POSITION - lSM, POSITION_CONTROL);
  canBus->setMotorControl(LEFT_HIP_REAR_MOTOR_ID, LEFT_HIP_REAR_REST_POSITION - lSM, POSITION_CONTROL);

  for(i = 0; i < lSS; i++) {
  	canBus->setMotorControl(RIGHT_KNEE_MOTOR_ID, RIGHT_KNEE_REST_POSITION + lSM, POSITION_CONTROL);
    if(lSPS > 90) {
      usleep(swingPeriodUS);
      canBus->setControlValue(RIGHT_KNEE_MOTOR_ID, RIGHT_KNEE_REST_POSITION, POSITION_CONTROL);
      waitPeriod = swingPeriodUS*(lSPS - 90)/90;
      usleep(waitPeriod);
      canBus->setMotorControl(LEFT_KNEE_MOTOR_ID, LEFT_KNEE_REST_POSITION + lSM, POSITION_CONTROL);
      usleep(swingPeriodUS);
      canBus->setControlValue(LEFT_KNEE_MOTOR_ID, LEFT_KNEE_REST_POSITION, POSITION_CONTROL);
    } else {
      waitPeriod = swingPeriodUS*lSPS/90;
      usleep(waitPeriod);
      canBus->setMotorControl(LEFT_KNEE_MOTOR_ID, LEFT_KNEE_REST_POSITION + lSM, POSITION_CONTROL);
      waitPeriod = swingPeriodUS*(90 - lSPS)/90;
      usleep(waitPeriod);
      canBus->setControlValue(RIGHT_KNEE_MOTOR_ID, RIGHT_KNEE_REST_POSITION, POSITION_CONTROL);
      waitPeriod = swingPeriodUS*lSPS/90;
      usleep(waitPeriod);
      canBus->setControlValue(LEFT_KNEE_MOTOR_ID, LEFT_KNEE_REST_POSITION, POSITION_CONTROL);
      waitPeriod = swingPeriodUS*(90 - lSPS)/90;
      usleep(waitPeriod);
    }  
  }
  canBus->setControlValue(RIGHT_HIP_REAR_MOTOR_ID, RIGHT_HIP_REAR_REST_POSITION, POSITION_CONTROL);
  canBus->setControlValue(RIGHT_HIP_FRONT_MOTOR_ID, RIGHT_HIP_FRONT_REST_POSITION, POSITION_CONTROL);
  canBus->setControlValue(LEFT_HIP_FRONT_MOTOR_ID, LEFT_HIP_FRONT_REST_POSITION, POSITION_CONTROL);
  canBus->setControlValue(LEFT_HIP_REAR_MOTOR_ID, LEFT_HIP_REAR_REST_POSITION, POSITION_CONTROL);
  
  isLegSwinging = false; 

  if(lSPS > 90) {
    if((2*swingPeriodUS + swingPeriodUS*(lSPS - 90)/90)*lSS < lSTP) {
      usleep(lSTP - swingPeriodUS - swingPeriodUS*lSPS/90);
    }
  } else {
    waitPeriod = swingPeriodUS*lSPS/90;
    usleep(waitPeriod);
    if((2*swingPeriodUS)*lSS + swingPeriodUS*lSPS/90 < lSTP) {
      usleep(lSTP - swingPeriodUS - swingPeriodUS*lSPS/90);
    }
  }
}

void 
RoboyBodyMovement::legSwingExecuteThread() {

	while(true) {
		while (legSwingTimePeriodUS == NO_LEG_SWING || legSwingMagnitude == NO_LEG_SWING || legSwingSets == NO_LEG_SWING) {
			usleep(LEG_SWING_CHECK_TIME_PERIOD_US);
		}
    isLegSwinging = true;
    performLegSwing(legSwingTimePeriodUS, legSwingVelocity, legSwingAcceleration, legSwingMagnitude, legSwingPhaseShift, legSwingSets);
	}
}


}
