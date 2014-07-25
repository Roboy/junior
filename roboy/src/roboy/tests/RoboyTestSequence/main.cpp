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
#include "main.h"

void initializeRobot(Robot *canBus, int motorID[], int numMotor) {
	int maxVelocity, i;

	canBus->initialize(CAN_BUS_NODE);

	canBus->enterPreOperational();
	for(i = 0; i < numMotor; i++) {
		canBus->clearFault(motorID[i]);
		canBus->initializePDORXMapping(motorID[i]);
		canBus->initializePDOTXMapping(motorID[i]);
		canBus->resetInterpolatedBuffer(motorID[i]);
		canBus->initializeMotorForControl(motorID[i]);
		// Increase the max velocity by 1000 RPM so that there will be no errors when computing the 
		// interconnecting trajectory
		
		if((motorID[i] >= 2 && motorID[i] <= 4) || motorID[i] == 23 || motorID[i] == 24 || (motorID[i] >= 26 && i <= 30) || (motorID[i] >= 45 && motorID[i] <= 48)) {
			maxVelocity = 11000;
		} else {
			if(motorID[i] == 1 || motorID[i] == 25) {
				maxVelocity = 5000;
			} else {
				maxVelocity = 7000;
			}
		}
		canBus->setMaxVelocityAndAcceleration(motorID[i], maxVelocity + 1000, 320000);
	}	
	canBus->initializeDigitalInput(25, 0, DIGITAL_INPUT_GENERAL_PURPOSE_A, ACTIVE_HIGH);
	canBus->initializeDigitalInput(25, 1, DIGITAL_INPUT_GENERAL_PURPOSE_B, ACTIVE_HIGH);
	canBus->initializeDigitalInput(25, 2, DIGITAL_INPUT_GENERAL_PURPOSE_C, ACTIVE_HIGH);
	canBus->initializeDigitalInput(25, 3, DIGITAL_INPUT_GENERAL_PURPOSE_D, ACTIVE_HIGH);

}


struct trajectoryFileInfo {
	int motorID, numWayPoints, totalTimeMS;
};

void playTrajectory(char *folderName, Robot *canBus) {
	std::string filePaths[TOTAL_MOTORS_IN_ROBOT];
	char buffer[100];

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
	
		TRACE_PURP_BOLD("To Load Motor [%d] (%d RPM) Num waypoints: [%d] Total time (ms): [%d]", motorID[i], maxVelocity[i], tFileInfo[i].numWayPoints, tFileInfo[i].totalTimeMS);TRACE("\n");			
	}
	gets(buffer);

	for(i = 0; i < tFileInfo.size(); i++) {
		canBus->setMotorControlModePDO(motorID[i], OPERATION_MODE_INTERPOLATED_POSITION);
	}	

	canBus->readTrajectoryMultipleInterpolate(motorID, filePaths, maxVelocity, tFileInfo.size(), true);
	
	while(canBus->isIPMBufferEmpty(motorID, tFileInfo.size()) == false) {
		waitTimer.wait();
	}
	for(i = 0; i < tFileInfo.size(); i++) {
		canBus->stopControlPDO(motorID[i], OPERATION_MODE_INTERPOLATED_POSITION);
	}	
	waitTimer.wait();
	
	free(motorID);
	free(maxVelocity);
}

int main() {
	Robot *canBus = new Robot("../../../../../roboy/", false);
	PauseTimer waitTimer(1000000000);
	char buffer[100];
	int motorID[17];
	bool output;
	
	motorID[0] = 14;
	motorID[1] = 15;
	motorID[2] = 16;
	motorID[3] = 17;
	motorID[4] = 18;
	motorID[5] = 19;
	motorID[6] = 20;
	motorID[7] = 21;
	motorID[8] = 22;
	motorID[9] = 23;
	motorID[10] = 24;
	motorID[11] = 25;
	motorID[12] = 26;
	motorID[13] = 27;
	motorID[14] = 28;
	motorID[15] = 29;
	motorID[16] = 30;
	
	initializeRobot(canBus, motorID, 17);
	canBus->startNode();
	canBus->setMotorControl(25, 0, POSITION_CONTROL);
	canBus->isMotorReachedBlocking(25, 0, 2000, true, 250000000, 0);

	playTrajectory((char *) "GivingHand_5", canBus);
	output = false;
	while(output == false) {
		output = canBus->readDigitalInput(25, 0);
	}
	canBus->setMotorControl(25, 600000, POSITION_CONTROL);
	canBus->isMotorReachedBlocking(25, 600000, 2000, true, 250000000, 0);
	
	playTrajectory((char *) "ShakingHand_5", canBus);
	canBus->setMotorControl(25, 0, POSITION_CONTROL);
	canBus->isMotorReachedBlocking(25, 0, 2000, true, 250000000, 0);
	output = true;
	while(output == true) {
		output = canBus->readDigitalInput(25, 0);
	}
	playTrajectory((char *) "DrawingBack_5", canBus);
	
	canBus->enterPreOperational();
	canBus->uninitialize();
	
	return 0;
}

