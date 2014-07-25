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

struct trajectoryFileInfo {
	int motorID, numWayPoints, totalTimeMS;
};

int main(int argc, const char *argv[]) {
	char buffer[100];
	std::string filePaths[TOTAL_MOTORS_IN_ROBOT];

	Robot *canBus = new Robot("../../../../../roboy/", false);
	int i, j;
	std::stringstream ss, folderPathStream;
	vector<trajectoryFileInfo> tFileInfo;
	int *motorID;
	int *maxVelocity;
		
	PauseTimer waitTimer(1000000000);
	dataArrayWrapper testFileExists;
	trajectoryFileInfo tmp;
	
	if(argc != 2) {
		std::cerr << "usage: " << argv[0] << " <folder>"  << std::endl; 
		std::cerr << "Example:"  << std::endl;
		std::cerr << argv[0] << " GivingHand"  << std::endl;
		return 0;
	}
	
	folderPathStream.str("");	
	folderPathStream << canBus->pathToTrajectories.str().c_str() << argv[1] << "/";

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
		return 0;
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

	canBus->initialize(CAN_BUS_NODE);
	
	waitTimer.wait();
	canBus->enterPreOperational();
	for(i = 0; i < tFileInfo.size(); i++) {
		canBus->clearFault(motorID[i]);
		canBus->initializePDORXMapping(motorID[i]);
		canBus->initializePDOTXMapping(motorID[i]);
		canBus->resetInterpolatedBuffer(motorID[i]);
		canBus->initializeMotorForControl(motorID[i]);
		// Increase the max velocity by 1000 RPM so that there will be no errors when computing the 
		// interconnecting trajectory
		canBus->setMaxVelocityAndAcceleration(motorID[i], maxVelocity[i] + 1000, 320000);
	}	
	
	waitTimer.wait();
	canBus->startNode();
	printf("Completed Initialization. Press ENTER to continue.\n");
	gets(buffer);
	canBus->readTrajectoryMultipleInterpolate(motorID, filePaths, maxVelocity, tFileInfo.size(), true);
	printf("Completed Loading. Press ENTER to exit.\n");
	gets(buffer);
	
	for(i = 0; i < tFileInfo.size(); i++) {
		canBus->stopControlPDO(motorID[i], OPERATION_MODE_INTERPOLATED_POSITION);
	}	
	waitTimer.wait();
	canBus->enterPreOperational();
	canBus->uninitialize();
	
	free(motorID);
	free(maxVelocity);
	
	return 0;
}

