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
#define NUM_MOTOR 7

int main() {
	Robot *canBus = new Robot("../../../../../roboy/", false);
	PauseTimer delayTimmer(100000000);
	std::stringstream ss;
	struct timeval curTime;	
	FILE *ptr_data[TOTAL_MOTORS_IN_ROBOT];
	char buffer[100];
	int i, timeElapsed, position, velocity;
	int minimumTension[NUM_MOTOR];
	int motorID[NUM_MOTOR];
	
	int maxVelocity[NUM_MOTOR];
	std::string fileName[NUM_MOTOR];
	motorID[0] = 38;
	motorID[1] = 39;
	motorID[2] = 40;
	motorID[3] = 41;
	motorID[4] = 42;
	motorID[5] = 43;
	motorID[6] = 44;
	
	minimumTension[0] = 200;
	minimumTension[1] = 200;
	minimumTension[2] = 200;
	minimumTension[3] = 200;
	minimumTension[4] = 200;
	minimumTension[5] = 200;
	minimumTension[6] = 200;

	canBus->initialize(CAN_BUS_NODE);
	delayTimmer.wait();

	if(NUM_MOTOR > TOTAL_MOTORS_IN_ROBOT) return 0;	
	canBus->enterPreOperational();
	
	for(i = 0; i < NUM_MOTOR; i++) { 
		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) return 0;	
		canBus->clearFault(motorID[i]);	
		canBus->initializePDORXMapping(motorID[i]);
		canBus->initializePDOTXMapping(motorID[i]);
		canBus->initializeMotorForControl(motorID[i]);
	}
	
	delayTimmer.wait();
	for(i = 0; i < NUM_MOTOR; i++) { 
		canBus->startNode(motorID[i]);
	}
	canBus->startForceControl(motorID, minimumTension, NUM_MOTOR);
	canBus->startRecord(motorID, NUM_MOTOR);
	
	for(i = 0; i < NUM_MOTOR; i++) {
		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT || motorID[i] <= 0) continue;
		ss.str("");
		ss << canBus->recordFolder;
		if(motorID[i] < 10) {
			ss << "/Motor0" << motorID[i] << "Waypoints.txt";
		} else {
			ss << "/Motor" << motorID[i] << "Waypoints.txt";
		}
		ptr_data[motorID[i] - 1] = fopen(ss.str().c_str(),"w");
	}

	for(i = 0; i < NUM_MOTOR; i++) {
		if(ptr_data[motorID[i] - 1] == NULL) continue;
		position = canBus->allMotors[motorID[i]].readParameter(ACTUAL_POSITION);
		velocity = canBus->allMotors[motorID[i]].readParameter(ACTUAL_VELOCITY);
		fprintf(ptr_data[motorID[i] - 1], "%d %d %d\n", 0, velocity, position);
	}		

	while(true) {
		gets(buffer);
		if(strncmp(buffer, "e", 1) == 0) {
			break;
		}
		gettimeofday(&curTime, NULL);
		// To account for day change
		if(curTime.tv_sec < canBus->recordStartTime.tv_sec) {
			timeElapsed = (curTime.tv_sec - canBus->recordStartTime.tv_sec + 86400)*1000 + (curTime.tv_usec - canBus->recordStartTime.tv_usec)/1000;
		} else {
			timeElapsed = (curTime.tv_sec - canBus->recordStartTime.tv_sec)*1000 + (curTime.tv_usec - canBus->recordStartTime.tv_usec)/1000;
		}
		
		for(i = 0; i < NUM_MOTOR; i++) {
			if(ptr_data[motorID[i] - 1] == NULL) continue;
			position = canBus->allMotors[motorID[i]].readParameter(ACTUAL_POSITION);
			velocity = canBus->allMotors[motorID[i]].readParameter(ACTUAL_VELOCITY);
			fprintf(ptr_data[motorID[i] - 1], "%d %d %d\n", timeElapsed, velocity, position);
		}		
	}
	gettimeofday(&curTime, NULL);
	// To account for day change
	if(curTime.tv_sec < canBus->recordStartTime.tv_sec) {
		timeElapsed = (curTime.tv_sec - canBus->recordStartTime.tv_sec + 86400)*1000 + (curTime.tv_usec - canBus->recordStartTime.tv_usec)/1000;
	} else {
		timeElapsed = (curTime.tv_sec - canBus->recordStartTime.tv_sec)*1000 + (curTime.tv_usec - canBus->recordStartTime.tv_usec)/1000;
	}
		
	for(i = 0; i < NUM_MOTOR; i++) {
		if(ptr_data[motorID[i] - 1] == NULL) continue;
		position = canBus->allMotors[motorID[i]].readParameter(ACTUAL_POSITION);
		velocity = canBus->allMotors[motorID[i]].readParameter(ACTUAL_VELOCITY);
		fprintf(ptr_data[motorID[i] - 1], "%d %d %d\n", timeElapsed, velocity, position);
		fclose(ptr_data[motorID[i] - 1]);
	}		
	
	canBus->stopForceControl();
	canBus->stopRecord();
	canBus->enterPreOperational();
	canBus->uninitialize();
	delayTimmer.wait();
	
}

