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

int main() {
	int controlType, value, motorID, commsStatus;
	char buffer[100];

	Robot *canBus = new Robot("../../../../../roboy/", false);
	canBus->initialize(CAN_BUS_NODE);
	
	while(true) {

		printf("Enter Motor to Control (type 'exit' to exit program): ");
		gets(buffer);
		if(strcmp(buffer, "exit") == 0) {
			printf("Exiting program...\n");
			//canBus->stopMotorControl(motorID);
			canBus->uninitialize();
			break;
		}
		motorID = 0xff & atoi(buffer);
		if(motorID < 1 || motorID > TOTAL_MOTORS_IN_ROBOT) {
			printf("Invalid Motor ID.\n");
			continue;
		}
		canBus->clearFault(motorID);
		
		canBus->updateParamMultiple(motorID, ACTUAL_POSITION);
		canBus->updateParamMultiple(motorID, ACTUAL_VELOCITY);
		commsStatus = canBus->readSDOSequence(300);
		
		if(commsStatus != SEND_SUCCESSFUL) {
			printf("Communication Error.\n");
			continue;
		}
		
		printf("Motor %d:\n  Velocity: %d\n  Position: %d\n", motorID, canBus->allMotors[motorID].readParameter(ACTUAL_VELOCITY), canBus->allMotors[motorID].readParameter(ACTUAL_POSITION));
		
		printf("Enter control type (1: Position, 2:Velocity, 3: Current) (type 'exit' to exit program): ");
		gets(buffer);
		
		if(strcmp(buffer, "exit") == 0) {
			printf("Exiting program...\n");
			canBus->stopMotorControl(motorID);
			canBus->uninitialize();
			break;
		}
		controlType = 0xff & atoi(buffer);
		if(controlType < 1 || controlType > 3) continue;
		//canBus->stopMotorControl(motorID);
		printf("Enter target value: ");
		gets(buffer);
		value = 0xffffffff & atoi(buffer);
		canBus->initializeMotorForControl(motorID);
		canBus->setMaxVelocityAndAcceleration(motorID, 5000, 16000);
				
		switch(controlType) {
			case 1:
				canBus->setMotorControl(motorID, value, POSITION_CONTROL);
				break;
			case 2:
				canBus->setMotorControl(motorID, value, VELOCITY_CONTROL);
				break;
			case 3:
				canBus->setMotorControl(motorID, value, CURRENT_CONTROL);
				break;
		}
	}
}

