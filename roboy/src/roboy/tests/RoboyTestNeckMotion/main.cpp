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
	Robot *canBus = new Robot("../../../../../roboy/", false);
	int controlAngle, angle, commsStatus, i;
	char buffer[100];
	neckMotorInfo a;
	
	canBus->initialize(CAN_BUS_NODE);
	
	canBus->enterPreOperational();
	
	for(i = 45; i < 49; i++) {
		canBus->clearFault(i);
		canBus->initializeMotorForControl(i);
	}	
	while(true) {
		
		printf("Neck Roll %d:\n  Neck Pitch: %d\n  Neck Yaw: %d\n", canBus->getNeckRoll(), canBus->getNeckPitch(), canBus->getNeckYaw());
		
		printf("Enter control angle (1: Roll, 2: Pitch, 3: Yaw) (type 'exit' to exit program): ");
		gets(buffer);
		
		if(strcmp(buffer, "exit") == 0) {
			printf("Exiting program...\n");
			for(i = 45; i < 49; i++) {
				canBus->stopMotorControl(i);
			}	
			canBus->uninitialize();
			break;
		}
		controlAngle = 0xff & atoi(buffer);
		if(controlAngle < 1 || controlAngle > 3) continue;
		printf("Enter target angle: ");
		gets(buffer);
		angle = 0xffffffff & atoi(buffer);
		
		a = canBus->setNeckAngle(angle, controlAngle);
		if(a.commandSent) {
			printf("Moving to [%d %d %d %d]\n", a.leftNeckRollMotorTargetValue, a.rightNeckRollMotorTargetValue, a.neckPitchMotorTargetValue, a.neckYawMotorTargetValue);
		} else {
			printf("Error\n");
		}
	}	
	
	canBus->uninitialize();
	return 0;
}

