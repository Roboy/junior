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
	int i;
	char buffer[100];
	neckMotorInfo a;
	neckMotionCommand c;
	canBus->initialize(CAN_BUS_NODE);
	
	canBus->enterPreOperational();
	
	for(i = 45; i < 49; i++) {
		canBus->clearFault(i);
		canBus->initializeMotorForControl(i);
	}	
	
	while(true) {
		
		printf("Neck Roll %d:\n  Neck Pitch: %d\n  Neck Yaw: %d\n", canBus->getNeckRoll(), canBus->getNeckPitch(), canBus->getNeckYaw());
		
		printf("Enter desired Roll angle (type 'exit' to exit program): ");
		gets(buffer);
		
		if(strcmp(buffer, "exit") == 0) {
			printf("Exiting program...\n");
			canBus->uninitialize();
			break;
		}
		c.neckRollAngle = atoi(buffer);
		if(c.neckRollAngle < MIN_NECK_ROLL_ANGLE || c.neckRollAngle > MAX_NECK_ROLL_ANGLE) continue;
		
		printf("Enter desired Pitch angle: ");
		gets(buffer);
		c.neckPitchAngle = atoi(buffer);
		if(c.neckPitchAngle < MIN_NECK_PITCH_ANGLE || c.neckPitchAngle > MAX_NECK_PITCH_ANGLE) continue;

		printf("Enter desired Yaw angle: ");
		gets(buffer);
		c.neckYawAngle = atoi(buffer);
		if(c.neckYawAngle < MIN_NECK_YAW_ANGLE || c.neckPitchAngle > MAX_NECK_YAW_ANGLE) continue;

		c.rollVelocity = 3000;
		c.pitchVelocity = 3000;
		c.yawVelocity = 3000;

		c.rollAcceleration = 4000;
		c.pitchAcceleration = 4000;
		c.yawAcceleration = 4000;
		
		a = canBus->setNeckAngle(c);
		if(a.commandSent) {
			printf("Moving to [%d %d %d %d]\n", a.leftNeckRollMotorTargetValue, a.rightNeckRollMotorTargetValue, a.neckPitchMotorTargetValue, a.neckYawMotorTargetValue);
		} else {
			printf("Error\n");
		}
	}	
	
	canBus->uninitialize();
	return 0;
}

