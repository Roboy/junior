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
	printf("Before init.\n");
	canBus->initialize(CAN_BUS_NODE);
	printf("After init.\n");
	
	canBus->enterPreOperational();
	printf("After preop.\n");
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		canBus->clearFault(i);
		canBus->updateParamMultiple(i, ANALOG_1_MV);
		canBus->updateParamMultiple(i, ANALOG_2_MV);
		if(canBus->readSDOSequence() != SEND_SUCCESSFUL) {
			TRACE_RED_BOLD("Error in communicating with Motor %d", i); TRACE("\n");
		}		
		printf("Motor %d\t A1: %d, A2:%d\n", i, canBus->allMotors[i].readParameter(ANALOG_1_MV), canBus->allMotors[i].readParameter(ANALOG_2_MV));
	}	
	canBus->uninitialize();
	return 0;
}

