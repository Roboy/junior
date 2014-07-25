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
#include "Motor.h"

Motor::Motor(float K, float L, float G, float a, int v, int zFAI, int cPT, int mV, int mA) {
	int i;
	
	computeLookupTable(K, L, G, a, v, zFAI);
	
	maxVelocity = mV;
	maxAcceleration = mA;
	countsPerTurn = cPT;
	
	for(i = 0; i < TOTAL_PARAMETERS; i++) {
		MotorParameters[i] = 0;
		updated[i] = false;
	}
}

void Motor::setMotorCharacteristics(int cPT, int mV, int mA) {
	maxVelocity = mV;
	maxAcceleration = mA;
	countsPerTurn = cPT;
}
		
void Motor::computeLookupTable(float K, float L, float G, float a, int mV, int zFAI) {
	int i, curMilliVolt;

	springConstant = K;
	lengthAtRest = L;
	milliVoltTolengthChangeScale = G;
	halfPulleyDistance = a;
	maxMilliVolt = mV;
	zeroForceAnalogValue = zFAI;
		
	if(mV > MAX_MILLI_VOLT) printf("Error in setting motor values, max input voltage beyond ADC.");
	if(G*((float) mV) >= L) printf("Error in setting motor values, will lead to infinite force.");
	
	maxForce = 1; // In the case that G*((float) mV) >= L which can lead to division by zero.
	maxForce = convertMilliVoltToForce(maxMilliVolt);
	milliVoltStep = (maxMilliVolt - zeroForceAnalogValue)/INTERPOLATION_COUNT;
	curMilliVolt = maxMilliVolt - milliVoltStep;
	minMilliVolt = maxMilliVolt - (INTERPOLATION_COUNT - 1)*milliVoltStep;
	
	interpolatedForce[INTERPOLATION_COUNT - 1] = maxForce;
	
	for(i = INTERPOLATION_COUNT - 2; i >= 0; i--) {
		interpolatedForce[i] = convertMilliVoltToForce(curMilliVolt);
		curMilliVolt = curMilliVolt - milliVoltStep;
	}
}

void Motor::setParameter(int id, int value) {
	MotorParameters[id] = value;
	updated[id] = false;
	
	switch(id) {
		case DEMAND_FORCE_MV:
			updated[DEMAND_FORCE_N] = false;
			MotorParameters[DEMAND_FORCE_N] = convertMilliVoltToForce(value);
			break;
		case DIGITAL_INPUT_1_CONFIGURATION:
			digitalInputsAssignedBit[0] = 1 << value;
			break;
		case DIGITAL_INPUT_2_CONFIGURATION:
			digitalInputsAssignedBit[1] = 1 << value;
			break;
		case DIGITAL_INPUT_3_CONFIGURATION:
			digitalInputsAssignedBit[2] = 1 << value;
			break;
		case DIGITAL_INPUT_4_CONFIGURATION:
			digitalInputsAssignedBit[3] = 1 << value;
			break;
		case DIGITAL_INPUT_5_CONFIGURATION:
			digitalInputsAssignedBit[4] = 1 << value;
			break;
		case DIGITAL_INPUT_6_CONFIGURATION:
			digitalInputsAssignedBit[5] = 1 << value;
			break;
	}
}

CubicSplineResult Motor::cubicSpline(const char* inputFilePath, const char* outputFilePath, double timeBetweenWaypoints[], int numWaypoints) {
	CubicSplineResult result;
	Trajectory t;
	FILE *ptr_data;
	dataArrayWrapper waypoints;
	int i, j;
	
	result.success = false;
	
	waypoints = loadDataFileArray(inputFilePath, NUM_COLS_WAYPOINT_FILE);
	if(waypoints.row != numWaypoints + WAYPOINT_FILE_EXCESS_POINTS) {
		TRACE_RED_BOLD("Input file error: Expected Rows: %d, Obtained rows: %d.", numWaypoints + WAYPOINT_FILE_EXCESS_POINTS, waypoints.row);TRACE("\n");
		return result;
	}
	
	ptr_data = fopen(outputFilePath, "w");
	if(ptr_data == NULL) {
		TRACE_RED_BOLD("Unable to write to output file.");TRACE("\n");
		return result;
	}
	
	for(i = 1; i < numWaypoints; i++) {
		t = cubicSpline(waypoints.dataArray[i*3 + 2], waypoints.dataArray[i*3 + 1], waypoints.dataArray[(i+1)*3 + 2], waypoints.dataArray[(i+1)*3 + 1], timeBetweenWaypoints[i-1], WAYPOINT_TIME_STEP);
		if(t.numElements <= 1) {
			TRACE_RED_BOLD("Error in generating trajectory at waypoints: %d-%d.", i, i+1);TRACE("\n");
			return result;
		}
		if(i == 1) {
			result.maxAcceleration = t.maxAcceleration;
			result.maxVelocity = t.maxVelocity;
		} else {
			if(result.maxVelocity < t.maxVelocity) {
				result.maxVelocity = t.maxVelocity;
			}
			if(result.maxAcceleration < t.maxAcceleration) {
				result.maxAcceleration = t.maxAcceleration;
			}
		}
		for(j = 0; j < t.numElements - 1; j++) {
			fprintf(ptr_data, "%d %d %d\n", t.t[j].time, t.t[j].velocity, t.t[j].position);
		}
	}
	fprintf(ptr_data, "%d %d %d\n", t.t[t.numElements - 1].time, t.t[t.numElements - 1].velocity, t.t[t.numElements - 1].position);

	fclose(ptr_data);
	result.success = true;
	
	return result;
}


void Motor::confirmUpdate(int id, int value) {
	int i;
	MotorParameters[id] = value;
	updated[id] = true;
	
	switch(id) {
		case ANALOG_1_MV:
			MotorParameters[ACTUAL_FORCE_MV] = value;
			updated[ACTUAL_FORCE_MV] = true;
			MotorParameters[ACTUAL_FORCE_N] = convertMilliVoltToForce(value); 
			updated[ACTUAL_FORCE_N] = true;
			break;
		case ACTUAL_FORCE_MV:
			MotorParameters[ANALOG_1_MV] = value;
			updated[ANALOG_1_MV] = true;
			MotorParameters[ACTUAL_FORCE_N] = convertMilliVoltToForce(value); 
			updated[ACTUAL_FORCE_N] = true;
			break;
		case BUFFER_CURRENT_SIZE:
			MotorParameters[BUFFER_REMAINING_MEMORY] = MAX_BUFFER_SIZE - value;
			updated[BUFFER_REMAINING_MEMORY] = true;
			break;
		case BUFFER_REMAINING_MEMORY:
			MotorParameters[BUFFER_CURRENT_SIZE] = MAX_BUFFER_SIZE - value;
			updated[BUFFER_CURRENT_SIZE] = true;
			break;
		case DIGITAL_INPUT_STATES:
			for(i = 0; i < NUM_DIGITAL_INPUTS; i++) {
				digitalInputs[i] = ((digitalInputsAssignedBit[i] & value) > 0);
			}
			break;
	} 
}

Trajectory Motor::cubicSpline(int startPosition, int startVelocity, int endPosition, int endVelocity, double timePeriod, int timeStep) {

	return cubicSpline(startPosition, startVelocity, endPosition, endVelocity, timePeriod, timeStep, countsPerTurn, maxVelocity, maxAcceleration);
}


Trajectory Motor::cubicSpline(int startPosition, int startVelocity, int endPosition, int endVelocity, double timePeriod, int timeStep, int cPT, int mV, int mA) {
	double a, b, c, d, curTime, zeroAccTime, velocity, initialAcceleration, finalAcceleration;
	Trajectory outputTrajectory;
	int totalStep, i;
	
	outputTrajectory.numElements = 0;
	if(timeStep > MAX_TIME_STEP) return outputTrajectory;
	if(timePeriod*1000 < timeStep) {
		timeStep = (int)(timePeriod*1000);	
		totalStep = 2;		
	} else {
		totalStep = ((int)(timePeriod*1000))/timeStep + 1;
	}
	
	if(totalStep > MAX_BUFFER_SIZE - 1) return outputTrajectory;
	if(norm(endVelocity) > mV) return outputTrajectory;
	if(norm(startVelocity) > mV) return outputTrajectory;
	
	d = startPosition;
	// Converting to qc per second
	c = startVelocity*cPT/60.0;
	a = (c*timePeriod + 2*d + timePeriod*endVelocity*cPT/60.0 - 2*endPosition)/((double)timePeriod*timePeriod*timePeriod);
	b = (endVelocity*cPT/60.0 - c - 3*a*timePeriod*timePeriod)/(2*timePeriod);
	
	zeroAccTime = -b/(3.0*a);
	outputTrajectory.maxVelocity = max(norm(startVelocity), norm(endVelocity));
	// Checking max velocity
	if(zeroAccTime < timePeriod && zeroAccTime > 0) {
		velocity = 3*a*zeroAccTime*zeroAccTime + 2*b*zeroAccTime + c;
		// Convert to RPM
		velocity = velocity*60.0/((double) cPT);
		outputTrajectory.maxVelocity = max(outputTrajectory.maxVelocity, (int)norm(velocity));
		if(norm(velocity) > mV) return outputTrajectory;
	}
	//BREAK_POINT("Verified Maximum Velocity.");
	
	// Checking max accerlation
	// For cubic spline, the maximum acceleration is either at the start or the end
	initialAcceleration = norm(2*b*60.0/((double) cPT));
	finalAcceleration = norm((6*a*timePeriod + 2*b)*60.0/((double) cPT));
	
	outputTrajectory.maxAcceleration = max(finalAcceleration, initialAcceleration);
	
	if(initialAcceleration > mA || finalAcceleration > mA) return outputTrajectory;

	for(i = 0; i < totalStep - 1; i++) {
		curTime = ((double)(i+1)*timeStep)/1000.0;
		outputTrajectory.t[i].time = timeStep;
		outputTrajectory.t[i].velocity = ((int) ((3*a*curTime*curTime + 2*b*curTime + c)*60.0/((double) cPT)));
		outputTrajectory.t[i].position = ((int) (a*curTime*curTime*curTime + b*curTime*curTime + c*curTime + d));
	}
	// To account for the remaining time if the time period is not a factor of the time step
	if((totalStep - 1)*timeStep < ((int)timePeriod*1000)) {
		curTime = timePeriod;
		outputTrajectory.t[totalStep-1].time = ((int)(timePeriod*1000)) - (totalStep - 1)*timeStep;
		outputTrajectory.t[totalStep-1].velocity = ((int) ((3*a*curTime*curTime + 2*b*curTime + c)*60.0/((double) cPT)));
		outputTrajectory.t[totalStep-1].position = ((int) (a*curTime*curTime*curTime + b*curTime*curTime + c*curTime + d));
		totalStep = totalStep + 1;
	}
	outputTrajectory.numElements = totalStep;
	
	outputTrajectory.t[totalStep - 1].time = 0;
	outputTrajectory.t[totalStep - 1].velocity = 0;
	if(totalStep > 1) {
		outputTrajectory.t[totalStep - 1].position = outputTrajectory.t[totalStep - 2].position;
	} else {
		outputTrajectory.t[totalStep - 1].position = startPosition;
	}
	return outputTrajectory;
}


void Motor::confirmUpdate(int id) {
	updated[id] = true;		
	if(id == DEMAND_FORCE_MV) {
		updated[DEMAND_FORCE_N] = true;
	}	
}

int Motor::readParameter(int id) {
	return MotorParameters[id];
}

bool Motor::isUpdated(int id) {
	return updated[id];
}

void Motor::requestParameter(int id) {
	updated[id] = false;
}
	
int Motor::convertForceToMilliVolt(float force) {

	int searchIndex;
	
	if(force <= interpolatedForce[0]) {
		return ((int)(((float)force*(minMilliVolt - zeroForceAnalogValue))/interpolatedForce[0]));
	}
	if(force >= maxForce) return maxMilliVolt;
	
	// A very rough estimate assuming linear relationship.
	searchIndex = ((int)(force*((float)INTERPOLATION_COUNT)/maxForce)); 
	
	// Due to nonlinear relationship between force and milliVolt
	if(searchIndex == 0) searchIndex = 1; 
	
	if(interpolatedForce[searchIndex] < force) {
		searchIndex++;
		while(interpolatedForce[searchIndex] < force) {
			searchIndex++;
			if(searchIndex > INTERPOLATION_COUNT - 1) break;
		}
	} else if(interpolatedForce[searchIndex - 1] > force) {
		searchIndex--;
		while(interpolatedForce[searchIndex-1] > force) {
			searchIndex--;
			if(searchIndex <= 1) break;
		}
	}
	
	return interpolate(interpolatedForce[searchIndex], interpolatedForce[searchIndex - 1], force);
}

int Motor::interpolate(float larger, float smaller, float value) {
	return ((int)(((float)(value - smaller)*milliVoltStep)/(larger - smaller)));
}

float Motor::convertMilliVoltToForce(int milliVolt) {
	float dL, reducedL;
	if(milliVolt <= 0) return 0;
	
	dL = milliVoltTolengthChangeScale*((float) milliVolt - zeroForceAnalogValue);
	reducedL = lengthAtRest - dL;
	
	if(reducedL <= 0) return maxForce;
	
	return dL*springConstant*sqrt(reducedL*reducedL + halfPulleyDistance*halfPulleyDistance)/(2.0*reducedL);
}

bool Motor::isAlive() {
	int maxTimeInterval;
	
	if(MotorParameters[HEARTBEAT_INTERVAL] == 0) return IS_ALIVE;
	
	maxTimeInterval = MotorParameters[HEARTBEAT_INTERVAL]/1000 + HEARTBEAT_TOLERANCE_S;
	
	if(time(NULL) > maxTimeInterval + lastHeartBeat) return NOT_ALIVE;

	return IS_ALIVE;
}

void Motor::heartbeat(int s) {
	lastHeartBeat = time(NULL);
	state = s;
}

void Motor::setState(int s) {
	state = s;
}


