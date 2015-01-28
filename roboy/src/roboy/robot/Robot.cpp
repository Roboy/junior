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
#include "Robot.h"
#include "nodebug.h"

const int Robot::addresses[TOTAL_PARAMETERS] = {A01, A02, A03, A04, A05, A06, A07, A08, A09, A10, A11,A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29, A30, A31, A32, A33, A34, A35, A36, A37, A38, A39, A40, A41, A42, A43, A44, A45, A46, A47, A48, A49, A50, A51, A52, A53, A54, A55, A56, A57, A58, A59, A60, A61, A62, A63, A64, A65, A66, A67, A68, A69, A70, A71, A72, A73, A74, A75, A76, A77, A78, A79, A80, A81, A82, A83, A84, A85, A86, A87, A88, A89, A90, A91, A92, A93, A94, A95, A96, A97, A98, A99, A100};

const int Robot::subIndices[TOTAL_PARAMETERS] = {S01, S02, S03, S04, S05, S06, S07, S08, S09, S10, S11, S12, S13, S14, S15, S16, S17, S18, S19, S20, S21, S22, S23, S24, S25, S26, S27, S28, S29, S30, S31, S32, S33, S34, S35, S36, S37, S38, S39, S40, S41, S42, S43, S44, S45, S46, S47, S48, S49, S50, S51, S52, S53, S54, S55, S56, S57, S58, S59, S60, S61, S62, S63, S64, S65, S66, S67, S68, S69, S70, S71, S72, S73, S74, S75, S76, S77, S78, S79, S80, S81, S82, S83, S84, S85, S86, S87, S88, S89, S90, S91, S92, S93, S94, S95, S96, S97, S98, S99, S100};

const int Robot::dataSizes[TOTAL_PARAMETERS] = {D01, D02, D03, D04, D05, D06, D07, D08, D09, D10, D11, D12, D13, D14, D15, D16, D17, D18, D19, D20, D21, D22, D23, D24, D25, D26, D27, D28, D29, D30, D31, D32, D33, D34, D35, D36, D37, D38, D39, D40, D41, D42, D43, D44, D45, D46, D47, D48, D49, D50, D51, D52, D53, D54, D55, D56, D57, D58, D59, D60, D61, D62, D63, D64, D65, D66, D67, D68, D69, D70, D71, D72, D73, D74, D75, D76, D77, D78, D79, D80, D81, D82, D83, D84, D85, D86, D87, D88, D89, D90, D91, D92, D93, D94, D95, D96, D97, D98, D99, D100};

const bool Robot::isSignedList[TOTAL_PARAMETERS] = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

const int Robot::PDOTXDataLength[PDO_NUM] = {PDO1_TX_DATA_LENGTH, PDO2_TX_DATA_LENGTH, PDO3_TX_DATA_LENGTH, PDO4_TX_DATA_LENGTH};
const int Robot::PDOTXIDStart[PDO_NUM] = {PDO1_TX_START, PDO2_TX_START, PDO3_TX_START, PDO4_TX_START};

const int Robot::PDOTXNumEntry[PDO_NUM] = {PDO1_TX_SIZE, PDO2_TX_SIZE, PDO3_TX_SIZE, PDO4_TX_SIZE};
const int Robot::PDOTXMotorParamMap[PDO_NUM][MAX_PDO_ENTRY] = {{CONTROL_WORD, OPERATION_MODE, INTERPOLATION_MODE_STATUS,0,0,0,0,0}, {ANALOG_1_MV, ANALOG_2_MV, 0,0,0,0,0,0},{ACTUAL_POSITION,ACTUAL_VELOCITY,0,0,0,0,0,0},{BUFFER_REMAINING_MEMORY,0,0,0,0,0,0,0}};
const int Robot::PDOTXEntryDataLength[PDO_NUM][MAX_PDO_ENTRY] = {{PDO1_TX1_DATA_LENGTH, PDO1_TX2_DATA_LENGTH, PDO1_TX3_DATA_LENGTH,0,0,0,0,0}, {PDO2_TX1_DATA_LENGTH, PDO2_TX2_DATA_LENGTH, 0,0,0,0,0,0},{PDO3_TX1_DATA_LENGTH,PDO3_TX2_DATA_LENGTH,0,0,0,0,0,0},{PDO4_TX1_DATA_LENGTH,0,0,0,0,0,0,0}};

SDOElement::SDOElement(int n, int p, int a, int s, int v, int d) {
	nodeID = n; 
	paramID = p;
	address = a;
	subIndex = s;
	value = v; 
	dataSize = d;
}

Robot::Robot(const char *relativePathToDataBaseFolder, bool RecordJointPosition) : MaxonCANOpen() {

	std::stringstream ss;
	dataArrayWrapper dArray;
	int i;
	int zeroForceSensorMap[TOTAL_MOTORS_IN_ROBOT + 1];
	Motor dummy(10000,0.1,0.000019,0.05,4800, 200, 2048, 8000, 16000);
	
	for(i = 0; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		zeroForceSensorMap[i] = 200;
	}
	
	ss.str("");
	ss << relativePathToDataBaseFolder << ZERO_FORCE_SENSOR_MAP;
	dArray = loadDataFileArray(ss.str().c_str(), NUM_COLS_IN_ZERO_FORCE_SENSOR_MAP_FILE);
	robotErrorFileIndex = addLogFile(ROBOT_ERROR_FILE_NAME);
	robotLogFileIndex = addLogFile(ROBOT_LOG_FILE_NAME);
	/*
	printf("File index: %d, File ptr: %ld\n", robotErrorFileIndex, logFile[robotErrorFileIndex]);
	printf("File index: %d, File ptr: %ld\n", robotLogFileIndex, logFile[robotLogFileIndex]);
	
	TRACE_ERROR_ROBOT_LN("Testing error file...");*/
	for(i = 0; i < dArray.row; i++) {
		zeroForceSensorMap[(int) dArray.dataArray[i*2]] = (int) dArray.dataArray[i*2 + 1];
		TRACE_LOG_CYAN_ROBOT_LN("Zero Force Sensor Map for Motor %d: %d", (int) dArray.dataArray[i*2] , zeroForceSensorMap[(int) dArray.dataArray[i*2]]);
	}
	free(dArray.dataArray);

	// To handle signed value conversions
	// 0 stands for int24
	// 1 stands for int16
	// 2 stands for int8
	maxValue[0] = 0xffffff;
	halfValue[0] = 0x7fffff;
		
	maxValue[1] = 0xffff;
	halfValue[1] = 0x7fff;
		
	maxValue[2] = 0xff;
	halfValue[2] = 0x7f;

	// To read in a text file that contains all the motor information.
	// This is required if we go with the force-analog conversion on
	// the cpu side.
	
	// Motor 0 is a dummy motor and will not be and should not accessed
	// This is to match the node ID on the CAN bus framework with the 
	// motor ID.
	ss.str("");
	ss << relativePathToDataBaseFolder << MOTOR_CHARACTERISTICS_FILE;
	dArray = loadDataFileArray(ss.str().c_str(), NUM_COLS_IN_MOTOR_CHARACTERISTICS_FILE);
	allMotors.push_back(dummy);
	
	if(dArray.row != TOTAL_MOTORS_IN_ROBOT) {
		TRACE_ERROR_ROBOT_LN("Motor characteristics file (%d) does not match total number of motors (%d)", dArray.row, TOTAL_MOTORS_IN_ROBOT);
		
		for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
			// Read in file for motor (i) and update parameters
			dummy.computeLookupTable(10000,0.1,0.000019,0.05,4800, zeroForceSensorMap[i]);
			dummy.setMotorCharacteristics(2048, 8000, 16000);
			allMotors.push_back(dummy);
			clearFault(i);
		}
	} else {
		for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
			// Read in file for motor (i) and update parameters
			dummy.computeLookupTable(10000,0.1,0.000019,0.05,4800, zeroForceSensorMap[i]);
			dummy.setMotorCharacteristics(dArray.dataArray[(i-1)*NUM_COLS_IN_MOTOR_CHARACTERISTICS_FILE], dArray.dataArray[(i-1)*NUM_COLS_IN_MOTOR_CHARACTERISTICS_FILE + 1], dArray.dataArray[(i-1)*NUM_COLS_IN_MOTOR_CHARACTERISTICS_FILE + 2]);
			allMotors.push_back(dummy);
			clearFault(i);
		}
	}
	
	
	lineOpen = false;
	recording = false;
	inForceControl = false;
	
	ss.str("");
	ss << relativePathToDataBaseFolder << NECK_ROLL_FILE_NAME;
	neckRollLookupTable = loadDataFileArray(ss.str().c_str(), NUM_COLS_IN_NECK_ROLL_LOOKUP_FILE);
	ss.str("");
	ss << relativePathToDataBaseFolder << NECK_PITCH_FILE_NAME;
	neckPitchLookupTable = loadDataFileArray(ss.str().c_str(), NUM_COLS_IN_NECK_PITCH_LOOKUP_FILE);
	ss.str("");
	ss << relativePathToDataBaseFolder << NECK_YAW_FILE_NAME;
	neckYawLookupTable = loadDataFileArray(ss.str().c_str(), NUM_COLS_IN_NECK_YAW_LOOKUP_FILE);
	
	if(RecordJointPosition == true) startJointPositionRecord();
	
	rPathToDataBaseFolder << relativePathToDataBaseFolder;
	printf("Database relative path: %s\n", rPathToDataBaseFolder.str().c_str());
	pathToTrajectories << rPathToDataBaseFolder.str().c_str() << PATH_TO_TRAJECTORIES;
} 



/*************** PDO TX handling, highly dependent on PDO settings ******************/

// Currently PDO 1 contains information on Control Word and Mode of Operation
void Robot::handlePDO(TPCANRdMsg readMsg, unsigned int id) {
	int motorID, i, j, index, value, operationMode, IPMBufferStatus;
	bool fault;
	
	fault = false;
	
	if(id >= PDO_NUM) return;
	if(readMsg.Msg.LEN != PDOTXDataLength[id]) return;
	motorID = readMsg.Msg.ID - PDOTXIDStart[id];
	if(motorID > TOTAL_MOTORS_IN_ROBOT) return;
	
	index = 0;
	TRACE_LOG_GREEN_ROBOT_TIME("Extracted PDO value: ");
	for(i = 0; i < PDOTXNumEntry[id]; i++) {
		value = 0;
		for(j = index; j < index + PDOTXEntryDataLength[id][i]; j++) {
			value = value + (readMsg.Msg.DATA[j]<<(8*(j-index)));
		}
		if(isSignedList[PDOTXMotorParamMap[id][i]]) {
			// Conversion using a coding, int8 = 3, int16 = 2, int24 = 1, int32 = 0 
			value = convertActualValue(value, 4 - PDOTXEntryDataLength[id][i]);
		}	
		index = j;
		TRACE_LOG_GREEN_ROBOT("[%04x] ", value);
		if(id == 0 && i == 0 && ((value & 0b1000) != 0)) fault = true;
		allMotors[motorID].confirmUpdate(PDOTXMotorParamMap[id][i], (int) value);
	}
	TRACE_LOG_GREEN_ROBOT_END_LN;
	
	if(fault) {
		operationMode = allMotors[motorID].readParameter(OPERATION_MODE);
		TRACE_ERROR_ROBOT_LN("Fault at motor %d. Control type: %d.", motorID, operationMode);
	}
	IPMBufferStatus = allMotors[motorID].readParameter(INTERPOLATION_MODE_STATUS);
	
	// There are warnings or errors for IPM
	if(id == 0 && ((IPMBufferStatus & 0b111100001111) != 0)) {
		if((IPMBufferStatus & 0b111100000000) != 0) {
			TRACE_ERROR_ROBOT_LN("Motor ID: %d, Interpolation Buffer Status: [0x%x]", motorID, IPMBufferStatus);
		} else {
			TRACE_LOG_BLUE_ROBOT_LN("Motor ID: %d, Interpolation Buffer Status: [0x%x]", motorID, IPMBufferStatus);
		}
		if((IPMBufferStatus & 0b1) != 0) TRACE_LOG_BLUE_ROBOT_LN("  Interpolation Buffer Warning: Underflow Warning");
		if((IPMBufferStatus & 0b10) != 0) TRACE_LOG_BLUE_ROBOT_LN("  Interpolation Buffer Warning: Overflow Warning");
		if((IPMBufferStatus & 0b100) != 0) TRACE_LOG_BLUE_ROBOT_LN("  Interpolation Buffer Warning: Velocity Warning");
		if((IPMBufferStatus & 0b1000) != 0) TRACE_LOG_BLUE_ROBOT_LN("  Interpolation Buffer Warning: Acceleration Warning");
		if((IPMBufferStatus & 0b100000000) != 0) TRACE_ERROR_ROBOT_LN("  Interpolation Buffer Error: Underflow Error");
		if((IPMBufferStatus & 0b1000000000) != 0) TRACE_ERROR_ROBOT_LN("  Interpolation Buffer Error: Overflow Error");
		if((IPMBufferStatus & 0b10000000000) != 0) TRACE_ERROR_ROBOT_LN("  Interpolation Buffer Error: Velocity Error");
		if((IPMBufferStatus & 0b100000000000) != 0) TRACE_ERROR_ROBOT_LN("  Interpolation Buffer Error: Acceleration Error");
	}
	
	for(i = 0; i < PDOTXNumEntry[id]; i++) {
		TRACE_LOG_PURP_ROBOT_LN("Updated Motor: %d Param ID: %d", motorID, PDOTXMotorParamMap[id][i]);
	}
}

/*************** PDO TX handling, highly dependent on PDO settings ******************/

bool Robot::checkHealth(int motorID) {
	return allMotors[motorID].isAlive();
}

int Robot::setCurrentReferencePDO(int motorID, int value) {
	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
	return writeObjectPDO(motorID, DEMAND_CURRENT_PDO_REF, (long) value, DEMAND_CURRENT_DATA_LENGTH);
}

int Robot::setPositionReferencePDO(int motorID, int value) {
	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
	return writeObjectPDO(motorID, DEMAND_POSITION_PDO_REF, (long) value, DEMAND_POSITION_DATA_LENGTH);
}

int Robot::startControlPDO(int motorID[], int numMotor, int controlType) {
	int i, error;
	long PDOCommand;
	
	if(controlType == OPERATION_MODE_INTERPOLATED_POSITION) {
		PDOCommand = CONTROL_WORD_START_INTERPOLATED + (controlType << 16);
	} else {
		PDOCommand = CONTROL_WORD_START + (controlType << 16);
	}
	error = initializeMotorForControl(motorID, numMotor);
	if(error != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error initializing motors for control %d.", controlType);
		return error;
	}

	for(i = 0; i < numMotor; i++) {
		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) continue;
		if(allMotors[motorID[i]].getState() != OPERATIONAL) continue;
		
		writeObjectPDO(motorID[i], CONTROL_WORD_MODE_OF_OPERATION_PDO_REF, PDOCommand, CONTROL_WORD_MODE_OF_OPERATION_DATA_LENGTH);
	}	
	
	//Ensures that the command is sent before sending the sync command. Should not have problems but it is better to be safe
	usleep(100000);

	return sync();
}	


int Robot::startControlPDO(int motorID, int controlType) {
	int motorIDArray[1];
	motorIDArray[0] = motorID;
	
 	return startControlPDO(motorIDArray, 1, controlType);
}

int Robot::setMotorControlModePDO(int motorID, int controlType) {
int error;
	long PDOCommand;
	PDOCommand = CONTROL_WORD_PRE_START + (controlType << 16);

	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
	error = writeObjectPDO(motorID, CONTROL_WORD_MODE_OF_OPERATION_PDO_REF, PDOCommand, CONTROL_WORD_MODE_OF_OPERATION_DATA_LENGTH);
	if(error != SEND_SUCCESSFUL) return error;
	//Ensures that the command is sent before sending the sync command. Should not have problems but it is better to be safe
	usleep(100000);
	
 	return sync();
}

int Robot::stopControlPDO(int motorID, int controlType) {
	int error;
	long PDOCommand;
	PDOCommand = CONTROL_WORD_QUICK_STOP + (controlType << 16);	
	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
	error = writeObjectPDO(motorID, CONTROL_WORD_MODE_OF_OPERATION_PDO_REF, PDOCommand, CONTROL_WORD_MODE_OF_OPERATION_DATA_LENGTH);
	if(error != SEND_SUCCESSFUL) return error;
	//Ensures that the command is sent before sending the sync command. Should not have problems but it is better to be safe
	usleep(100000);
	
	return sync();
}

int Robot::updateAnalogInputPDO(int motorID) {
	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
	allMotors[motorID].requestParameter(ANALOG_1_MV);
	allMotors[motorID].requestParameter(ANALOG_2_MV);
	
	return readObjectPDO(motorID, ANALOG_INPUTS_PDO_REF);
}

int Robot::updateActualPositionPDO(int motorID) {
	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
	allMotors[motorID].requestParameter(ACTUAL_POSITION);

	return readObjectPDO(motorID, ACTUAL_POSITION_PDO_REF);
}

int Robot::updateActualVelocityPDO(int motorID) {
	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
	allMotors[motorID].requestParameter(ACTUAL_VELOCITY);

	return readObjectPDO(motorID, ACTUAL_VELOCITY_PDO_REF);
}


int Robot::updateBufferSizePDO(int motorID) {
	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
	allMotors[motorID].requestParameter(BUFFER_REMAINING_MEMORY);
	allMotors[motorID].requestParameter(BUFFER_CURRENT_SIZE);

	return readObjectPDO(motorID, REMAINING_BUFFER_SIZE_PDO_REF);
}


int Robot::readTrajectory(int motorID, std::string fileName, bool offset, bool usePDO) {
	dataArrayWrapper dArray;	
	int i, offsetPos, error;
	
	dArray = loadDataFileArray(fileName.c_str(), NUM_PARAMS_PER_WAYPOINT);
	
	if(dArray.row == 0) {
		free(dArray.dataArray);
		return ERROR_READING_FILE;
	}
	
	if(offset) {
		error = updateMotorParam(motorID, ACTUAL_POSITION, usePDO);
		if(error != READ_SUCCESSFUL) return error;
		offsetPos = allMotors[motorID].readParameter(ACTUAL_POSITION);
	} else {
		offsetPos = 0;
	}	
	
	if(dArray.row > MAX_BUFFER_SIZE) {	
		free(dArray.dataArray);
		return ERROR_TOO_MANY_WAYPOINTS;
	}
	
	for(i = 0; i < dArray.row;i++) {
		addPositionWaypoint(motorID, (int) dArray.dataArray[i*NUM_PARAMS_PER_WAYPOINT], (int) dArray.dataArray[i*NUM_PARAMS_PER_WAYPOINT + 1], (int) dArray.dataArray[i*NUM_PARAMS_PER_WAYPOINT + 2] + offsetPos, usePDO);
	}
	
	free(dArray.dataArray);
	return ADDED_WAYPOINTS;
}


int Robot::readTrajectoryMultiple(int motorID[], std::string fileName[], int numMotors, bool offset, bool usePDO) {
	dataArrayWrapper dArray;	
	int i, j;
	int offsetPos[TOTAL_MOTORS_IN_ROBOT];
	
	if(numMotors > TOTAL_MOTORS_IN_ROBOT) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;
	
	if(offset) {
		if(updateMotorParamMultiple(motorID, numMotors, ACTUAL_POSITION, usePDO) == ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;
		for(i = 0; i < numMotors; i++) offsetPos[i] = allMotors[motorID[i]].readParameter(ACTUAL_POSITION);
	} else {
		for(i = 0; i < numMotors; i++) offsetPos[i] = 0;
	}	

	for(i = 0; i < numMotors; i++) {
		dArray = loadDataFileArray(fileName[i].c_str(), NUM_PARAMS_PER_WAYPOINT);
		if(dArray.row == 0) {
			free(dArray.dataArray);
			// Reset all loaded buffers
			for(j = 0; j < i; j++) {
				resetInterpolatedBuffer(motorID[j]);
			}
			return ERROR_READING_FILE;
		}
		if(dArray.row > MAX_BUFFER_SIZE) {
			free(dArray.dataArray);
			// Reset all loaded buffers
			for(j = 0; j < i; j++) {
				resetInterpolatedBuffer(motorID[j]);
			}
			return ERROR_TOO_MANY_WAYPOINTS;
		}
		
		for(j = 0; j < dArray.row;j++) {
			addPositionWaypoint(motorID[i], (int) dArray.dataArray[j*NUM_PARAMS_PER_WAYPOINT], (int) dArray.dataArray[j*NUM_PARAMS_PER_WAYPOINT + 1], (int) dArray.dataArray[j*NUM_PARAMS_PER_WAYPOINT + 2] + offsetPos[i], usePDO);
		}
		free(dArray.dataArray);
	}
	return ADDED_WAYPOINTS;
}

int Robot::updateAnalogInputMultiplePDO(int motorID[], int numMotors) {
	int i, error;
	
	if(numMotors > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	
	for(i = 0; i < numMotors; i++) {

		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
		if(allMotors[motorID[i]].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
		allMotors[motorID[i]].requestParameter(ANALOG_1_MV);
		allMotors[motorID[i]].requestParameter(ANALOG_2_MV);
		allMotors[motorID[i]].requestParameter(ACTUAL_FORCE_MV);
		allMotors[motorID[i]].requestParameter(ACTUAL_FORCE_N);
		error = readObjectPDO(motorID[i], ANALOG_INPUTS_PDO_REF);
		if(error != SEND_SUCCESSFUL) return error;
	}
	return SEND_SUCCESSFUL;
}

int Robot::updateActualPositionMultiplePDO(int motorID[], int numMotors) {
	int i, error;
	
	if(numMotors > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	
	for(i = 0; i < numMotors; i++) {

		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
		if(allMotors[motorID[i]].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
		allMotors[motorID[i]].requestParameter(ACTUAL_POSITION);
		error = readObjectPDO(motorID[i], ACTUAL_POSITION_PDO_REF);
		if(error != SEND_SUCCESSFUL) return error;
	}
	return SEND_SUCCESSFUL;
}

int Robot::updateActualVelocityMultiplePDO(int motorID[], int numMotors) {
	int i, error;
	
	if(numMotors > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	
	for(i = 0; i < numMotors; i++) {
		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
		if(allMotors[motorID[i]].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;
		allMotors[motorID[i]].requestParameter(ACTUAL_VELOCITY);
		error = readObjectPDO(motorID[i], ACTUAL_VELOCITY_PDO_REF);
		if(error != SEND_SUCCESSFUL) return error;
	}
	
	return SEND_SUCCESSFUL;
}

int Robot::updateBufferSizeMultiplePDO(int motorID[], int numMotors) {
	int i, error;
	
	if(numMotors > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	
	for(i = 0; i < numMotors; i++) {

		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) {
			TRACE_ERROR_ROBOT_LN("Error Motor %d: Invalid ID.", motorID[i]);
			return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
		}
		if(allMotors[motorID[i]].getState() != OPERATIONAL) {
			TRACE_ERROR_ROBOT_LN("Error Motor %d: Invalid Operational State: %d.", motorID[i], allMotors[motorID[i]].getState());
			return ERROR_MOTOR_NOT_OPERATIONAL;
		}
		allMotors[motorID[i]].requestParameter(BUFFER_REMAINING_MEMORY);
		allMotors[motorID[i]].requestParameter(BUFFER_CURRENT_SIZE);
		error = readObjectPDO(motorID[i], REMAINING_BUFFER_SIZE_PDO_REF);
		
		if(error != SEND_SUCCESSFUL) {
			TRACE_ERROR_ROBOT_LN("Error Motor %d: PDO sending error %d.", motorID[i], error);
			return error;
		}
		TRACE_LOG_GREEN_ROBOT_LN("Motor %d: PDO Update Buffer Memory.", motorID[i]);
	}
	return SEND_SUCCESSFUL;
}


int Robot::updateMotorParamMultiple(int motorID[], int numMotors, int paramID, bool usePDO) {
	int i;
	bool validPDOParam;
	vector<SDOElement> blockList;

	if(numMotors > TOTAL_MOTORS_IN_ROBOT) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;
	if(usePDO) {
		validPDOParam = false;
		switch(paramID) {
			case ACTUAL_POSITION:
				if(updateActualPositionMultiplePDO(motorID, numMotors) == SEND_SUCCESSFUL) {
					for(i = 0; i < numMotors; i++) blockList.push_back(SDOElement(motorID[i], ACTUAL_POSITION, 0, 0, 0, 0));
					validPDOParam = true;
				} else {
					TRACE_ERROR_ROBOT_LN("PDO Sending Error");
					return ERROR_SENDING_PDO_ERROR;
				}
				break;
			case ACTUAL_VELOCITY:
				if(updateActualVelocityMultiplePDO(motorID, numMotors) == SEND_SUCCESSFUL) {
					for(i = 0; i < numMotors; i++) blockList.push_back(SDOElement(motorID[i], ACTUAL_VELOCITY, 0, 0, 0, 0));
					validPDOParam = true;
				} else {
					TRACE_ERROR_ROBOT_LN("PDO Sending Error");
					return ERROR_SENDING_PDO_ERROR;
				}
				break;
			case ACTUAL_FORCE_MV:
			case ACTUAL_FORCE_N:
			case ANALOG_1_MV:
			case ANALOG_2_MV:
				if(updateAnalogInputMultiplePDO(motorID, numMotors) == SEND_SUCCESSFUL) {
					for(i = 0; i < numMotors; i++) blockList.push_back(SDOElement(motorID[i], ANALOG_1_MV, 0, 0, 0, 0));
					validPDOParam = true;
				} else {
					TRACE_ERROR_ROBOT_LN("PDO Sending Error");
					return ERROR_SENDING_PDO_ERROR;
				}
				break;
			case BUFFER_REMAINING_MEMORY:
			case BUFFER_CURRENT_SIZE:
				if(updateBufferSizeMultiplePDO(motorID, numMotors) == SEND_SUCCESSFUL) {
					for(i = 0; i < numMotors; i++) blockList.push_back(SDOElement(motorID[i], BUFFER_REMAINING_MEMORY, 0, 0, 0, 0));
					validPDOParam = true;
				} else {
					TRACE_ERROR_ROBOT_LN("PDO Sending Error");
					return ERROR_SENDING_PDO_ERROR;
				}
				break;
		}
		if(validPDOParam == false) return ERROR_INVALID_PDO_PARAM;
		multipleBlocking(blockList, READ_TRAJECTORY_MULTIPLE_OFFSET_BLOCK_MS);
	} else {
		for(i = 0; i < numMotors; i++) { 
			if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;
			updateParamMultiple(motorID[i], paramID);
		}
		readSDOSequence(READ_TRAJECTORY_MULTIPLE_OFFSET_BLOCK_MS);
	}
	return READ_SUCCESSFUL;
}

// Currently working with PDO only
int Robot::readTrajectoryMultipleInterpolate(int motorID[], std::string fileName[], int maxVelocity[], int numMotors, bool usePDO) {
	dataArrayWrapper dArray;	
	int i, maxVelAdjusted[TOTAL_MOTORS_IN_ROBOT];
	int curPos[TOTAL_MOTORS_IN_ROBOT], initialPos[TOTAL_MOTORS_IN_ROBOT], initialVel[TOTAL_MOTORS_IN_ROBOT];
	int curVel[TOTAL_MOTORS_IN_ROBOT], posDiff[TOTAL_MOTORS_IN_ROBOT];
	double maxTime, timeTaken;
	Trajectory tra, allTrajectory[TOTAL_MOTORS_IN_ROBOT];

	// Read in current positions
	if(updateMotorParamMultiple(motorID, numMotors, ACTUAL_POSITION, usePDO) == ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;

	// Read in all files
	// Check initial position with current position
	// Identify the motor that takes the longest time (Time based on distance/0.5*maxVelocity to account for acceleration and deceleration)
	maxTime = 0;
	for(i = 0; i < numMotors; i++) {
		curPos[i] = allMotors[motorID[i]].readParameter(ACTUAL_POSITION);
		curVel[i] = allMotors[motorID[i]].readParameter(ACTUAL_VELOCITY);
		dArray = loadDataFileArray(fileName[i].c_str(), NUM_PARAMS_PER_WAYPOINT);
		if(dArray.row == 0) {
			free(dArray.dataArray);
			return ERROR_READING_FILE;
		}
		initialVel[i] = dArray.dataArray[1];
		initialPos[i] = dArray.dataArray[2];
		posDiff[i] = initialPos[i] - curPos[i];
		
		// velocity in RPM, default to 90% of max motor velocity if given velocity is higher than motor's 90% max velocity
		if(maxVelocity[i] > MAX_VELOCITY_SCALE*allMotors[motorID[i]].maxVelocity) {
			maxVelAdjusted[i] = MAX_VELOCITY_SCALE*allMotors[motorID[i]].maxVelocity;
		} else {
			maxVelAdjusted[i] = maxVelocity[i];
		}
		
		// Position difference in qc, velocity in RPM, conversion from RPM to qc per second
		// Approximated time taken based on triangular velocity profile
		timeTaken = norm(((double)2.0*posDiff[i])/((double)maxVelAdjusted[i]*allMotors[motorID[i]].countsPerTurn/60.0));
		if(timeTaken > 0.255*(MAX_BUFFER_SIZE - 1)) {
			TRACE_ERROR_ROBOT_LN("Error: Time taken exceeds maximum time.");
		}
		
		// Rounding up to multiples of time steps
		timeTaken = ((double) (1 +  ((int) (timeTaken*1000)/MAX_TRAJECTORY_TIME_STEP))*MAX_TRAJECTORY_TIME_STEP)/1000.0;
		
		TRACE_LOG_PURP_ROBOT_LN("Time Taken: %lf", timeTaken); 
		TRACE_LOG_PURP_ROBOT_LN("Max Velocity Adjusted: %d", maxVelAdjusted[i]);
		
		// Confirm with spline computation, increase time taken until trajectory is feasible
		tra = allMotors[motorID[i]].cubicSpline(curPos[i], curVel[i], initialPos[i], initialVel[i], timeTaken, MAX_TRAJECTORY_TIME_STEP, allMotors[motorID[i]].countsPerTurn, maxVelAdjusted[i], allMotors[motorID[i]].maxAcceleration);
		
		while(tra.numElements <= 0) {
			timeTaken = timeTaken*1.1;
			timeTaken = ((double) (1 +  ((int) (timeTaken*1000)/MAX_TRAJECTORY_TIME_STEP))*MAX_TRAJECTORY_TIME_STEP)/1000.0;
			if(timeTaken > MAX_TRAJECTORY_TIME_STEP*(MAX_BUFFER_SIZE - 1)/1000.0) {
				TRACE_ERROR_ROBOT_LN("Error: Time taken exceeds maximum time.");
			}
			tra = allMotors[motorID[i]].cubicSpline(curPos[i], curVel[i], initialPos[i], initialVel[i], timeTaken, MAX_TRAJECTORY_TIME_STEP, allMotors[motorID[i]].countsPerTurn, maxVelAdjusted[i], allMotors[motorID[i]].maxAcceleration);
		}
		TRACE_LOG_PURP_ROBOT_LN("Trajectory %d, Num Element %d", motorID[i], tra.numElements);		
		
		if(timeTaken > maxTime) maxTime = timeTaken;
		TRACE_LOG_PURP_ROBOT_LN("Time Taken %lf, Max Time %lf", timeTaken, maxTime);	
		free(dArray.dataArray);
	}
	
	// Calculate all trajectories based on the one that takes the longest time
	for(i = 0; i < numMotors; i++) {
		allTrajectory[i] = allMotors[motorID[i]].cubicSpline(curPos[i], curVel[i], initialPos[i], initialVel[i], maxTime, MAX_TRAJECTORY_TIME_STEP, allMotors[motorID[i]].countsPerTurn, maxVelAdjusted[i], allMotors[motorID[i]].maxAcceleration);

		allTrajectory[i].numElements = allTrajectory[i].numElements - 1;
		TRACE_LOG_PURP_ROBOT_LN("Trajectory %d, Num Element %d", motorID[i], allTrajectory[i].numElements);
		for(int j = 0; j < allTrajectory[i].numElements; j++) {
			TRACE_LOG_PURP_ROBOT_LN("  Trajectory %d, Element %d: [%d] [%d] [%d]", motorID[i], j+1, allTrajectory[i].t[j].time, allTrajectory[i].t[j].velocity, allTrajectory[i].t[j].position);
		}
		
		//BREAK_POINT("Updated Trajectory.");
	}		
	
	
	
	
	//return 0;
	
	uploadTrajectoryMultiple(motorID, allTrajectory, numMotors, false, usePDO);
	
	// Upload the data points from the files as the current points are consumed
	// Wait until all the points from the files are loaded into EPOS2
	return readTrajectoryExtendedMultiple(motorID, fileName, numMotors, false, false, usePDO);
}


// Currently working with PDO only
int Robot::readTrajectoryExtended(int motorID, std::string fileName, bool isStarted, bool offset, bool usePDO) {
	dataArrayWrapper dArray;	
	int i, remainingBufferSize, curIndex;
	int offsetPos, error;
	PauseTimer waitTimer(EXTENDED_LOADING_SAMPLE_RATE_NS);

	if(offset) {
		error = updateMotorParam(motorID, ACTUAL_POSITION, usePDO);
		if(error != READ_SUCCESSFUL) return error;
		offsetPos = allMotors[motorID].readParameter(ACTUAL_POSITION);
	} else {
		offsetPos = 0;
	}	
	
	dArray = loadDataFileArray(fileName.c_str(), NUM_PARAMS_PER_WAYPOINT);
	if(dArray.row == 0) {
		free(dArray.dataArray);
		return ERROR_READING_FILE;
	}
	curIndex = 0;
	
	// Get current buffer size based on PDO or SDO
	// Add points until buffer is full
	if(updateMotorParam(motorID, BUFFER_CURRENT_SIZE, usePDO) == READ_SUCCESSFUL) {
		
		if(curIndex < dArray.row) {
			remainingBufferSize = allMotors[motorID].readParameter(BUFFER_REMAINING_MEMORY);
			for(i = 0; i < (remainingBufferSize - BUFFER_ALLOWANCE) && curIndex < dArray.row; i++, curIndex++) {
				addPositionWaypoint(motorID, (int) dArray.dataArray[curIndex*NUM_PARAMS_PER_WAYPOINT], (int) dArray.dataArray[curIndex*NUM_PARAMS_PER_WAYPOINT + 1], (int) dArray.dataArray[curIndex*NUM_PARAMS_PER_WAYPOINT + 2] + offsetPos, usePDO);
			}
		}
	}
	
	// Start synchronized motion
	if(isStarted == false) startControlPDO(motorID, OPERATION_MODE_INTERPOLATED_POSITION);
	
	// Continue loading points as they are consumed
	while(true) {
		if(updateMotorParam(motorID, BUFFER_CURRENT_SIZE, usePDO) == READ_SUCCESSFUL) {
			if(curIndex < dArray.row) {
				remainingBufferSize = allMotors[motorID].readParameter(BUFFER_REMAINING_MEMORY);
				for(i = 0; i < (remainingBufferSize - BUFFER_ALLOWANCE) && curIndex < dArray.row; i++, curIndex++) {
					addPositionWaypoint(motorID, (int) dArray.dataArray[curIndex*NUM_PARAMS_PER_WAYPOINT], (int) dArray.dataArray[curIndex*NUM_PARAMS_PER_WAYPOINT + 1], (int) dArray.dataArray[curIndex*NUM_PARAMS_PER_WAYPOINT + 2] + offsetPos, usePDO);
				}
				if(curIndex >= dArray.row) break;
			}
		}
		waitTimer.wait();
	}
	
	free(dArray.dataArray);

	return ADDED_WAYPOINTS;
}

// Currently working with PDO only
int Robot::readTrajectoryExtendedMultiple(int motorID[], std::string fileName[], int numMotors, bool isStarted, bool offset, bool usePDO) {
	dataArrayWrapper dArray[TOTAL_MOTORS_IN_ROBOT];	
	int i, j, k, remainingBufferSize, curIndex[TOTAL_MOTORS_IN_ROBOT];
	int offsetPos[TOTAL_MOTORS_IN_ROBOT];
	int curBufferSize, position, velocity, time;
	bool loadingIncomplete;
	PauseTimer waitTimer(EXTENDED_LOADING_SAMPLE_RATE_NS);

	if(numMotors > TOTAL_MOTORS_IN_ROBOT) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;

	if(offset) {
		if(updateMotorParamMultiple(motorID, numMotors, ACTUAL_POSITION, usePDO) == ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;
		for(i = 0; i < numMotors; i++) offsetPos[i] = allMotors[motorID[i]].readParameter(ACTUAL_POSITION);
	} else {
		for(i = 0; i < numMotors; i++) offsetPos[i] = 0;
	}	
	
	// Load in all the data files
	
	for(i = 0; i < numMotors; i++) {
		dArray[i] = loadDataFileArray(fileName[i].c_str(), NUM_PARAMS_PER_WAYPOINT);
		if(dArray[i].row == 0) {
			// Free all memory (regardless of loading success or failure, memory is always allocated to dataArray)
			for(j = 0; j <= i; j++)	free(dArray[j].dataArray);
			return ERROR_READING_FILE;
		}
		if(dArray[i].dataArray[0] == 0) {
			curIndex[i] = 1;
		} else {
			curIndex[i] = 0;
		}
	}	
	
	// Get current buffer size based on PDO or SDO
	// Add points until buffer is full
	if(updateMotorParamMultiple(motorID, numMotors, BUFFER_CURRENT_SIZE, usePDO) == READ_SUCCESSFUL) {	
		for(i = 0; i < numMotors; i++) {
			if(curIndex[i] < dArray[i].row) {
				remainingBufferSize = allMotors[motorID[i]].readParameter(BUFFER_REMAINING_MEMORY);
				for(j = 0; j < (remainingBufferSize - BUFFER_ALLOWANCE) && curIndex[i] < dArray[i].row; j++, curIndex[i]++) {
					addPositionWaypoint(motorID[i], (int) dArray[i].dataArray[curIndex[i]*NUM_PARAMS_PER_WAYPOINT], (int) dArray[i].dataArray[curIndex[i]*NUM_PARAMS_PER_WAYPOINT + 1], (int) dArray[i].dataArray[curIndex[i]*NUM_PARAMS_PER_WAYPOINT + 2] + offsetPos[i], usePDO);
				}
			}
		}
	}
	
	// Start synchronized motion
	if(isStarted == false) startControlPDO(motorID, numMotors, OPERATION_MODE_INTERPOLATED_POSITION);
	
	// Continue loading points as they are consumed
	loadingIncomplete = true;
	while(loadingIncomplete) {
		if(updateMotorParamMultiple(motorID, numMotors, BUFFER_CURRENT_SIZE, usePDO) == READ_SUCCESSFUL) {	
			loadingIncomplete = false;
			for(i = 0; i < numMotors; i++) {
				// Check to make sure that any of the motors are in stop mode
				if(allMotors[motorID[i]].getState() == STOP) {
					// Record the possible current waypoints in the buffer for all motors
					// If the waypoint is based on others before this then
					
					TRACE_ERROR_ROBOT_LN("Possible waypoints causing errors:");
					for(j = 0; j < numMotors; j++) {
						curBufferSize = allMotors[motorID[j]].readParameter(BUFFER_CURRENT_SIZE);
						if(curIndex[j] <= curBufferSize + 1) {
							TRACE_ERROR_ROBOT_LN("  Possible error waypoint before current list for motor %d: Buffer Size: %d Current Index: %d", j, curBufferSize, curIndex[j]);
						} else {
							for(k = 0; k < 3; k++) {
								time =  dArray[j].dataArray[(curIndex[j] - curBufferSize - 1 + k)*NUM_PARAMS_PER_WAYPOINT];
								velocity =  dArray[j].dataArray[(curIndex[j] - curBufferSize - 1 + k)*NUM_PARAMS_PER_WAYPOINT + 1];
								position =  dArray[j].dataArray[(curIndex[j] - curBufferSize - 1 + k)*NUM_PARAMS_PER_WAYPOINT + 2] + offsetPos[j];
								TRACE_ERROR_ROBOT_LN("  Motor %d: [%d %d %d]", j, time, velocity, position);
							}
						}
						TRACE_ERROR_ROBOT_LN("Possible waypoints causing errors:");
					}
					for(j = 0; j < numMotors; j++) free(dArray[j].dataArray);
					return ERROR_IPM_FAULT;
				}
				if(curIndex[i] < dArray[i].row) {
					remainingBufferSize = allMotors[motorID[i]].readParameter(BUFFER_REMAINING_MEMORY);
					for(j = 0; j < (remainingBufferSize - BUFFER_ALLOWANCE) && curIndex[i] < dArray[i].row; j++, curIndex[i]++) {
						addPositionWaypoint(motorID[i], (int) dArray[i].dataArray[curIndex[i]*NUM_PARAMS_PER_WAYPOINT], (int) dArray[i].dataArray[curIndex[i]*NUM_PARAMS_PER_WAYPOINT + 1], (int) dArray[i].dataArray[curIndex[i]*NUM_PARAMS_PER_WAYPOINT + 2] + offsetPos[i], usePDO);
					}
					if(curIndex[i] < dArray[i].row) loadingIncomplete = true;
				}
			}
		}
		waitTimer.wait();
	}
	
	for(i = 0; i < numMotors; i++) free(dArray[i].dataArray);

	return ADDED_WAYPOINTS;
}


int Robot::updateMotorParam(int motorID, int paramID, bool usePDO) {
	bool validPDOParam;
	if(usePDO) {
		validPDOParam = false;
		switch(paramID) {
			case ACTUAL_POSITION:
				updateActualPositionPDO(motorID);
				validPDOParam = true;
				break;
			case ACTUAL_VELOCITY:
				updateActualVelocityPDO(motorID);
				validPDOParam = true;
				break;
			case ACTUAL_FORCE_MV:
			case ACTUAL_FORCE_N:
			case ANALOG_1_MV:
			case ANALOG_2_MV:
				updateAnalogInputPDO(motorID);
				validPDOParam = true;
				break;
			case BUFFER_REMAINING_MEMORY:
			case BUFFER_CURRENT_SIZE:
				updateBufferSizePDO(motorID);
				validPDOParam = true;
				break;
		}
		if(allMotors[motorID].getState() != OPERATIONAL) {
			TRACE_ERROR_ROBOT_LN("Error: Wrong state for PDO request Motor ID: %d", motorID);
			return ERROR_MOTOR_NOT_OPERATIONAL;
		}
		
		if(validPDOParam) {
			return singleBlocking(motorID, paramID, READ_TRAJECTORY_OFFSET_BLOCK_MS);
		} else {
			return ERROR_INVALID_PDO_PARAM;
		}
	} else {
		return updateParam(motorID, paramID, READ_TRAJECTORY_OFFSET_BLOCK_MS);
	}
}

int Robot::uploadTrajectory(int motorID, Trajectory tra, bool offset, bool usePDO) {
	int i, offsetPos, error;
	
	if(offset) {
		error = updateMotorParam(motorID, ACTUAL_POSITION, usePDO);
		if(error != READ_SUCCESSFUL) return error;
		offsetPos = allMotors[motorID].readParameter(ACTUAL_POSITION);
	} else {
		offsetPos = 0;
	}	
	
	for(i = 0; i < tra.numElements; i++) {
		addPositionWaypoint(motorID, (int) tra.t[i].time, (int) tra.t[i].velocity, (int) tra.t[i].position + offsetPos, usePDO);
	}
	
	return ADDED_WAYPOINTS;
}
int Robot::uploadTrajectoryMultiple(int motorID[], Trajectory tra[], int numMotors, bool offset, bool usePDO) {
	int i, j;
	int offsetPos[TOTAL_MOTORS_IN_ROBOT];
	
	if(numMotors > TOTAL_MOTORS_IN_ROBOT) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;
	
	if(offset) {
		if(updateMotorParamMultiple(motorID, numMotors, ACTUAL_POSITION, usePDO) == ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED) return ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED;
		for(i = 0; i < numMotors; i++) offsetPos[i] = allMotors[motorID[i]].readParameter(ACTUAL_POSITION);
	} else {
		for(i = 0; i < numMotors; i++) offsetPos[i] = 0;
	}	

	for(i = 0; i < numMotors; i++) {
		for(j = 0; j < tra[i].numElements;j++) {
			addPositionWaypoint(motorID[i], (int) tra[i].t[j].time, (int) tra[i].t[j].velocity, (int) tra[i].t[j].position + offsetPos[i], usePDO);
		}
	}
	return ADDED_WAYPOINTS;
}




void Robot::updateAnalogInputPDO() {
	int i;
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		if(allMotors[i].getState() != OPERATIONAL) continue;
		allMotors[i].requestParameter(ANALOG_1_MV);
		allMotors[i].requestParameter(ANALOG_2_MV);
		readObjectPDO(i, ANALOG_INPUTS_PDO_REF);	
	}
}

void Robot::updateActualPositionPDO() {
	int i;
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		if(allMotors[i].getState() != OPERATIONAL) continue;
		allMotors[i].requestParameter(ACTUAL_POSITION);
		readObjectPDO(i, ACTUAL_POSITION_PDO_REF);	
	}
}

void Robot::updateBufferSizePDO() {
	int i;
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		if(allMotors[i].getState() != OPERATIONAL) continue;
		allMotors[i].requestParameter(BUFFER_REMAINING_MEMORY);
		allMotors[i].requestParameter(BUFFER_CURRENT_SIZE);
		readObjectPDO(i, REMAINING_BUFFER_SIZE_PDO_REF);	
	}
}

void Robot::setMotorState(int motorID, int state) {
	int i;
	if(motorID == 0) {
		for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
			allMotors[i].setState(state);
		}
	} else {
		if(motorID <= TOTAL_MOTORS_IN_ROBOT) allMotors[motorID].setState(state);
	}
}


// NMT commands, set nodeID = 0 for all nodes
int Robot::enterPreOperational(int motorID) {
	setMotorState(motorID, PREOPERATIONAL);
	return sendNMTCommand(ENTER_PREOPERATIONAL_CS, motorID);
}
		
// Enter operational state
int Robot::startNode(int motorID) {
	setMotorState(motorID, OPERATIONAL);
	return sendNMTCommand(START_REMOTE_NODE_CS, motorID);
}

// Enter stopped state
int Robot::stopNode(int motorID) {
	setMotorState(motorID, STOP);
	return sendNMTCommand(STOP_REMOTE_NODE_CS, motorID);
}

// Enter initialisation state
int Robot::resetNode(int motorID) {
	setMotorState(motorID, BOOTUP);
	return sendNMTCommand(RESET_NODE_CS, motorID);
}

// Enter initialisation state
int Robot::resetCommunication(int motorID) {
	setMotorState(motorID, BOOTUP);
	return sendNMTCommand(RESET_COMMUNICATION_CS, motorID);
}

int Robot::setMaxVelocityAndAcceleration(int motorID, int maxVelocity, int maxAcceleration) {

	writeParamMultiple(motorID, MAX_PROFILE_VELOCITY, maxVelocity);
	writeParamMultiple(motorID, MAX_ACCELERATION, maxAcceleration);
	
	return writeSDOSequence(100);
}


int Robot::initializePDORXMapping(int motorID) {
	
	int error;

	// Need to set PDO mapping size to 0 before writing new PDO mapping
	// Ensure that the size has been set to 0 before writting
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_RX_MAPPING_SIZE, 0);
		writeParamMultiple(motorID, PDO2_RX_MAPPING_SIZE, 0);
		writeParamMultiple(motorID, PDO3_RX_MAPPING_SIZE, 0);
		writeParamMultiple(motorID, PDO4_RX_MAPPING_SIZE, 0);
		error = writeSDOSequence(300);
	}

	// Ensure that all the mapping is done
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_RX_MAPPING_ENTRY1, PDO1_ENTRY1_RX_MAP);
		writeParamMultiple(motorID, PDO1_RX_MAPPING_ENTRY2, PDO1_ENTRY2_RX_MAP);
		writeParamMultiple(motorID, PDO2_RX_MAPPING_ENTRY1, PDO2_ENTRY1_RX_MAP);
		writeParamMultiple(motorID, PDO3_RX_MAPPING_ENTRY1, PDO3_ENTRY1_RX_MAP);
		writeParamMultiple(motorID, PDO4_RX_MAPPING_ENTRY1, PDO4_ENTRY1_RX_MAP);
		error = writeSDOSequence(300);
	}

	// Need to set PDO mapping size to 0 before writing new PDO mapping
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_RX_MAPPING_SIZE, PDO1_RX_SIZE);
		writeParamMultiple(motorID, PDO2_RX_MAPPING_SIZE, PDO2_RX_SIZE);
		writeParamMultiple(motorID, PDO3_RX_MAPPING_SIZE, PDO3_RX_SIZE);
		writeParamMultiple(motorID, PDO4_RX_MAPPING_SIZE, PDO4_RX_SIZE);
		error = writeSDOSequence(300);
	}

	// Setting to SYNC/ASYNC
	// In SYNC mode, the mapped object dictionary entry will only be changed 
	// after a sync command is received. In ASYNC mode, it is updated immediately
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_RX_PARAMETER_TYPE, PDO1_RX_SYNC_ASYNC);
		writeParamMultiple(motorID, PDO2_RX_PARAMETER_TYPE, PDO2_RX_SYNC_ASYNC);
		writeParamMultiple(motorID, PDO3_RX_PARAMETER_TYPE, PDO3_RX_SYNC_ASYNC);
		writeParamMultiple(motorID, PDO4_RX_PARAMETER_TYPE, PDO4_RX_SYNC_ASYNC);
		error = writeSDOSequence(300);
	}
	
	return SEND_SUCCESSFUL;
}

int Robot::initializePDOTXMapping(int motorID) {
	
	int error;
	
	// Setting to RTR
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_TX_PARAMETER_COBID, PDO1_TX_RTR + motorID);
		writeParamMultiple(motorID, PDO2_TX_PARAMETER_COBID, PDO2_TX_RTR + motorID);
		writeParamMultiple(motorID, PDO3_TX_PARAMETER_COBID, PDO3_TX_RTR + motorID);
		writeParamMultiple(motorID, PDO4_TX_PARAMETER_COBID, PDO4_TX_RTR + motorID);
		error = writeSDOSequence(300);
	}
	
	// Setting to SYNC/ASYNC
	// In SYNC mode, the mapped object dictionary entry will transmitted 
	// after a sync command is received. In ASYNC mode, it is ignored
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_TX_PARAMETER_TYPE, PDO1_TX_SYNC_ASYNC);
		writeParamMultiple(motorID, PDO2_TX_PARAMETER_TYPE, PDO2_TX_SYNC_ASYNC);
		writeParamMultiple(motorID, PDO3_TX_PARAMETER_TYPE, PDO3_TX_SYNC_ASYNC);
		writeParamMultiple(motorID, PDO4_TX_PARAMETER_TYPE, PDO4_TX_SYNC_ASYNC);
		error = writeSDOSequence(300);
	}
	
	// Setting to Inhibit time in multiples of 100us
	// Is is ignored on sync mode and async on rtr only
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_TX_PARAMETER_INHIBIT, PDO1_TX_INHIBIT);
		writeParamMultiple(motorID, PDO2_TX_PARAMETER_INHIBIT, PDO2_TX_INHIBIT);
		writeParamMultiple(motorID, PDO3_TX_PARAMETER_INHIBIT, PDO3_TX_INHIBIT);
		writeParamMultiple(motorID, PDO4_TX_PARAMETER_INHIBIT, PDO4_TX_INHIBIT);
		error = writeSDOSequence(300);
	}
			
	// Need to set PDO mapping size to 0 before writing new PDO mapping
	// Ensure that the size has been set to 0 before writting
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_TX_MAPPING_SIZE, 0);
		writeParamMultiple(motorID, PDO2_TX_MAPPING_SIZE, 0);
		writeParamMultiple(motorID, PDO3_TX_MAPPING_SIZE, 0);
		writeParamMultiple(motorID, PDO4_TX_MAPPING_SIZE, 0);
		error = writeSDOSequence(300);
	}
	// Ensure that all the mapping is done
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_TX_MAPPING_ENTRY1, PDO1_ENTRY1_TX_MAP);
		writeParamMultiple(motorID, PDO1_TX_MAPPING_ENTRY2, PDO1_ENTRY2_TX_MAP);
		writeParamMultiple(motorID, PDO1_TX_MAPPING_ENTRY3, PDO1_ENTRY3_TX_MAP);
		writeParamMultiple(motorID, PDO2_TX_MAPPING_ENTRY1, PDO2_ENTRY1_TX_MAP);
		writeParamMultiple(motorID, PDO2_TX_MAPPING_ENTRY2, PDO2_ENTRY2_TX_MAP);
		writeParamMultiple(motorID, PDO3_TX_MAPPING_ENTRY1, PDO3_ENTRY1_TX_MAP);
		writeParamMultiple(motorID, PDO3_TX_MAPPING_ENTRY2, PDO3_ENTRY2_TX_MAP);
		writeParamMultiple(motorID, PDO4_TX_MAPPING_ENTRY1, PDO4_ENTRY1_TX_MAP);
		error = writeSDOSequence(300);
	}

	// To set the correct PDO mapping size
	error = -1;
	while(error != SEND_SUCCESSFUL) {
		writeParamMultiple(motorID, PDO1_TX_MAPPING_SIZE, PDO1_TX_SIZE);
		writeParamMultiple(motorID, PDO2_TX_MAPPING_SIZE, PDO2_TX_SIZE);
		writeParamMultiple(motorID, PDO3_TX_MAPPING_SIZE, PDO3_TX_SIZE);
		writeParamMultiple(motorID, PDO4_TX_MAPPING_SIZE, PDO4_TX_SIZE);
		error = writeSDOSequence(300);
	}

	return SEND_SUCCESSFUL;
}

void Robot::handleHeartbeat(TPCANRdMsg readMsg) {
	if(readMsg.Msg.ID <= HEARTBEAT_START + TOTAL_MOTORS_IN_ROBOT) {
		if(readMsg.Msg.LEN != 1) { 
			TRACE_ERROR_ROBOT_LN("Data ID: %d, data length error, expect 1 received %d.", readMsg.Msg.ID, readMsg.Msg.LEN);
			return;
		}
		allMotors[readMsg.Msg.ID - HEARTBEAT_START].heartbeat(readMsg.Msg.DATA[0]);
		TRACE_LOG_PURP_ROBOT_LN("Heartbeat for Motor %d.", readMsg.Msg.ID - HEARTBEAT_START);
	}
}

void Robot::handleEmergency(TPCANRdMsg readMsg) {
	int motorID, i, errorIndex, bufferStatus;
	int errorCode;
	
	if(readMsg.Msg.LEN != EMERGENCY_MSG_LEN) return;
	
	errorIndex = -1;
	motorID = readMsg.Msg.ID - EMERGENCY_START + 1;
	errorCode = readMsg.Msg.DATA[0] + readMsg.Msg.DATA[1]*0x100;
	if(errorCode == NO_EMERGENCY) return;
	for(i = 0; i < TOTAL_EMERGENCY_CODES; i++) {
		if(errorCode == ErrorCodeList[i]) {
			errorIndex = i;
			break;
		}
	}
	
	if(errorIndex < 0) {
		TRACE_ERROR_ROBOT_LN("Uncertain Error Message: (0x%x) [0x%x] [0x%x] [0x%x]", readMsg.Msg.ID, readMsg.Msg.DATA[0], readMsg.Msg.DATA[1], readMsg.Msg.DATA[2]);
		return;
	}
	
	
	TRACE_ERROR_ROBOT_TIME("Emergency Message: Motor %d, Error Code: [0x%x] Error: [%s] Error Register: ", motorID, errorCode, ErrorCodeTranslate[errorIndex]);
	
	for(i = 0; i < TOTAL_ERROR_REGISTER; i++) {
		if((readMsg.Msg.DATA[2]>>i & 1) > 0) TRACE_ERROR_ROBOT("[%s] ", ErrorCodeRegister[i]);
	} 
	TRACE_ERROR_ROBOT_END_LN;

	// Interpolation Mode Error, identify the error
	// Currently unable to read, error not resolved
	if(errorCode == 0xFF0C) {
		updateParam(motorID, INTERPOLATION_MODE_STATUS);
		bufferStatus = allMotors[motorID].readParameter(INTERPOLATION_MODE_STATUS);
		TRACE_ERROR_ROBOT_LN("Interpolation Buffer Status: (%d) [0x%x]", bufferStatus, bufferStatus);
		if((bufferStatus & 0b100000000) != 0) TRACE_ERROR_ROBOT_LN("Interpolation Buffer Error: Underflow Error");
		if((bufferStatus & 0b1000000000) != 0) TRACE_ERROR_ROBOT_LN("Interpolation Buffer Error: Overflow Error");
		if((bufferStatus & 0b10000000000) != 0) TRACE_ERROR_ROBOT_LN("Interpolation Buffer Error: Velocity Error");
		if((bufferStatus & 0b100000000000) != 0) TRACE_ERROR_ROBOT_LN("Interpolation Buffer Error: Acceleration Error");
	}


	// Emergency stop everything
	//stopNode();
	
	// Collect all information from the IPM buffer
	
}


int Robot::addPositionWaypoint(int motorID, int time, int velocity, int position, bool usePDO) {
	int data[8];

	data[0] = position & 0xff;
	data[1] = (position >> 8) & 0xff;
	data[2] = (position >> 16) & 0xff;
	data[3] = (position >> 24) & 0xff;
	
	data[4] = velocity & 0xff;
	data[5] = (velocity >> 8) & 0xff;
	data[6] = (velocity >> 16) & 0xff;

	data[7] = time & 0xff;
	
	TRACE_LOG_BLUE_ROBOT_LN("Motor: %d Set position: %d, velocity: %d, time: %d.", motorID, position, velocity, time);
		
	if(usePDO) {
		return writeObjectPDO(motorID, INTERPOLATED_POSITION_BUFFER_PDO_REF, data, INTERPOLATED_POSITION_BUFFER_DATA_LENGTH);
	} else {
		return writeObjectSDONonExpediated(motorID, INTERPOLATED_POSITION_BUFFER_ADD, INTERPOLATED_POSITION_BUFFER_SUBINDEX, data, 8);
	}
}


int Robot::resetInterpolatedBuffer(int motorID) {
	int error;
	error = clearInterpolatedBuffer(motorID);
	if(error != SEND_SUCCESSFUL) return error;
	return enableInterpolatedBuffer(motorID);
}

int Robot::clearInterpolatedBuffer(int motorID) {
	int error;
	error = writeSDOSequence(motorID, BUFFER_CLEAR_ENABLE, BUFFER_CONFIG_ADD, BUFFER_CLEAR_ENABLE_SUBINDEX, CLEAR_BUFFER, BUFFER_CLEAR_ENABLE_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for CLEAR BUFFER: %d.", motorID);
	return SEND_SUCCESSFUL;
}
int Robot::enableInterpolatedBuffer(int motorID) {
	int error;

	error = writeSDOSequence(motorID, BUFFER_CLEAR_ENABLE, BUFFER_CONFIG_ADD, BUFFER_CLEAR_ENABLE_SUBINDEX, ENABLE_BUFFER, BUFFER_CLEAR_ENABLE_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for ENABLE BUFFER: %d.\n", motorID);
	return SEND_SUCCESSFUL;
}

ObjDictionaryParam Robot::motorParamIDToObjDictionaryParam(int motorParamID) {
	ObjDictionaryParam a;
	a.address = 0;
	a.subIndex = 0;
	a.dataSize = -1;
	
	if(motorParamID < 0 || motorParamID >= TOTAL_PARAMETERS) return a;
	
	a.address = addresses[motorParamID];
	a.subIndex = subIndices[motorParamID];
	a.dataSize = dataSizes[motorParamID];
	
	return a;
}

MotorParam Robot::addressAndSubIndexToMotorParam(int address, int subIndex) {
	int i;
	MotorParam a;
	a.paramID = TOTAL_PARAMETERS;
	a.isSigned = true;

	for(i = 0; i < TOTAL_PARAMETERS; i++) {
		if(address == addresses[i] && subIndex == subIndices[i]) {
			a.paramID = i;
			a.isSigned = isSignedList[i];
			break;
		}
	}

	return a;	
}

void Robot::handleSDO(TPCANRdMsg readMsg) {
	int address, subIndex, motorID, value, scs, n, e, s, i;
	MotorParam paramInfo;
	
	if(readMsg.Msg.ID <= SDO_START + TOTAL_MOTORS_IN_ROBOT) {
		if(readMsg.Msg.LEN < 8) { 
			TRACE_ERROR_ROBOT_LN("Data ID: %d, data length error, expect 8 received %d.", readMsg.Msg.ID, readMsg.Msg.LEN);
			return;
		}
		motorID = readMsg.Msg.ID - SDO_START;
		
		searchNonExpediatedMessage(readMsg);
		
		address = readMsg.Msg.DATA[1] + readMsg.Msg.DATA[2]*0x100;
		subIndex = readMsg.Msg.DATA[3];
	
		paramInfo = addressAndSubIndexToMotorParam(address, subIndex);
		TRACE_LOG_GREEN_ROBOT_LN("Extracted Parameter ID: %d", paramInfo.paramID);
	
		if(paramInfo.paramID < TOTAL_PARAMETERS) {
			scs = (0b11100000 & readMsg.Msg.DATA[0]) >> 5;
			n = (0b01100 & readMsg.Msg.DATA[0]) >> 2;
			e = (0b10 & readMsg.Msg.DATA[0]) >> 1;
			s = (0b1 & readMsg.Msg.DATA[0]);
		
			value = 0;
			if(e == 1 && s == 1) {
				for(i = 4; i < 8-n; i++) {
					value = value + (readMsg.Msg.DATA[i]<<(8*(i-4)));
				}
				if(paramInfo.isSigned) {
					value = convertActualValue(value, n);
				}
				TRACE_LOG_GREEN_ROBOT_LN("Extracted value: %d", value);
			}

			switch(scs) {
				case INITIATE_DOMAIN_UPLOAD:
					if(e == 1 && s == 1) allMotors[motorID].confirmUpdate(paramInfo.paramID, value);
					break;
				case INITIATE_DOMAIN_DOWNLOAD:
					allMotors[motorID].confirmUpdate(paramInfo.paramID);
					break;
				case DOWNLOAD_DOMAIN_SEGMENT:
					allMotors[motorID].confirmUpdate(paramInfo.paramID);
					break;
			}
		}
		
	} else if(readMsg.Msg.ID <= SDO_RX_START + TOTAL_MOTORS_IN_ROBOT) {
		// Should not need to be implemented as the CPU should not receive this type of message from the slaves.
	}
}

int Robot::convertActualValue(int value, int n) {
	
	// Using coded information, 3 = int8, 2 = int16, 1 = int24 and 0 = int32
	// 0 which means int32 does not require conversion
	if(n == 0) return value;
	
	// Shift back by 1 to 
	if(value > halfValue[n-1]) {
		return value - maxValue[n-1] - 1;
	}	else {
		return value;
	}
}


// To set encoder value for homing
void Robot::testInitializeMotor(int motorID) {

	std::stringstream ss;
	dataArrayWrapper inputGraph;
	dataArrayWrapper toMatchGraph;
	PauseTimer waitTimer(INIT_STEP_WAIT_TIME_NS);
	bestFitResult result;
	
	int j;
	int steps[INIT_NUM_STEPS];
		
	ss.str("");
	ss << rPathToDataBaseFolder.str().c_str();
	if(motorID < 10) {
		ss << SENSOR_MAPS_FILE_NAME << "0" << motorID << ".txt";
	} else {			
		ss << SENSOR_MAPS_FILE_NAME << motorID << ".txt";
	}
	inputGraph = loadDataFileArray(ss.str().c_str(), 2);
	// For now, we move each motor individually for safety.
		
	if(inputGraph.row == 0) {
		free(inputGraph.dataArray);
		return;
	}
	// Check if the reference block is too far, i.e. Analog input 2 > 4000
	updateMotorParam(motorID, ANALOG_2_MV);
	if(allMotors[motorID].readParameter(ANALOG_2_MV) > BLOCK_TOO_CLOSE_TO_OUTER_LIMIT_MV) {
		if(allMotors[motorID].readParameter(ANALOG_2_MV) >= MAX_ANALOG_INPUT_MV) {
			TRACE_ERROR_ROBOT_LN("Error in initializing Motor %d: Outside of reading range.", motorID);
			return;
		}
		// If we need to tighten the motor, we must make sure that the the spring is not too tight
		updateMotorParam(motorID, ANALOG_1_MV);
		if(allMotors[motorID].readParameter(ANALOG_1_MV) > MAX_EXTENSION_BEYOND_ZERO_FORCE_MV + allMotors[motorID].zeroForceAnalogValue) {
			TRACE_ERROR_ROBOT_LN("Error in initializing Motor %d: Block near outer range and Spring too loaded.", motorID);
			return;
		}	else {
			for(j = 0; j < INIT_NUM_STEPS; j++) {
				steps[j] = (j+1)*INIT_STEP_SIZE;
			}
		}
	} else {
		for(j = 0; j < INIT_NUM_STEPS; j++) {
			steps[j] = -(j+1)*INIT_STEP_SIZE;
		}
	}
	TRACE_LOG_GREEN_ROBOT_LN("Performing initialization for Motor %d", motorID);
	updateMotorParam(motorID, ACTUAL_POSITION);
	toMatchGraph.dataArray = zeros(INIT_NUM_STEPS + 1, 2);
	toMatchGraph.dataArray[0] = allMotors[motorID].readParameter(ANALOG_2_MV);
	toMatchGraph.dataArray[1] = allMotors[motorID].readParameter(ACTUAL_POSITION);
	toMatchGraph.col = 2;
	toMatchGraph.row = INIT_NUM_STEPS + 1;
	initializeMotorForControl(motorID);
	for(j = 0; j < INIT_NUM_STEPS; j++) {
		// Move motor according to step size and wait, then read Analog input 2
		setMotorControl(motorID, toMatchGraph.dataArray[1] + steps[j], POSITION_CONTROL);
		waitTimer.wait();
		updateMotorParam(motorID, ANALOG_2_MV);
		updateMotorParam(motorID, ACTUAL_POSITION);
		toMatchGraph.dataArray[j*2 + 2] = allMotors[motorID].readParameter(ANALOG_2_MV);
		toMatchGraph.dataArray[j*2 + 3] = allMotors[motorID].readParameter(ACTUAL_POSITION);
	}		
	//allMotors[motorID].encoderOffset = toMatchGraph.dataArray[1] - bestFitGraph2D(inputGraph, toMatchGraph);
	result = bestFitGraph2D(inputGraph, toMatchGraph);
	if(result.validResult) {
		if(setMotorEncoderValue(motorID, result.matchedValue - toMatchGraph.dataArray[1] + toMatchGraph.dataArray[INIT_NUM_STEPS*2 + 1]) == HOMING_SUCCESSFUL) {
		
			TRACE_LOG_CYAN_ROBOT_LN("Motor %d: Homing successful. Initial Encoder Value: %d.\n", motorID, result.matchedValue);
			waitTimer.wait();
			updateMotorParam(motorID, ACTUAL_POSITION);
			TRACE_LOG_CYAN_ROBOT_LN("Motor %d: Current Encoder Value: %d, Moving to %d.\n", motorID, allMotors[motorID].readParameter(ACTUAL_POSITION), result.matchedValue);
			BREAK_POINT("Before Position Control.");
			setMotorControl(motorID, result.matchedValue, POSITION_CONTROL);
		} else {
			TRACE_ERROR_ROBOT_LN("Motor %d: Homing command error. Initial Encoder Value: %d.\n", motorID, result.matchedValue);
			setMotorControl(motorID, toMatchGraph.dataArray[1], POSITION_CONTROL);
		}
	} else {
		TRACE_ERROR_ROBOT_LN("Motor %d: Best fit error.\n", motorID);
		setMotorControl(motorID, toMatchGraph.dataArray[1], POSITION_CONTROL);
	}	
	waitTimer.wait();
	free(inputGraph.dataArray);
	free(toMatchGraph.dataArray);
}

// To set encoder value for homing
void Robot::initializeMotors() {

	std::stringstream ss;
	bool moveMotor;
	dataArrayWrapper inputGraph;
	dataArrayWrapper toMatchGraph;
	PauseTimer waitTimer(INIT_STEP_WAIT_TIME_NS);
	PauseTimer waitTimerSetup(INIT_SETUP_WAIT_TIME_NS);
	bestFitResult result;
	int i, j, bestMatchValue, maxEncoderValue, minEncoderValue, totalEncoderValue;
	int steps[INIT_NUM_STEPS];
	vector<matchingCoord> setOfMatchingCoords;

	toMatchGraph.col = 2;
	toMatchGraph.row = INIT_NUM_STEPS + 1;
			
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		ss.str("");
		ss << rPathToDataBaseFolder.str().c_str();
		if(i < 10) {
			ss << SENSOR_MAPS_FILE_NAME << "0" << i << ".txt";
		} else {			
			ss << SENSOR_MAPS_FILE_NAME << i << ".txt";
		}
		inputGraph = loadDataFileArray(ss.str().c_str(), 2);
		// For now, we move each motor individually for safety.
		
		if(inputGraph.row == 0) {
			free(inputGraph.dataArray);
			continue;
		}
		clearFault(i);
		setMaxVelocityAndAcceleration(i, 5000, 20000);
		
		updateMotorParam(i, ANALOG_2_MV);
		updateMotorParam(i, ACTUAL_POSITION);
		toMatchGraph.dataArray = zeros(INIT_NUM_STEPS + 1, 2);
		toMatchGraph.dataArray[0] = allMotors[i].readParameter(ANALOG_2_MV);
		toMatchGraph.dataArray[1] = allMotors[i].readParameter(ACTUAL_POSITION);
		
		// Check if we have to move motors, i.e. more than 1 encoder value for current sensor reading
		setOfMatchingCoords = findMatchingCoords(inputGraph, toMatchGraph.dataArray[0], false, INITIAL_SEARCH_TOLERANCE);
		
		// No matching coordinates, do not initialize motor
		if(setOfMatchingCoords.size() == 0) {
			TRACE_ERROR_ROBOT_LN("Motor %d: Initialization error: Cannot find matching sensor value (%d) in sensor map.", i, (int)toMatchGraph.dataArray[0]);
			free(inputGraph.dataArray);
			free(toMatchGraph.dataArray);
			continue;
		}
		TRACE_LOG_CYAN_ROBOT_LN("Motor %d: Total Matching Encoder Value: %d.", i, setOfMatchingCoords.size());
				
		moveMotor = true;
		
		// If only 1 match, we just use this one and do not need to move the motor
		if(setOfMatchingCoords.size() == 1) {
			moveMotor = false;
			bestMatchValue = setOfMatchingCoords[0].value;
		} else {
			// If all the matching values are all within ENCODER_COUNTS_INIT_TOLERANCE then we just use the average and do not need
			// to move the motor
			maxEncoderValue = INT_MIN;
			minEncoderValue = INT_MAX;
			totalEncoderValue = 0;
			for(j = 0; j < (int) setOfMatchingCoords.size(); j++) {
				if(maxEncoderValue < setOfMatchingCoords[j].value) {
					maxEncoderValue = setOfMatchingCoords[j].value;
				}
				if(minEncoderValue > setOfMatchingCoords[j].value) {
					minEncoderValue = setOfMatchingCoords[j].value;
				}
				totalEncoderValue = totalEncoderValue + setOfMatchingCoords[j].value;
			}
			TRACE_LOG_CYAN_ROBOT_LN("Motor %d: Max Encoder Value: %d.", i, maxEncoderValue);
			TRACE_LOG_CYAN_ROBOT_LN("Motor %d: Min Encoder Value: %d.", i, minEncoderValue);
			TRACE_LOG_CYAN_ROBOT_LN("Motor %d: Total Encoder Value: %d.", i, totalEncoderValue);
			if(minEncoderValue <= maxEncoderValue && maxEncoderValue - minEncoderValue < ENCODER_COUNTS_INIT_TOLERANCE && setOfMatchingCoords.size() > 0) {
				moveMotor = false;
				bestMatchValue = totalEncoderValue/((int) setOfMatchingCoords.size());

				/***************************************************************/
				// To handle the exception case when Motor Encoder value is super large
				// Cause was due to not typecasting setOfMatchingCoords.size() when used in division above
				// Should not occur any more but check is kept here for safety
				if(bestMatchValue > EXCEPTIONALLY_LARGE_ENCODER_VALUE || bestMatchValue < -EXCEPTIONALLY_LARGE_ENCODER_VALUE) {
					TRACE_ERROR_ROBOT_LN("Large Value Detected, Motor %d: Total Encoder Value: %d. Best Match Value: %d. Total Matches %d.", i, totalEncoderValue, bestMatchValue, (int) setOfMatchingCoords.size());
					TRACE_ERROR_ROBOT_LN("Large Value Detected, Motor %d: Total Encoder Value: %d.", i, totalEncoderValue);
					for(j = 0; j < (int) setOfMatchingCoords.size(); j++) {
						TRACE_ERROR_ROBOT_LN("  Match Value %d Encoder Value: %d.", j, setOfMatchingCoords[j].value);
					}
					bestMatchValue = setOfMatchingCoords[0].value;
				}
				/***************************************************************/
			}
		}
		
		if(moveMotor) {
			// Check if the reference block is too far, i.e. Analog input 2 > 4000
			if(allMotors[i].readParameter(ANALOG_2_MV) > BLOCK_TOO_CLOSE_TO_OUTER_LIMIT_MV) {
				if(allMotors[i].readParameter(ANALOG_2_MV) >= MAX_ANALOG_INPUT_MV) {
					TRACE_ERROR_ROBOT_LN("Error in initializing Motor %d: Outside of reading range.", i);
					continue;
				}
				// If we need to tighten the motor, we must make sure that the the spring is not too tight
				updateMotorParam(i, ANALOG_1_MV);
				if(allMotors[i].readParameter(ANALOG_1_MV) > MAX_EXTENSION_BEYOND_ZERO_FORCE_MV + allMotors[i].zeroForceAnalogValue) {
					TRACE_ERROR_ROBOT_LN("Error in initializing Motor %d: Block near outer range and Spring too loaded.", i);
					continue;
				}	else {
					for(j = 0; j < INIT_NUM_STEPS; j++) {
						steps[j] = (j+1)*INIT_STEP_SIZE;
					}
				}
			} else {
				for(j = 0; j < INIT_NUM_STEPS; j++) {
					steps[j] = -(j+1)*INIT_STEP_SIZE;
				}
			}
			TRACE_LOG_GREEN_ROBOT_LN("Performing initialization for Motor %d", i);
			initializeMotorForControl(i);
			for(j = 0; j < INIT_NUM_STEPS; j++) {
				// Move motor according to step size and wait, then read Analog input 2
				setMotorControl(i, toMatchGraph.dataArray[1] + steps[j], POSITION_CONTROL);
				waitTimer.wait();
				updateMotorParam(i, ANALOG_2_MV);
				updateMotorParam(i, ACTUAL_POSITION);
				toMatchGraph.dataArray[j*2 + 2] = allMotors[i].readParameter(ANALOG_2_MV);
				toMatchGraph.dataArray[j*2 + 3] = allMotors[i].readParameter(ACTUAL_POSITION);
			}		
		
			/*** Obsolete, setting encoder value in motor directly ***/
			// The best fit functions returns the correct (homed) position at toMatchGraph.dataArray[1]
			// The encoderOffset is defined as the offset to add to the correct position such that
			// when this value is passed to the motor, it will move to the correct (homed) position
			// i.e. desiredHomeValue + encoderOffset = correspondingBoardEncoderValue
			//allMotors[i].encoderOffset = toMatchGraph.dataArray[1] - bestFitGraph2D(inputGraph, toMatchGraph);
			result = bestFitGraph2D(inputGraph, toMatchGraph);
			if(result.validResult) {
				if(setMotorEncoderValue(i, result.matchedValue - toMatchGraph.dataArray[1] + toMatchGraph.dataArray[INIT_NUM_STEPS*2 + 1]) == HOMING_SUCCESSFUL) {
					waitTimerSetup.wait();
					TRACE_LOG_CYAN_ROBOT_LN("Motor %d: Homing successful. Initial Encoder Value: %d.", i, result.matchedValue);
					waitTimerSetup.wait();
					setMotorControl(i, result.matchedValue, POSITION_CONTROL);
				} else {	
					TRACE_ERROR_ROBOT_LN("Motor %d: Homing command error. Initial Encoder Value: %d.", i, result.matchedValue);
					setMotorControl(i, toMatchGraph.dataArray[1], POSITION_CONTROL);
				}
			} else {
				TRACE_ERROR_ROBOT_LN("Motor %d: Best fit error.\n", i);
				setMotorControl(i, toMatchGraph.dataArray[1], POSITION_CONTROL);
			}			
		} else {
			initializeMotorForControl(i);
			if(setMotorEncoderValue(i, bestMatchValue) == HOMING_SUCCESSFUL) {
				TRACE_LOG_CYAN_ROBOT_LN("Motor %d: Homing successful. Initial Encoder Value: %d.", i, bestMatchValue);
			} else {	
				TRACE_ERROR_ROBOT_LN("Motor %d: Homing command error. Initial Encoder Value: %d.", i, bestMatchValue);
			}
		}
		waitTimerSetup.wait();
		
		free(inputGraph.dataArray);
		free(toMatchGraph.dataArray);
	}	
}

int Robot::clearFault(int motorID) {
	int error;
	
	error = writeSDOSequence(motorID, CONTROL_WORD, CONTROL_WORD_ADD, CONTROL_WORD_SUBINDEX, CONTROL_WORD_CLEAR_FAULT, CONTROL_WORD_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for control word setting to CLEAR FAULT: %d.", motorID);
	return SEND_SUCCESSFUL;
}

int Robot::clearFaultPDO(int motorID, int controlType) {
	int error;
	long PDOCommand;
	PDOCommand = CONTROL_WORD_CLEAR_FAULT + (controlType << 16);

	if(motorID > TOTAL_MOTORS_IN_ROBOT) return ERROR_MOTOR_ID_EXCEED_MOTOR_NUM;
	if(allMotors[motorID].getState() != OPERATIONAL) return ERROR_MOTOR_NOT_OPERATIONAL;	

	// There will be problem if the PRE_START is not acknowledged before setting it to START
	if(allMotors[motorID].readParameter(CONTROL_WORD) != STATUS_READY_FOR_CONTROL) {
		error = initializeMotorForControl(motorID);
		if(error != SEND_SUCCESSFUL) {
			TRACE_ERROR_ROBOT_LN("Error initializing motor %d for control.", motorID);
			return error;
		}
	}		
	error = writeObjectPDO(motorID, CONTROL_WORD_MODE_OF_OPERATION_PDO_REF, PDOCommand, CONTROL_WORD_MODE_OF_OPERATION_DATA_LENGTH);
	if(error != SEND_SUCCESSFUL) return error;
	
	//Ensures that the command is sent before sending the sync command. Should not have problems but it is better to be safe
	usleep(100000);
	
 	return sync();
}

int Robot::initializeMotorForControl(int motorID) {
	int error;
	// Set Motor to quick stop
	error = writeSDOSequence(motorID, CONTROL_WORD, CONTROL_WORD_ADD, CONTROL_WORD_SUBINDEX, CONTROL_WORD_QUICK_STOP, CONTROL_WORD_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for control word setting to QUICK STOP for motor: %d.", motorID);

	// Set Motor to pre-start
	error = writeSDOSequence(motorID, CONTROL_WORD, CONTROL_WORD_ADD, CONTROL_WORD_SUBINDEX, CONTROL_WORD_PRE_START, CONTROL_WORD_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for control word setting to PRE-START: %d.", motorID);
	return SEND_SUCCESSFUL;
}

int Robot::initializeMotorForControl(int motorID[], int motorNum) {
	int error, i, j;
	// Set Motor to quick stop
	j = 0;
	for(i = 0; i < motorNum; i++) {
		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) continue;
		writeParamMultiple(motorID[i], CONTROL_WORD, CONTROL_WORD_QUICK_STOP);
		j++;
	}
	error = writeSDOSequence(300);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for control word setting to QUICK STOP for %d motors. Targeted: %d", j, motorNum);
	
	// Set Motor to pre start
	j = 0;
	for(i = 0; i < motorNum; i++) {
		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) continue;
		writeParamMultiple(motorID[i], CONTROL_WORD, CONTROL_WORD_PRE_START);
		j++;
	}
	error = writeSDOSequence(300);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for control word setting to PRE START for %d motors. Targeted: %d", j, motorNum);
	
	return SEND_SUCCESSFUL;
}


int Robot::stopMotorControl(int motorID) {
	int error;
	// Set the motor to stop
	error = writeSDOSequence(motorID, CONTROL_WORD, CONTROL_WORD_ADD, CONTROL_WORD_SUBINDEX, CONTROL_WORD_STOP, CONTROL_WORD_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for control word setting to STOP: %d.", motorID);
	return SEND_SUCCESSFUL;
}

int Robot::executeInterpolatedMode(int motorID) {
	int error;
	// Execute interpolated position control
	error = writeSDOSequence(motorID, CONTROL_WORD, CONTROL_WORD_ADD, CONTROL_WORD_SUBINDEX, CONTROL_WORD_START_INTERPOLATED, CONTROL_WORD_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation for control word setting to START INTERPOLATED POSITION MODE: %d.", motorID);
	return SEND_SUCCESSFUL;

}

int Robot::setMotorControlMode(int motorID, int controlType) {
	int error;
	// Set Motor to desired control mode
	error = writeSDOSequence(motorID, OPERATION_MODE, OPERATION_MODE_ADD, OPERATION_MODE_SUBINDEX, controlType, OPERATION_MODE_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation from Motor %d for operation mode setting to MODE: %d.", motorID, OPERATION_MODE);
	return SEND_SUCCESSFUL;
}


// Interpolated position mode to be done in another function
// This is for init motor for control and then set the control mode
int Robot::setMotorControl(int motorID, int value, int controlType) {
	int error;
	// Set Motor to desired control mode
	setMotorControlMode(motorID, controlType);
	// Set Motor to start	
	error = writeSDOSequence(motorID, CONTROL_WORD, CONTROL_WORD_ADD, CONTROL_WORD_SUBINDEX, CONTROL_WORD_START, CONTROL_WORD_DATA_SIZE);
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation from Motor %d for control word setting to START.", motorID);

	// Set control value
	switch(controlType) {
		case POSITION_CONTROL:
			error = writeSDOSequence(motorID, DEMAND_POSITION, DEMAND_POSITION_ADD, DEMAND_POSITION_SUBINDEX, value, DEMAND_POSITION_DATA_SIZE);
			break;
		case VELOCITY_CONTROL:
			error = writeSDOSequence(motorID, DEMAND_VELOCITY, DEMAND_VELOCITY_ADD, DEMAND_VELOCITY_SUBINDEX, value, DEMAND_VELOCITY_DATA_SIZE);
			break;
		case CURRENT_CONTROL:
			error = writeSDOSequence(motorID, DEMAND_CURRENT, DEMAND_CURRENT_ADD, DEMAND_CURRENT_SUBINDEX, value, DEMAND_CURRENT_DATA_SIZE);
			break;
	}
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation from motor %d for control value setting: %d.", motorID, value);

	return SEND_SUCCESSFUL;
}

// This is for setting the control mode
// If the motor was set to stop before this, this will not work, will have to use "setMotorControl" instead
int Robot::setControlValue(int motorID, int value, int controlType) {
	int error;
	// Set control value
	switch(controlType) {
		case POSITION_CONTROL:
			error = writeSDOSequence(motorID, DEMAND_POSITION, DEMAND_POSITION_ADD, DEMAND_POSITION_SUBINDEX, value, DEMAND_POSITION_DATA_SIZE);
			break;
		case VELOCITY_CONTROL:
			error = writeSDOSequence(motorID, DEMAND_VELOCITY, DEMAND_VELOCITY_ADD, DEMAND_VELOCITY_SUBINDEX, value, DEMAND_VELOCITY_DATA_SIZE);
			break;
		case CURRENT_CONTROL:
			error = writeSDOSequence(motorID, DEMAND_CURRENT, DEMAND_CURRENT_ADD, DEMAND_CURRENT_SUBINDEX, value, DEMAND_CURRENT_DATA_SIZE);
			break;
		default:
			return ERROR_INVALID_CONTROL_TYPE;
	}
	if(error != SEND_SUCCESSFUL) return error;
	TRACE_LOG_GREEN_ROBOT_LN("Received confirmation from motor %d for control value setting: %d.", motorID, value);

	return SEND_SUCCESSFUL;
}


int Robot::multipleBlocking(vector<SDOElement> checkList, int maxBlockMs) {
	int maxCycles, cycleNum;
	unsigned int i;
	PauseTimer waitTimer(REPLY_CHECKING_TIME_DELAY_NS);
	bool allReceived;

	if(maxBlockMs == 0) {
		return SEND_SUCCESSFUL;
	}
	
	maxCycles = maxBlockMs*1000000/REPLY_CHECKING_TIME_DELAY_NS;
	cycleNum = 0;
	TRACE_LOG_GREEN_ROBOT_LN("Num elements in block list: %d.", checkList.size());
	while(true) {
		allReceived = true;
		for(i = 0; i < checkList.size(); i++) {	
			if(allMotors[checkList[i].nodeID].isUpdated(checkList[i].paramID) == false) {
				allReceived = false;
				break;
			}
		}
		if(allReceived) break;
		
		waitTimer.wait();		
		if(cycleNum > maxCycles && maxCycles > 0) {
			TRACE_ERROR_ROBOT_LN("Time out, expected replies not received.");
			for(i = 0; i < checkList.size(); i++) {	
				if(allMotors[checkList[i].nodeID].isUpdated(checkList[i].paramID) == false) {
					TRACE_ERROR_ROBOT_LN("Missing Reply for MotorID %d, Param ID %d.", checkList[i].nodeID, checkList[i].paramID);
				}
			}
			return ERROR_NO_REPLY;
		}
		cycleNum++;
	}
	return SEND_SUCCESSFUL;
}



int maxRetries=2;
int Robot::transmitSDO(int nodeID, int paramID, int address, int subIndex, int value, int dataSize, int maxBlockMs)
{
    int retryCount = 0;
    int writeStatus=writeSDOSequence(nodeID, paramID, address,subIndex, value,dataSize, maxBlockMs);
    if(writeStatus==SEND_SUCCESSFUL)
    {
        mtx_.lock();
        while(true)
        {
            if(lineOpen)
            {
                TPCANRdMsg readMsg;
                while(line.read(&readMsg, 100))
                {
                    retryCount++;
                    PauseTimer timer(COMMS_SAMPLING_RATE_N);
                    timer.wait();
                    if(retryCount==maxRetries)
                    {
                        return -1;
                    }
                }

                while(line.read(&readMsg, 100) != CAN_ERR_QRCVEMPTY) {

                    #if MONITOR_COMMS == 1
                        monitorComms(readMsg);
                    #else
                        traceCommsRX(readMsg.Msg);
                        if(readMsg.Msg.ID > PDO1_START && readMsg.Msg.ID < PDO1_END) handlePDO(readMsg, 0);
                        else if(readMsg.Msg.ID > PDO2_START && readMsg.Msg.ID < PDO2_END) handlePDO(readMsg, 1);
                        else if(readMsg.Msg.ID > PDO3_START && readMsg.Msg.ID < PDO3_END) handlePDO(readMsg, 2);
                        else if(readMsg.Msg.ID > PDO4_START && readMsg.Msg.ID < PDO4_END) handlePDO(readMsg, 3);
                        else if(readMsg.Msg.ID > SDO_START && readMsg.Msg.ID < SDO_END) handleSDO(readMsg);
                        else if(readMsg.Msg.ID > HEARTBEAT_START && readMsg.Msg.ID < HEARTBEAT_END) handleHeartbeat(readMsg);
                        else if(readMsg.Msg.ID > EMERGENCY_START && readMsg.Msg.ID < EMERGENCY_END) handleEmergency(readMsg);
                    #endif
                }

                return SEND_SUCCESSFUL;
            }
            else
            {
                return ERROR_LINE_NOT_OPEN;
            }


//            int readStatus= readPDOSequence(nodeID, paramID, address,subIndex, value,dataSize, refPDO, maxBlockMs));
//            if(readStatus==SEND_SUCCESSFUL)
//            {
//                return SEND_SUCCESSFUL;
//            }

        }
        mtx_.unlock();

    }

    return writeStatus;
}

int Robot::transmitSDO(int maxBlockMs)
{
    int retryCount = 0;
    int writeStatus=writeSDOSequence(maxBlockMs);
    if(writeStatus==SEND_SUCCESSFUL)
    {
        mtx_.lock();
        while(true)
        {
            if(lineOpen)
            {
                TPCANRdMsg readMsg;
                while(line.read(&readMsg, 100))
                {
                    retryCount++;
                    PauseTimer timer(COMMS_SAMPLING_RATE_N);
                    timer.wait();
                    if(retryCount==maxRetries)
                    {
                        return -1;
                    }
                }

                while(line.read(&readMsg, 100) != CAN_ERR_QRCVEMPTY) {

                    #if MONITOR_COMMS == 1
                        monitorComms(readMsg);
                    #else
                        traceCommsRX(readMsg.Msg);
                        if(readMsg.Msg.ID > PDO1_START && readMsg.Msg.ID < PDO1_END) handlePDO(readMsg, 0);
                        else if(readMsg.Msg.ID > PDO2_START && readMsg.Msg.ID < PDO2_END) handlePDO(readMsg, 1);
                        else if(readMsg.Msg.ID > PDO3_START && readMsg.Msg.ID < PDO3_END) handlePDO(readMsg, 2);
                        else if(readMsg.Msg.ID > PDO4_START && readMsg.Msg.ID < PDO4_END) handlePDO(readMsg, 3);
                        else if(readMsg.Msg.ID > SDO_START && readMsg.Msg.ID < SDO_END) handleSDO(readMsg);
                        else if(readMsg.Msg.ID > HEARTBEAT_START && readMsg.Msg.ID < HEARTBEAT_END) handleHeartbeat(readMsg);
                        else if(readMsg.Msg.ID > EMERGENCY_START && readMsg.Msg.ID < EMERGENCY_END) handleEmergency(readMsg);
                    #endif
                }
                return SEND_SUCCESSFUL;
            }
            else
            {
                return ERROR_LINE_NOT_OPEN;
            }


//            int readStatus= readPDOSequence(refPDO,maxBlockMs);
//            if(readStatus==SEND_SUCCESSFUL)
//            {
//                return SEND_SUCCESSFUL;
//            }


        }
        mtx_.unlock();

    }

    return writeStatus;
}

int Robot::writeSDOSequence(int maxBlockMs) {
	int error;
	unsigned int i;

	for(i = 0; i < toWrite.size(); i++) {	
		allMotors[toWrite[i].nodeID].setParameter(toWrite[i].paramID, toWrite[i].value);
		error = writeObjectSDO(toWrite[i].nodeID, toWrite[i].address, toWrite[i].subIndex, toWrite[i].value, toWrite[i].dataSize);
		if(error != SEND_SUCCESSFUL) {
			toWrite.clear();
			TRACE_ERROR_ROBOT_LN("Sending Error: %d", error);
			return error;
		}
	}
	error = multipleBlocking(toWrite, maxBlockMs);
	toWrite.clear();
	return error;
}

int Robot::writeSDOSequence(int motorID, int paramID, int address, int subIndex, int value, int dataSize, int maxBlockMs) {
	int error;

	allMotors[motorID].setParameter(paramID, value);
	
	error = writeObjectSDO(motorID, address, subIndex, value, dataSize);
	if(error != SEND_SUCCESSFUL) return error;
	return singleBlocking(motorID, paramID, maxBlockMs);
}


int Robot::readSDOSequence(int maxBlockMs) {
	int error, maxCycles, cycleNum;
	unsigned int i;
	PauseTimer waitTimer(REPLY_CHECKING_TIME_DELAY_NS);
	bool allReceived;

	for(i = 0; i < toRead.size(); i++) {	
		allMotors[toRead[i].nodeID].setParameter(toRead[i].paramID, toRead[i].value);
		error = readObjectSDO(toRead[i].nodeID, toRead[i].address, toRead[i].subIndex);
		if(error != SEND_SUCCESSFUL) {
			toRead.clear();
			TRACE_ERROR_ROBOT_LN("Sending Error: %d", error);
			return error;
		}
	}

	if(maxBlockMs == 0) {
		toRead.clear();
		return SEND_SUCCESSFUL;
	}
	
	maxCycles = maxBlockMs*1000000/REPLY_CHECKING_TIME_DELAY_NS;
	cycleNum = 0;
	while(true) {
		allReceived = true;
		for(i = 0; i < toRead.size(); i++) {	
			if(allMotors[toRead[i].nodeID].isUpdated(toRead[i].paramID) == false) {
				allReceived = false;
				break;
			}
		}
		if(allReceived) break;
		
		waitTimer.wait();		
		if(cycleNum > maxCycles && maxCycles > 0) {
			TRACE_ERROR_ROBOT_LN("Time out, expected replies not received. Missing Replies: ");
			for(i = 0; i < toRead.size(); i++) {	
				if(allMotors[toRead[i].nodeID].isUpdated(toRead[i].paramID) == false) {
					TRACE_ERROR_ROBOT_LN("  Motor ID: %d, Parameter: %d", toRead[i].nodeID, toRead[i].paramID);
				}
			}
			toRead.clear();
			return ERROR_NO_REPLY;
		}
		cycleNum++;
	}
	toRead.clear();
	return SEND_SUCCESSFUL;
}


int Robot::readPDOSequence(int refPDO, int maxBlockMs) {
    int error, maxCycles, cycleNum;
    unsigned int i;
    PauseTimer waitTimer(REPLY_CHECKING_TIME_DELAY_NS);
    bool allReceived;

    for(i = 0; i < toRead.size(); i++) {
        allMotors[toRead[i].nodeID].setParameter(toRead[i].paramID, toRead[i].value);
        error = readObjectPDO(toRead[i].nodeID, refPDO);
        if(error != SEND_SUCCESSFUL) {
            toRead.clear();
            TRACE_ERROR_ROBOT_LN("Sending Error: %d", error);
            return error;
        }
    }

    if(maxBlockMs == 0) {
        toRead.clear();
        return SEND_SUCCESSFUL;
    }

    maxCycles = maxBlockMs*1000000/REPLY_CHECKING_TIME_DELAY_NS;
    cycleNum = 0;
    while(true) {
        allReceived = true;
        for(i = 0; i < toRead.size(); i++) {
            if(allMotors[toRead[i].nodeID].isUpdated(toRead[i].paramID) == false) {
                allReceived = false;
                break;
            }
        }
        if(allReceived) break;

        waitTimer.wait();
        if(cycleNum > maxCycles && maxCycles > 0) {
            TRACE_ERROR_ROBOT_LN("Time out, expected replies not received. Missing Replies: ");
            for(i = 0; i < toRead.size(); i++) {
                if(allMotors[toRead[i].nodeID].isUpdated(toRead[i].paramID) == false) {
                    TRACE_ERROR_ROBOT_LN("  Motor ID: %d, Parameter: %d", toRead[i].nodeID, toRead[i].paramID);
                }
            }
            toRead.clear();
            return ERROR_NO_REPLY;
        }
        cycleNum++;
    }
    toRead.clear();
    return SEND_SUCCESSFUL;
}

int Robot::singleBlockingValue(int motorID, int paramID, int value, int maxBlockMs, int alternateValue) {
	PauseTimer waitTimer(REPLY_CHECKING_TIME_DELAY_NS);
	int maxCycles, cycleNum;
	
	if(maxBlockMs == 0) return SEND_SUCCESSFUL;
	
	maxCycles = maxBlockMs*1000000/REPLY_CHECKING_TIME_DELAY_NS;
	cycleNum = 0;

	while(allMotors[motorID].readParameter(paramID) != value) {
		TRACE_LOG_BLUE_ROBOT_LN("Waiting for Motor %d, ParamID %d, Value %04x. Current Value: %04x.", motorID, paramID, value, allMotors[motorID].readParameter(paramID));	
		
		if(alternateValue != 0) {
			if(allMotors[motorID].readParameter(paramID) == alternateValue) {
				break;
			} else {
				TRACE_LOG_BLUE_ROBOT_LN("Waiting for Motor %d, ParamID %d, Alternative Value %04x. Current Value: %04x.", motorID, paramID, alternateValue, allMotors[motorID].readParameter(paramID));	
			}
		}
		waitTimer.wait();		
		if(cycleNum > maxCycles && maxCycles > 0) return ERROR_NO_REPLY;
		cycleNum++;
	}
	return OBTAINED_DESIRED_VALUE;
}

int Robot::singleBlocking(int motorID, int paramID, int maxBlockMs) {
	PauseTimer waitTimer(REPLY_CHECKING_TIME_DELAY_NS);
	int maxCycles, cycleNum;
	
	if(maxBlockMs == 0) return SEND_SUCCESSFUL;
	
	maxCycles = maxBlockMs*1000000/REPLY_CHECKING_TIME_DELAY_NS;
	cycleNum = 0;

	while(allMotors[motorID].isUpdated(paramID) == false) {
		waitTimer.wait();		
		if(cycleNum > maxCycles && maxCycles > 0) return ERROR_NO_REPLY;
		cycleNum++;
	}
	return SEND_SUCCESSFUL;
}

int Robot::readSDOSequence(int motorID, int paramID, int address, int subIndex, int maxBlockMs) {
	int error;

	allMotors[motorID].requestParameter(paramID);
	
	error = readObjectSDO(motorID, address, subIndex);
	if(error != SEND_SUCCESSFUL) return error;
	return singleBlocking(motorID, paramID, maxBlockMs);
}

int Robot::readPDOSequence(int motorID, int paramID, int address, int subIndex, int refPDO, int maxBlockMs) {
    int error;

    allMotors[motorID].requestParameter(paramID);

    error = readObjectPDO(motorID, refPDO);
    if(error != SEND_SUCCESSFUL) return error;
    return singleBlocking(motorID, paramID, maxBlockMs);
}

int Robot::updateParam(int motorID, int paramID, int maxBlockMs) {
	ObjDictionaryParam a;
	int error;
	
	a = motorParamIDToObjDictionaryParam(paramID);
	if(a.address == 0) return ERROR_INVALID_PARAM_ID;
	
	error = readSDOSequence(motorID, paramID, a.address, a.subIndex);
	if(error != SEND_SUCCESSFUL) return error;
	return singleBlocking(motorID, paramID, maxBlockMs);
}
int Robot::updateParamMultiple(int motorID, int paramID) {
	ObjDictionaryParam a;
	a = motorParamIDToObjDictionaryParam(paramID);
	if(a.address == 0) return ERROR_INVALID_PARAM_ID;
	
	// Value and data size is not needed for reading of object dictionary entries
	toRead.push_back(SDOElement(motorID, paramID, a.address, a.subIndex, 0, 0));
	return toRead.size();
}
	
int Robot::writeParam(int motorID, int paramID, int value, int maxBlockMs) {
	ObjDictionaryParam a;
	int error;
	
	a = motorParamIDToObjDictionaryParam(paramID);
	if(a.address == 0) return ERROR_INVALID_PARAM_ID;
	if(a.dataSize == -1) return ERROR_INVALID_PARAM_TO_WRITE;
	
	error = writeSDOSequence(motorID, paramID, a.address, a.subIndex, value, a.dataSize);
	if(error != SEND_SUCCESSFUL) return error;
	return singleBlocking(motorID, paramID, maxBlockMs);
}
int Robot::writeParamMultiple(int motorID, int paramID, int value) {
	ObjDictionaryParam a;
	a = motorParamIDToObjDictionaryParam(paramID);
	if(a.address == 0) return ERROR_INVALID_PARAM_ID;
	if(a.dataSize == -1) return ERROR_INVALID_PARAM_TO_WRITE;
	
	// Value and data size is not needed for reading of object dictionary entries
	toWrite.push_back(SDOElement(motorID, paramID, a.address, a.subIndex, value, a.dataSize));
	return toWrite.size();
}

/***************** Joint Recording Thread ********************/

void Robot::startJointPositionRecord() {
	recordingJointPosition(RECORD_JOINT_POSITION_FILE_1);
	
	// pThread
	//pthread_create(&jointRecordingThread_ID, NULL, jointRecordingThreadStarter, (void *) this);
	
	jointRecordingThreadStarter();
}

void Robot::jointRecordingThreadStarter() {
	jointRecordingThread_ID = new boost::thread(&Robot::jointRecordingThread, this);
}

/* pThread
void *Robot::jointRecordingThreadStarter(void *a) {
	Robot *b = (Robot *)a;
	b->jointRecordingThread();
	return NULL;
}
*/

void Robot::jointRecordingThread() {
	PauseTimer waitTimer(JOINT_RECORDING_FREQUENCY_NS);
	
	while(true) {
		// Open first file and write joint angles (buffering to avoid file corruption)
		recordingJointPosition(RECORD_JOINT_POSITION_FILE_2);
		// Open second file and write new joint angles (buffering to avoid file corruption)
		recordingJointPosition(RECORD_JOINT_POSITION_FILE_1);
		waitTimer.wait();
	}
}

void Robot::recordingJointPosition(const char * filePath) {
	bool requireReadExecution;
	FILE *recordedJointPositions;
	int i;
		
	requireReadExecution = false;
	// If we are in operational mode, we do not need to send a request as it is sending back the position information
	// periodically
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		if(allMotors[i].getState() != OPERATIONAL) {
			updateParamMultiple(i, ACTUAL_POSITION);
			requireReadExecution = true;
		}
	}
	// Execute all the read SDOs with 200ms max blocking, will cause error if there are less motors but it is a non issue 
	// will not be executed if all motors are in operational state
	if(requireReadExecution) readSDOSequence(200);
	// Open file
	recordedJointPositions = fopen(filePath, "w");
		
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		fprintf(recordedJointPositions, "%d %d\n", i, allMotors[i].readParameter(ACTUAL_POSITION));
	}
		
	// Close file
	fclose(recordedJointPositions);
}


/***************** Recording Thread ********************/

void Robot::pathRecordingThread() {
	PauseTimer waitTimer(RECORDING_FREQUENCY_NS);
	struct timeval prevTime, curTime;
	int timeElapsed;
	gettimeofday(&recordStartTime, NULL);
	gettimeofday(&prevTime, NULL);
	// First position, robot has to move to somewhere close to this position 
	// using position control before starting the trajectory
	writeRecordEntry(0, true);
	waitTimer.wait();
	
	while(recording) {
		// Get time elapsed
		gettimeofday(&curTime, NULL);
		// To account for day change
		if(curTime.tv_sec < prevTime.tv_sec) {
			timeElapsed = (curTime.tv_sec - prevTime.tv_sec + 86400)*1000 + (curTime.tv_usec - prevTime.tv_usec)/1000;
		} else {
			timeElapsed = (curTime.tv_sec - prevTime.tv_sec)*1000 + (curTime.tv_usec - prevTime.tv_usec)/1000;
		}
		// Write change to File		
		writeRecordEntry(timeElapsed);
		prevTime = curTime;
		waitTimer.wait();
		boost::this_thread::interruption_point();
	}
}

void Robot::writeRecordEntry(int timeElapsed, bool initialPoint) {
	int i, velocity, position;
	for(i = 0; i < recordNumMotor; i++) {
		if(ptr_data[recordMotorID[i] - 1] == NULL) continue;
		position = allMotors[recordMotorID[i]].readParameter(ACTUAL_POSITION);
		if(timeElapsed == 0 && initialPoint == false) {
			velocity = 0;
		} else {
			velocity = allMotors[recordMotorID[i]].readParameter(ACTUAL_VELOCITY);
		}
		fprintf(ptr_data[recordMotorID[i] - 1], "%d %d %d\n", timeElapsed, velocity, position);
	}
}

void Robot::startRecord(int motorID[], int numMotor) {

	// Create folder
	std::stringstream ss;
	int i;

	for(i = 0; i < TOTAL_MOTORS_IN_ROBOT; i++) {
		ptr_data[i] = NULL;
	}

	ss.str("");
	ss << rPathToDataBaseFolder.str().c_str() << RECORD_FOLDER_PREFIX;
	recordFolder = createFolder(ss.str().c_str());
	recordNumMotor = 0;
	// Open files
	for(i = 0; i < numMotor; i++) {
		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT || motorID[i] <= 0) continue;
		ss.str("");
		ss << recordFolder;
		if(motorID[i] < 10) {
			ss << "/Motor0" << motorID[i] << ".txt";
		} else {
			ss << "/Motor" << motorID[i] << ".txt";
		}
		ptr_data[motorID[i] - 1] = fopen(ss.str().c_str(),"w");
		recordMotorID[i] = motorID[i];
		updateParamMultiple(motorID[i], ACTUAL_POSITION);
		updateParamMultiple(motorID[i], ACTUAL_VELOCITY);
		recordNumMotor++;
	}
	// Execute all the read SDOs with 1s max blocking
	readSDOSequence(1000);
	
	recording = true;
	// pThread
	// pthread_create(&recordingThread_ID, NULL, recordingThreadStarter, (void *) this);
	recordingThreadStarter();
}


void Robot::stopRecord() {
	// pThread
	// void *exit_status;
	PauseTimer delayTimmer(RECORDING_FREQUENCY_NS);
	int i;

	recording = false;
	
	/* pThread
	pthread_join(recordingThread_ID, &exit_status);
	*/
	recordingThread_ID->interrupt();
	
	// Close files
	// Double sampling rate delay
	delayTimmer.wait();
	delayTimmer.wait();
	
	writeRecordEntry();

	for(i = 0; i < TOTAL_MOTORS_IN_ROBOT; i++) {
		if(ptr_data[i] == NULL) continue;
		fclose(ptr_data[i]);
		ptr_data[i] = NULL;
	}	
}

/* pThread
void *Robot::recordingThreadStarter(void *a) {
	Robot *b = (Robot *)a;

	b->pathRecordingThread();
	return NULL;
}
*/

void Robot::recordingThreadStarter() {
	recordingThread_ID = new boost::thread(&Robot::pathRecordingThread, this);
}


void Robot::forceControlThread() {
	PauseTimer samplingTimer(PSEUDO_FORCE_CONTROL_SAMPLING_RATE_NS);
	int error, velocityOutput;
	unsigned int i;
	
	if(forceControlMotorID.size() != forceControlReferenceForce.size()) {
		TRACE_ERROR_ROBOT_LN("Mismatch in the number of entries for force control motor ID and force reference.");
		return;	
	}
	
	while(inForceControl) {
		for(i = 0; i < forceControlMotorID.size(); i++) { 
			error = forceControlReferenceForce[i] - allMotors[forceControlMotorID[i]].readParameter(ANALOG_1_MV);
			if(forceControlMotorID[i] == 17) {
				TRACE_LOG_PURP_ROBOT_LN("Raw error for motor 17: %d", error);
			}
			if(norm(error) < FORCE_CONTROL_DEAD_ZONE_MV) {
				error = 0;
			}
			if(forceControlMotorID[i] == 17) {
				TRACE_LOG_PURP_ROBOT_LN("Error after deadzone for motor 17: %d", error);
			}
			if(error > 0) {
				if(allMotors[forceControlMotorID[i]].readParameter(ANALOG_2_MV) < 1000) {
					error = 0;
				}
			}
			if(forceControlMotorID[i] == 17) {
				TRACE_LOG_PURP_ROBOT_LN("Error after safety check for motor 17: %d, A2 value: %d", error, allMotors[forceControlMotorID[i]].readParameter(ANALOG_2_MV) );
			}
			
			
			velocityOutput = PSEUDO_FORCE_CONTROL_P_GAIN*error;
			velocityOutput = bound(velocityOutput, MAX_SPEED_OUTPUT_RPM);
			if(forceControlMotorID[i] == 17) {
				TRACE_LOG_PURP_ROBOT_LN("Velocity ouput for motor 17: %d", velocityOutput);
			}
			setControlValue(forceControlMotorID[i], velocityOutput, VELOCITY_CONTROL);
			updateAnalogInputPDO(forceControlMotorID[i]);
		}
		samplingTimer.wait();
		boost::this_thread::interruption_point();
	}	
}

void Robot::startForceControl(int motorID[], int minimumTension[], int numMotor) {
	int i;
	PauseTimer delayTimmer(FORCE_CONTROL_SETTING_DELAY_NS);
	
	inForceControl = false;
	delayTimmer.wait();
	forceControlMotorID.clear();
	forceControlReferenceForce.clear();
	
	for(i = 0; i < numMotor; i++) {
		if(allMotors[motorID[i]].zeroForceAnalogValue > MAX_ZERO_FORCE_REFERNCE_VALUE_MV) {
			TRACE_ERROR_ROBOT_LN("Error in Zero Force Analog Value: %d", allMotors[motorID[i]].zeroForceAnalogValue);
			return;
		} 
	}
	
	if(numMotor > TOTAL_MOTORS_IN_ROBOT) return;	
	for(i = 0; i < numMotor; i++) { 
		if(motorID[i] > TOTAL_MOTORS_IN_ROBOT) return;	
		clearFault(motorID[i]);	
		initializeMotorForControl(motorID[i]);
		forceControlMotorID.push_back(motorID[i]);
	}
	
	delayTimmer.wait();
	for(i = 0; i < numMotor; i++) { 
		startNode(motorID[i]);
		setMotorControl(motorID[i], 0, VELOCITY_CONTROL);
		updateAnalogInputPDO(motorID[i]);
		forceControlReferenceForce.push_back(allMotors[motorID[i]].zeroForceAnalogValue + minimumTension[i]);
	}	

	inForceControl = true;
	/* pThread
	pthread_create(&forceControlThread_ID, NULL, forceControlThreadStarter, (void *) this);
	*/
	forceControlThreadStarter();
}

/* pThread
void *Robot::forceControlThreadStarter(void *a) {
	Robot *b = (Robot *)a;

	b->forceControlThread();
	return NULL;
}
*/

void Robot::forceControlThreadStarter() {
	forceControlThread_ID = new boost::thread(&Robot::forceControlThread, this);
}

void Robot::stopForceControl() {
	unsigned int i;
	// pThread
	// void *exit_status;
	PauseTimer delayTimmer(FORCE_CONTROL_SETTING_DELAY_NS);
	inForceControl = false;
	delayTimmer.wait();
	/* pThread
	pthread_join(forceControlThread_ID, &exit_status);
	*/
	forceControlThread_ID->interrupt();
	
	for(i = 0; i < forceControlMotorID.size(); i++) {
		stopMotorControl(forceControlMotorID[i]);
	}
	
	forceControlMotorID.clear();
	forceControlReferenceForce.clear();
	
}

int Robot::setMotorEncoderValue(int motorID, int encoderValue) {
	if(writeParam(motorID, HOME_METHOD, HOME_METHOD_ACTUAL_POSITION) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error writing home method for motor %d with encoder value %d.", motorID, encoderValue);
		return ERROR_HOMING_UNSUCCESSFUL;
	}
	if(writeParam(motorID, HOME_POSITION, encoderValue) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error writing home position for motor %d with encoder value %d.", motorID, encoderValue);
		return ERROR_HOMING_UNSUCCESSFUL;
	}
	if(writeParam(motorID, OPERATION_MODE, OPERATION_MODE_HOMING) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error writing homing mode operation for motor %d with encoder value %d.", motorID, encoderValue);
		return ERROR_HOMING_UNSUCCESSFUL;
	}
	if(writeParam(motorID, CONTROL_WORD, CONTROL_WORD_PRE_START) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error writing pre start homing for motor %d with encoder value %d.", motorID, encoderValue);
		return ERROR_HOMING_UNSUCCESSFUL;
	}
	if(writeParam(motorID, CONTROL_WORD, CONTROL_WORD_START_HOMING) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error writing start homing for motor %d with encoder value %d.", motorID, encoderValue);
		return ERROR_HOMING_UNSUCCESSFUL;
	}
	return HOMING_SUCCESSFUL;
}

int Robot::initializeDigitalInput(int motorID, int digitalInputNum, int assignedFunctionality, int activeHighLow) {
	int settingMask;

	// Set the digital Input configuration
	if(writeParam(motorID, DIGITAL_INPUT_1_CONFIGURATION + digitalInputNum, assignedFunctionality) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error writing configuration for digital input %d configuration for motor %d with functionality %d.", digitalInputNum + 1, motorID, assignedFunctionality);
		return ERROR_DIGITAL_INPUT_INITIALIZATION_UNSUCCESSFUL;
	}

	// Set polarity
	// Read first before writing so that we do not change values for other inputs
	updateParamMultiple(motorID, DIGITAL_INPUT_POLARITY);
	updateParamMultiple(motorID, DIGITAL_INPUT_MASKS);
	if(readSDOSequence(200) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error reading mask and polarity when setting digital input %d configuration for motor %d with functionality %d.", digitalInputNum + 1, motorID, assignedFunctionality);
		return ERROR_DIGITAL_INPUT_INITIALIZATION_UNSUCCESSFUL;
	}
	settingMask = activeHighLow << assignedFunctionality;
	if(writeParam(motorID, DIGITAL_INPUT_POLARITY, (settingMask | allMotors[motorID].readParameter(DIGITAL_INPUT_POLARITY))) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error writing polarity for digital input %d configuration for motor %d with functionality %d.", digitalInputNum + 1, motorID, assignedFunctionality);
		return ERROR_DIGITAL_INPUT_INITIALIZATION_UNSUCCESSFUL;
	}
	// Enable the digital Input mask
	// Read first before writing so that we do not change values for other inputs
	
	settingMask = 1 << assignedFunctionality;
	if(writeParam(motorID, DIGITAL_INPUT_MASKS, (settingMask | allMotors[motorID].readParameter(DIGITAL_INPUT_MASKS))) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error enabling mask for digital input %d configuration for motor %d with functionality %d.", digitalInputNum + 1, motorID, assignedFunctionality);
		return ERROR_DIGITAL_INPUT_INITIALIZATION_UNSUCCESSFUL;
	}
	
	return DIGITAL_INPUT_CONFIGURATION_SUCCESSFUL;
}


bool Robot::readDigitalInput(int motorID, int digitalInputNum) {

	if(updateParam(motorID, DIGITAL_INPUT_STATES) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error reading digital input %d of Motor %d", digitalInputNum + 1, motorID);
		return false;
	}
	return allMotors[motorID].digitalInputs[digitalInputNum];
}

bool Robot::isIPMBufferEmpty(bool usePDO) {
	int i;
	
	for(i = 0; i < TOTAL_MOTORS_IN_ROBOT; i++) {
		if(isIPMBufferEmpty(i, usePDO) == false) return false;
	}
	return true;
}
bool Robot::isIPMBufferEmpty(int motorID, bool usePDO) {
	if(motorID <= 0 || motorID > TOTAL_MOTORS_IN_ROBOT) {
		TRACE_ERROR_ROBOT_LN("Error: Invalid Motor ID: %d", motorID);
		return false;
	}
	if(updateMotorParam(motorID, BUFFER_CURRENT_SIZE, usePDO) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error: No reply on buffer size. Motor ID: %d", motorID);
		return false;
	}
	if(allMotors[motorID].readParameter(BUFFER_CURRENT_SIZE) > 0) {
		TRACE_LOG_PURP_ROBOT_LN("Motor ID: %d, Buffer not empty. Current size: %d", motorID, allMotors[motorID].readParameter(BUFFER_CURRENT_SIZE));
		return false;
	}
	return true;
}

bool Robot::isIPMBufferEmpty(int motorID[], int numMotor, bool usePDO) {
	int i;
	
	for(i = 0; i < numMotor; i++) {
		if(isIPMBufferEmpty(motorID[i], usePDO) == false) return false;
	}
	return true;
}


bool Robot::isMotorReached(int motorID, int value, int tolerance, bool usePDO) {
	if(motorID <= 0 || motorID > TOTAL_MOTORS_IN_ROBOT) {
		TRACE_ERROR_ROBOT_LN("Error: Invalid Motor ID: %d", motorID);
		return false;
	}
	if(updateMotorParam(motorID, ACTUAL_POSITION, usePDO) != SEND_SUCCESSFUL) {
		TRACE_ERROR_ROBOT_LN("Error: No reply on actual position. Motor ID: %d", motorID);
		return false;
	}
	if(norm(allMotors[motorID].readParameter(ACTUAL_POSITION) - value) > tolerance) {
		TRACE_LOG_PURP_ROBOT_LN("Motor ID: %d, Position (%d) not reached. Current position: %d", motorID, value, allMotors[motorID].readParameter(ACTUAL_POSITION));
		return false;
	}
	return true;
}

bool Robot::isMotorReachedBlocking(int motorID, int value, int tolerance, bool usePDO, int samplingNS, int maxCycle) {
	
	PauseTimer waitTimer(samplingNS);
	int i = 0;

	while(maxCycle == 0 || i < maxCycle) {
		if(isMotorReached(motorID, value, tolerance, usePDO)) {
			return true;
		}
		i++;
		waitTimer.wait();
	}
	return false;
}

neckMotorInfo Robot::setNeckAngle(int angle, int angleType, int velocity, int acceleration, bool initMotorForControl) {
	neckMotorInfo result;
	vector<matchingCoordHighOrder> identifiedCoords;
	
	result.commandSent = false;

	switch(angleType) {
		case NECK_ROLL:	
			if(angle < MIN_NECK_ROLL_ANGLE || angle > MAX_NECK_ROLL_ANGLE) return result;
			identifiedCoords = findMatchingCoords(neckRollLookupTable, angle, NECK_ANGLE_COL_INDEX);
			if(identifiedCoords.size() < 1 || identifiedCoords.size() > 2) return result;
			result.leftNeckRollMotorTargetValue = identifiedCoords[0].value[LEFT_NECK_ROLL_COL_INDEX];
			result.rightNeckRollMotorTargetValue = identifiedCoords[0].value[RIGHT_NECK_ROLL_COL_INDEX];
			if(initMotorForControl) {
				if(initializeMotorForControl(LEFT_NECK_ROLL_MOTOR_ID) != SEND_SUCCESSFUL) return result;
				if(initializeMotorForControl(RIGHT_NECK_ROLL_MOTOR_ID) != SEND_SUCCESSFUL) return result;
			}
			if(setMaxVelocityAndAcceleration(LEFT_NECK_ROLL_MOTOR_ID, velocity, acceleration) != SEND_SUCCESSFUL) return result;
			if(setMaxVelocityAndAcceleration(RIGHT_NECK_ROLL_MOTOR_ID, velocity, acceleration) != SEND_SUCCESSFUL) return result;
			if(angle > neckRollAngle) {
				if(setMotorControl(LEFT_NECK_ROLL_MOTOR_ID, result.leftNeckRollMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
				if(setMotorControl(RIGHT_NECK_ROLL_MOTOR_ID, result.rightNeckRollMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
			} else {
				if(setMotorControl(RIGHT_NECK_ROLL_MOTOR_ID, result.rightNeckRollMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
				if(setMotorControl(LEFT_NECK_ROLL_MOTOR_ID, result.leftNeckRollMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
			}
			neckRollAngle = angle;
			break;
		case NECK_PITCH:	
			if(angle < MIN_NECK_PITCH_ANGLE || angle > MAX_NECK_PITCH_ANGLE) return result;
			identifiedCoords = findMatchingCoords(neckPitchLookupTable, angle, NECK_ANGLE_COL_INDEX);
			if(identifiedCoords.size() < 1 || identifiedCoords.size() > 2) return result;
			result.neckPitchMotorTargetValue = identifiedCoords[0].value[NECK_PITCH_COL_INDEX];
			if(initMotorForControl) {
				if(initializeMotorForControl(NECK_PITCH_MOTOR_ID) != SEND_SUCCESSFUL) return result;
			}
			if(setMaxVelocityAndAcceleration(NECK_PITCH_MOTOR_ID, velocity, acceleration) != SEND_SUCCESSFUL) return result;
			if(setMotorControl(NECK_PITCH_MOTOR_ID, result.neckPitchMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
			neckPitchAngle = angle;
			break;
		case NECK_YAW:	
			if(angle < MIN_NECK_YAW_ANGLE || angle > MAX_NECK_YAW_ANGLE) return result;
			identifiedCoords = findMatchingCoords(neckYawLookupTable, angle, NECK_ANGLE_COL_INDEX);
			if(identifiedCoords.size() < 1 || identifiedCoords.size() > 2) return result;
			result.neckYawMotorTargetValue = identifiedCoords[0].value[NECK_YAW_COL_INDEX];
			if(initMotorForControl) {
				if(initializeMotorForControl(NECK_YAW_MOTOR_ID) != SEND_SUCCESSFUL) return result;
			}
			if(setMaxVelocityAndAcceleration(NECK_YAW_MOTOR_ID, velocity, acceleration) != SEND_SUCCESSFUL) return result;
			if(setMotorControl(NECK_YAW_MOTOR_ID, result.neckYawMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
			neckYawAngle = angle;
			break;
	}
	
	result.commandSent = true;
	return result;
}

neckMotorInfo Robot::setNeckAngle(neckMotionCommand c, bool initMotorForControl) {

	neckMotorInfo result;
	vector<matchingCoordHighOrder> identifiedCoords;
	
	result.commandSent = false;

	// Initialize motor boards before motion
	if(c.neckRollAngle < MIN_NECK_ROLL_ANGLE || c.neckRollAngle > MAX_NECK_ROLL_ANGLE) return result;
	if(c.neckPitchAngle < MIN_NECK_PITCH_ANGLE || c.neckPitchAngle > MAX_NECK_PITCH_ANGLE) return result;
	if(c.neckYawAngle < MIN_NECK_YAW_ANGLE || c.neckYawAngle > MAX_NECK_YAW_ANGLE) return result;
	
	identifiedCoords = findMatchingCoords(neckRollLookupTable, c.neckRollAngle, NECK_ANGLE_COL_INDEX);
	if(identifiedCoords.size() < 1 || identifiedCoords.size() > 2) return result;
	result.leftNeckRollMotorTargetValue = identifiedCoords[0].value[LEFT_NECK_ROLL_COL_INDEX];
	result.rightNeckRollMotorTargetValue = identifiedCoords[0].value[RIGHT_NECK_ROLL_COL_INDEX];
	
	identifiedCoords = findMatchingCoords(neckPitchLookupTable, c.neckPitchAngle, NECK_ANGLE_COL_INDEX);
	if(identifiedCoords.size() < 1 || identifiedCoords.size() > 2) return result;
	result.neckPitchMotorTargetValue = identifiedCoords[0].value[NECK_PITCH_COL_INDEX];
	
	identifiedCoords = findMatchingCoords(neckYawLookupTable, c.neckYawAngle, NECK_ANGLE_COL_INDEX);
	if(identifiedCoords.size() < 1 || identifiedCoords.size() > 2) return result;
	result.neckYawMotorTargetValue = identifiedCoords[0].value[NECK_YAW_COL_INDEX];

	if(initMotorForControl) {
		if(initializeMotorForControl(LEFT_NECK_ROLL_MOTOR_ID) != SEND_SUCCESSFUL) return result;
		if(initializeMotorForControl(RIGHT_NECK_ROLL_MOTOR_ID) != SEND_SUCCESSFUL) return result;
		if(initializeMotorForControl(NECK_PITCH_MOTOR_ID) != SEND_SUCCESSFUL) return result;
		if(initializeMotorForControl(NECK_YAW_MOTOR_ID) != SEND_SUCCESSFUL) return result;
	}

	if(setMaxVelocityAndAcceleration(LEFT_NECK_ROLL_MOTOR_ID, c.rollVelocity, c.rollAcceleration) != SEND_SUCCESSFUL) return result;
	if(setMaxVelocityAndAcceleration(RIGHT_NECK_ROLL_MOTOR_ID, c.rollVelocity, c.rollAcceleration) != SEND_SUCCESSFUL) return result;
	if(setMaxVelocityAndAcceleration(NECK_PITCH_MOTOR_ID, c.pitchVelocity, c.pitchAcceleration) != SEND_SUCCESSFUL) return result;
	if(setMaxVelocityAndAcceleration(NECK_YAW_MOTOR_ID, c.yawVelocity, c.yawAcceleration) != SEND_SUCCESSFUL) return result;

	if(c.neckRollAngle > neckRollAngle) {
		if(setMotorControl(LEFT_NECK_ROLL_MOTOR_ID, result.leftNeckRollMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
		if(setMotorControl(RIGHT_NECK_ROLL_MOTOR_ID, result.rightNeckRollMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
		if(setMotorControl(NECK_PITCH_MOTOR_ID, result.neckPitchMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
		if(setMotorControl(NECK_YAW_MOTOR_ID, result.neckYawMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
	} else {
		if(setMotorControl(RIGHT_NECK_ROLL_MOTOR_ID, result.rightNeckRollMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
		if(setMotorControl(LEFT_NECK_ROLL_MOTOR_ID, result.leftNeckRollMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
		if(setMotorControl(NECK_PITCH_MOTOR_ID, result.neckPitchMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
		if(setMotorControl(NECK_YAW_MOTOR_ID, result.neckYawMotorTargetValue, POSITION_CONTROL) != SEND_SUCCESSFUL) return result;
	}
	
	neckRollAngle = c.neckRollAngle;
	neckPitchAngle = c.neckPitchAngle;
	neckYawAngle = c.neckYawAngle;
	
	result.commandSent = true;
	return result;
}



