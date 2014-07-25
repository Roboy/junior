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
#ifndef _ROBOT_
#define _ROBOT_

#include "Motor.h"
#include <sys/stat.h>
#include <sstream>
#include <string>

// Motor 0 is a dummy motor, this is to link to the node ID on the CANOpen
#define TOTAL_MOTORS_IN_ROBOT 48

#define POSITION_CONTROL OPERATION_MODE_POSITION
#define CURRENT_CONTROL OPERATION_MODE_CURRENT
#define VELOCITY_CONTROL OPERATION_MODE_VELOCITY
#define INTERPOLATED_POSITION_CONTROL OPERATION_MODE_INTERPOLATED_POSITION
#define REPLY_CHECKING_TIME_DELAY_NS 5000000

// Error Codes
#define ERROR_NO_REPLY -5
#define ERROR_INVALID_PARAM_ID -15
#define ERROR_INVALID_PARAM_TO_WRITE -16
#define ERROR_MOTOR_NOT_OPERATIONAL -25
#define ERROR_MOTOR_ID_EXCEED_MOTOR_NUM -26
#define ERROR_TOTAL_MOTOR_NUMBER_EXCEEDED -35
#define ERROR_INVALID_CONTROL_TYPE -36
#define ERROR_SENDING_PDO_ERROR -100
#define ERROR_IPM_FAULT -41
#define ERROR_HOMING_UNSUCCESSFUL -1
#define ERROR_DIGITAL_INPUT_INITIALIZATION_UNSUCCESSFUL -1

#define NO_BLOCK 0
#define INFINITE_BLOCK -1
#define ERROR_READING_FILE -35
#define ERROR_TOO_MANY_WAYPOINTS -36

#define ERROR_INVALID_PDO_PARAM -2147483647
#define READ_SUCCESSFUL 1
#define HOMING_SUCCESSFUL 1
#define DIGITAL_INPUT_CONFIGURATION_SUCCESSFUL 1

#define OBTAINED_DESIRED_VALUE 0

// Trajectory Related constants
// [Time] [Velocity] [Position]
#define NUM_PARAMS_PER_WAYPOINT 3
#define ADDED_WAYPOINTS 0
#define READ_TRAJECTORY_OFFSET_BLOCK_MS 200
#define READ_TRAJECTORY_MULTIPLE_OFFSET_BLOCK_MS 200
#define MAX_TRAJECTORY_TIME_STEP 255

// PDO RX settings
#define PDO1_RX_SIZE 2	// Control Word and Mode of Operation
#define PDO2_RX_SIZE 1	// Demand current
#define PDO3_RX_SIZE 1	// Demand position 
#define PDO4_RX_SIZE 1	// Interpolated position buffer

#define PDO1_ENTRY1_RX_MAP	0x60400010 // Control word address: 6040, Subindex 0, Number of bits 16
#define PDO1_ENTRY2_RX_MAP	0x60600008 // Mode of operation address: 6060, Subindex 0, Number of bits 8
#define PDO2_ENTRY1_RX_MAP	0x20300010 // Demand current (Current Mode Setting Value) address: 2030, Subindex 0, Number of bits 16
#define PDO3_ENTRY1_RX_MAP	0x20620020 // Demand position (Position Mode Setting Value) address: 2062, Subindex 0, Number of bits 32
#define PDO4_ENTRY1_RX_MAP	0x20C10040 // IPM buffer address: 20C1, Subindex 0, Number of bits 16

#define CONTROL_WORD_MODE_OF_OPERATION_PDO_REF PDO1_RX_START // Control Word and Mode of Operation linked to PDO1 RX
#define CONTROL_WORD_MODE_OF_OPERATION_DATA_LENGTH 3
#define DEMAND_CURRENT_PDO_REF PDO2_RX_START // Demand current linked to PDO2 RX
#define DEMAND_CURRENT_DATA_LENGTH 2
#define DEMAND_POSITION_PDO_REF PDO3_RX_START // Demand position linked to PDO3 RX
#define DEMAND_POSITION_DATA_LENGTH 4
#define INTERPOLATED_POSITION_BUFFER_PDO_REF PDO4_RX_START // IPM buffer linked to PDO4 RX
#define INTERPOLATED_POSITION_BUFFER_DATA_LENGTH 8

#define PDO1_RX_SYNC_ASYNC PDO_PARAMETER_TYPE_SYNC
#define PDO2_RX_SYNC_ASYNC PDO_PARAMETER_TYPE_ASYNC
#define PDO3_RX_SYNC_ASYNC PDO_PARAMETER_TYPE_ASYNC
#define PDO4_RX_SYNC_ASYNC PDO_PARAMETER_TYPE_ASYNC

// PDO TX settings
#define PDO1_TX_SIZE 3	// Control Word Display, Mode of Operation Display and IPM Status Buffer
#define PDO2_TX_SIZE 2	// Analog input 1 and 2
#define PDO3_TX_SIZE 2	// Actual position
#define PDO4_TX_SIZE 1	// IPM buffer size

#define PDO1_ENTRY1_TX_MAP	0x60410010 // Control Word Display
#define PDO1_ENTRY2_TX_MAP	0x60610008 // Mode of Operation Display
#define PDO1_ENTRY3_TX_MAP	0x20C40110 // IPM Status Buffer

#define PDO2_ENTRY1_TX_MAP	0x207C0110 // Analog input 1
#define PDO2_ENTRY2_TX_MAP	0x207C0210 // Analog input 2

#define PDO3_ENTRY1_TX_MAP	0x60640020 // Actual position
#define PDO3_ENTRY2_TX_MAP	0x606C0020 // Actual velocity

#define PDO4_ENTRY1_TX_MAP	0x60C40220 // IPM total Buffer Size


#define CONTROL_AND_OPERATION_DISPLAY_PDO_REF PDO1_TX_START // Control Word Display and Mode of Operation Display linked to PDO1 TX
#define ANALOG_INPUTS_PDO_REF PDO2_TX_START // Analog input 1 and Analog input 2 linked to PDO2 TX
#define ACTUAL_POSITION_PDO_REF PDO3_TX_START // Actual position linked to PDO3 TX
#define ACTUAL_VELOCITY_PDO_REF PDO3_TX_START // Actual velocity linked to PDO3 TX
#define REMAINING_BUFFER_SIZE_PDO_REF PDO4_TX_START // IPM buffer linked to PDO4 TX


#define CONTROL_AND_OPERATION_INHIBIT_TIME 0 // No inhibit time, send when changed
#define ANALOG_INPUTS_INHIBIT_TIME 0 // Only using on RTR
#define POSITION_AND_VELOCITY_INHIBIT_TIME 2000 // in multiples of 100us, currently set as 200ms since we need this frequency for 
																								// recording the trajectory
#define BUFFER_SIZE_INHIBIT_TIME 0 // Only using on RTR

#define PDO1_TX_RTR PDO1_TX_ENABLE_RTR
#define PDO2_TX_RTR PDO2_TX_ENABLE_RTR
#define PDO3_TX_RTR PDO3_TX_ENABLE_RTR
#define PDO4_TX_RTR PDO4_TX_ENABLE_RTR

#define PDO1_TX_SYNC_ASYNC PDO_PARAMETER_TYPE_ASYNC
#define PDO2_TX_SYNC_ASYNC PDO_PARAMETER_TYPE_ASYNC_ON_RTR
#define PDO3_TX_SYNC_ASYNC PDO_PARAMETER_TYPE_ASYNC
#define PDO4_TX_SYNC_ASYNC PDO_PARAMETER_TYPE_ASYNC_ON_RTR

#define PDO1_TX_INHIBIT CONTROL_AND_OPERATION_INHIBIT_TIME
#define PDO2_TX_INHIBIT ANALOG_INPUTS_INHIBIT_TIME
#define PDO3_TX_INHIBIT POSITION_AND_VELOCITY_INHIBIT_TIME
#define PDO4_TX_INHIBIT BUFFER_SIZE_INHIBIT_TIME

#define PDO1_TX_DATA_LENGTH 5
#define PDO2_TX_DATA_LENGTH 4
#define PDO3_TX_DATA_LENGTH 8
#define PDO4_TX_DATA_LENGTH 4

#define PDO1_TX1_DATA_LENGTH 2
#define PDO1_TX2_DATA_LENGTH 1
#define PDO1_TX3_DATA_LENGTH 2
#define PDO2_TX1_DATA_LENGTH 2
#define PDO2_TX2_DATA_LENGTH 2
#define PDO3_TX1_DATA_LENGTH 4
#define PDO3_TX2_DATA_LENGTH 4
#define PDO4_TX1_DATA_LENGTH 4

// Putting all related addresses, subindices, data sizes and data types into pre-defined arrays
// to facilitate subsequent mapping and conversions
#include "RobotParamMap.h"

// Recording parameters
#define RECORDING_FREQUENCY_NS 230000000
#define RECORD_FOLDER_PREFIX "database/Trajectories/record"

// Joint recording parameters for initialization
#define JOINT_RECORDING_FREQUENCY_NS 500000000
#define RECORD_JOINT_POSITION_FILE_1 "JointRecordFile1.txt"
#define RECORD_JOINT_POSITION_FILE_2 "JointRecordFile2.txt"


// Trajectory Parameters
#define MAX_VELOCITY_SCALE 0.9
	// To reduce the used buffer size by some amount such that we do not bust the buffer
	// which will lead to critical failure
#define BUFFER_ALLOWANCE 20
#define EXTENDED_LOADING_SAMPLE_RATE_NS 100000000
#define PATH_TO_TRAJECTORIES "database/Trajectories/"

// Sensor Map files
#define ZERO_FORCE_SENSOR_MAP "database/SensorMap/ZeroForceSensorMap.txt"
#define SENSOR_MAPS_FILE_NAME "database/SensorMap/SensorMapMotor"
#define NUM_COLS_IN_ZERO_FORCE_SENSOR_MAP_FILE 2

// Neck lookup table file paths
#define NECK_ROLL_FILE_NAME "database/SensorMap/NeckRollLookupTable.txt"
#define NECK_PITCH_FILE_NAME "database/SensorMap/NeckPitchLookupTable.txt"
#define NECK_YAW_FILE_NAME "database/SensorMap/NeckYawLookupTable.txt"
#define NUM_COLS_IN_NECK_ROLL_LOOKUP_FILE 3
#define NUM_COLS_IN_NECK_PITCH_LOOKUP_FILE 2
#define NUM_COLS_IN_NECK_YAW_LOOKUP_FILE 2

// Motor Characteristics
#define MOTOR_CHARACTERISTICS_FILE "database/SensorMap/MotorCharacteristics.txt"
#define NUM_COLS_IN_MOTOR_CHARACTERISTICS_FILE 3

// Force Control
#define PSEUDO_FORCE_CONTROL_P_GAIN 5
#define FORCE_CONTROL_DEAD_ZONE_MV 20
#define PSEUDO_FORCE_CONTROL_SAMPLING_RATE_NS 30000000
#define FORCE_CONTROL_SETTING_DELAY_NS 500000000
#define MAX_ZERO_FORCE_REFERNCE_VALUE_MV 1000
#define MAX_SPEED_OUTPUT_RPM 700

// Initialization parameters and safety checks
#define MAX_EXTENSION_BEYOND_ZERO_FORCE_MV 500
#define INIT_NUM_STEPS 9
#define INIT_STEP_SIZE 7000
#define BLOCK_TOO_CLOSE_TO_OUTER_LIMIT_MV 4000
#define MAX_ANALOG_INPUT_MV 5000
#define INIT_STEP_WAIT_TIME_NS 1500000000
#define INIT_SETUP_WAIT_TIME_NS 100000000
#define ENCODER_COUNTS_INIT_TOLERANCE 5000

#define DEFAULT_CAN_BUS_RESPONSE_WAIT_TIME_MS 100

// Error and logging
#define ROBOT_LOG_FILE_NAME (const char *) "RobotLogFile.txt"
#define ROBOT_ERROR_FILE_NAME (const char *) "RobotErrorFile.txt"

/****** Robot specific mappings ******/
// Hand related
#define RIGHT_HAND_SENSOR_ID 25
#define LEFT_HAND_SENSOR_ID 1
#define RIGHT_HAND_MOTOR_ID 25
#define LEFT_HAND_MOTOR_ID 1
#define CLOSE_HAND_VALUE 700000
#define OPEN_HAND_VALUE 0

// Neck related
#define LEFT_NECK_ROLL_MOTOR_ID 45
#define RIGHT_NECK_ROLL_MOTOR_ID 46
#define NECK_PITCH_MOTOR_ID 47
#define NECK_YAW_MOTOR_ID 48

#define NECK_ANGLE_COL_INDEX 0
#define LEFT_NECK_ROLL_COL_INDEX 1
#define RIGHT_NECK_ROLL_COL_INDEX 2
#define NECK_PITCH_COL_INDEX 1
#define NECK_YAW_COL_INDEX 1

#define NECK_ROLL 1
#define NECK_PITCH 2
#define NECK_YAW 3

// Neck angle limits
#define MAX_NECK_ROLL_ANGLE 4
#define MIN_NECK_ROLL_ANGLE -4
#define MAX_NECK_PITCH_ANGLE 15
#define MIN_NECK_PITCH_ANGLE 0
#define MAX_NECK_YAW_ANGLE 15
#define MIN_NECK_YAW_ANGLE -15

// Knee related
#define LEFT_KNEE_MOTOR_ID 34
#define LEFT_HIP_FRONT_MOTOR_ID 32
#define LEFT_HIP_REAR_MOTOR_ID 31
#define RIGHT_KNEE_MOTOR_ID 41 
#define RIGHT_HIP_FRONT_MOTOR_ID 39
#define RIGHT_HIP_REAR_MOTOR_ID 38

#define CONVERT_RPM_TO_COUNTS_PER_SEC (2048/60)

// Forearm related
#define RIGHT_FOREARM_MOTOR_ID 26
#define LEFT_FOREARM_MOTOR_ID 2
#define LEFT_OUTWARD_FOREARM_VALUE 0
#define LEFT_MID_FOREARM_VALUE -100000
#define LEFT_INWARD_FOREARM_VALUE -200000
#define RIGHT_OUTWARD_FOREARM_VALUE 0
#define RIGHT_MID_FOREARM_VALUE 100000
#define RIGHT_INWARD_FOREARM_VALUE 200000

// Motor position blocking parameters
#define INFINITE_CYCLES 0

// Debugging
#define EXCEPTIONALLY_LARGE_ENCODER_VALUE 10000000

#define TRACE_ERROR_ROBOT_LN(a...) do{{TRACE_RED_BOLD(a); printf("\033[0;37m\n"); TRACE_TIME(robotErrorFileIndex); TRACE_LOC_LOG(robotErrorFileIndex, a);TRACE_LOG(robotErrorFileIndex, "\n"); TRACE_LOG_CONSOLIDATED_LN_RED_BOLD(a); fflush(logFile[robotErrorFileIndex]);}}while(0)
#define TRACE_ERROR_ROBOT(a...) do{{TRACE_RED_BOLD(a); TRACE_LOG(robotErrorFileIndex, a);TRACE_LOG_CONSOLIDATED_RED_BOLD(a); fflush(logFile[robotErrorFileIndex]);}}while(0)
#define TRACE_ERROR_ROBOT_TIME(a...) do{{TRACE_RED_BOLD(a); TRACE_TIME(robotErrorFileIndex); TRACE_LOC_LOG(robotErrorFileIndex, a);TRACE_LOG_CONSOLIDATED_TIME_RED_BOLD(a); fflush(logFile[robotErrorFileIndex]);}}while(0)
#define TRACE_ERROR_ROBOT_END_LN do{{printf("\033[0;37m\n"); TRACE_LOG(robotErrorFileIndex, "\n");TRACE_LOG_CONSOLIDATED("<BR>\n"); fflush(logFile[robotErrorFileIndex]);}}while(0)

#define TRACE_LOG_CYAN_ROBOT_LN(a...) do{{TRACE_CYAN_BOLD(a); TRACE("\n"); TRACE_TIME(robotLogFileIndex); TRACE_LOC_LOG(robotLogFileIndex, a);TRACE_LOG(robotLogFileIndex, "\n");TRACE_LOG_CONSOLIDATED_LN_CYAN_BOLD(a);}}while(0)
#define TRACE_LOG_CYAN_ROBOT(a...) do{{TRACE_CYAN_BOLD(a); TRACE_LOC_LOG(robotLogFileIndex, a);TRACE_LOG_CONSOLIDATED_CYAN_BOLD(a);}}while(0)

#define TRACE_LOG_PURP_ROBOT_LN(a...) do{{TRACE_PURP_BOLD(a); TRACE("\n"); TRACE_TIME(robotLogFileIndex); TRACE_LOC_LOG(robotLogFileIndex, a);TRACE_LOG(robotLogFileIndex, "\n");TRACE_LOG_CONSOLIDATED_LN_PURP_BOLD(a);}}while(0)
#define TRACE_LOG_PURP_ROBOT(a...) do{{TRACE_PURP_BOLD(a); TRACE_LOC_LOG(robotLogFileIndex, a);TRACE_LOG_CONSOLIDATED_PURP_BOLD(a);}}while(0)

#define TRACE_LOG_BLUE_ROBOT_LN(a...) do{{TRACE_BLUE_BOLD(a); TRACE("\n"); TRACE_TIME(robotLogFileIndex); TRACE_LOC_LOG(robotLogFileIndex, a);TRACE_LOG(robotLogFileIndex, "\n");TRACE_LOG_CONSOLIDATED_LN_BLUE_BOLD(a);}}while(0)
#define TRACE_LOG_BLUE_ROBOT(a...) do{{TRACE_BLUE_BOLD(a); TRACE_LOC_LOG(robotLogFileIndex, a);TRACE_LOG_CONSOLIDATED_BLUE_BOLD(a);}}while(0)

#define TRACE_LOG_GREEN_ROBOT_LN(a...) do{{TRACE_GREEN_BOLD(a); TRACE("\n"); TRACE_TIME(robotLogFileIndex); TRACE_LOC_LOG(robotLogFileIndex, a);TRACE_LOG(robotLogFileIndex, "\n");TRACE_LOG_CONSOLIDATED_LN_GREEN_BOLD(a);}}while(0)
#define TRACE_LOG_GREEN_ROBOT(a...) do{{TRACE_GREEN_BOLD(a); TRACE_LOC_LOG(robotLogFileIndex, a);TRACE_LOG_CONSOLIDATED_GREEN_BOLD(a);}}while(0)
#define TRACE_LOG_GREEN_ROBOT_TIME(a...) do{{TRACE_GREEN_BOLD(a); TRACE_TIME(robotLogFileIndex); TRACE_LOC_LOG(robotLogFileIndex, a); TRACE_LOG_CONSOLIDATED_TIME_GREEN_BOLD(a);}}while(0)
#define TRACE_LOG_GREEN_ROBOT_END_LN do{{TRACE("\n"); TRACE_LOG(robotLogFileIndex, "\n");TRACE_LOG_CONSOLIDATED("<BR>\n");}}while(0)



class SDOElement {
	public:
	int nodeID, paramID, address, subIndex, value, dataSize;
	
	SDOElement(int n, int p, int a, int s, int v, int d);
};

struct neckMotorInfo {
	int leftNeckRollMotorTargetValue, rightNeckRollMotorTargetValue, neckPitchMotorTargetValue, neckYawMotorTargetValue;
	bool commandSent;
};

struct neckMotionCommand {
	int neckRollAngle, neckPitchAngle, neckYawAngle;
	int rollVelocity, pitchVelocity, yawVelocity;
	int rollAcceleration, pitchAcceleration, yawAcceleration;
	
	neckMotionCommand() : rollVelocity(12000), pitchVelocity(12000), yawVelocity(12000), rollAcceleration(30000), pitchAcceleration(30000), yawAcceleration(30000) {}
};

class Robot : public MaxonCANOpen {
	private:
		vector<TPCANRdMsg> dataReceived;
		vector<SDOElement> toWrite, toRead;
		vector<int> forceControlMotorID;
		vector<int> forceControlReferenceForce;
		int robotLogFileIndex, robotErrorFileIndex;
		
		int neckRollAngle, neckPitchAngle, neckYawAngle;
		dataArrayWrapper neckRollLookupTable, neckPitchLookupTable, neckYawLookupTable;
		
		bool recording, inForceControl;
		// Note that due to the convention of the motor ID, ptr_data[0] is for motor ID 1
		// As such, ptr_data[n] is for motor ID n+1
		FILE *ptr_data[TOTAL_MOTORS_IN_ROBOT];
		int recordMotorID[TOTAL_MOTORS_IN_ROBOT];
		int recordNumMotor;
		
		// To be placed within each motor as these varies between motors
		// but it is unlikely to vary in current framework
		static const int addresses[TOTAL_PARAMETERS];
		static const int subIndices[TOTAL_PARAMETERS];
		static const int dataSizes[TOTAL_PARAMETERS];
		static const bool isSignedList[TOTAL_PARAMETERS];

		static const int PDOTXDataLength[PDO_NUM];
		static const int PDOTXIDStart[PDO_NUM];
		
		static const int PDOTXNumEntry[PDO_NUM];
		static const int PDOTXMotorParamMap[PDO_NUM][MAX_PDO_ENTRY];
		static const int PDOTXEntryDataLength[PDO_NUM][MAX_PDO_ENTRY];

		// To handle signed values
		int maxValue[3], halfValue[3];
		
		/* pThread
		pthread_t recordingThread_ID, forceControlThread_ID, jointRecordingThread_ID;

		// Recording thread starting
		static void *recordingThreadStarter(void *a);
		
		// Force control thread starter
		static void *forceControlThreadStarter(void *a);

		// Joint recording thread for initialization
		static void *jointRecordingThreadStarter(void *a);		
		*/
		
		boost::thread* recordingThread_ID; 
		boost::thread* forceControlThread_ID; 
		boost::thread* jointRecordingThread_ID;
		void recordingThreadStarter();
		void forceControlThreadStarter();
		void jointRecordingThreadStarter();
		
		// For joint recording threads for initialization
		void startJointPositionRecord();
		void jointRecordingThread();
		void recordingJointPosition(const char * filePath);
		
		std::stringstream rPathToDataBaseFolder;
	public:		
		struct timeval recordStartTime;	
		std::string recordFolder;
		std::stringstream pathToTrajectories;
	
		Robot(const char *relativePathToDataBaseFolder, bool RecordJointPosition = true);

		vector<Motor> allMotors;
		// Test function, to be removed after testing
		// To set encoder value for homing
		void testInitializeMotor(int motorID);	

		// Initialization Commands
		int clearFault(int motorID);
		int initializePDORXMapping(int motorID);
		int initializePDOTXMapping(int motorID);
		
		// digitalInputNum = 0  for digital input 1 and so on.
		int initializeDigitalInput(int motorID, int digitalInputNum, int assignedFunctionality, int activeHighLow);
		// To set encoder value for homing
		void initializeMotors();
		
		// Max accerlation given in max RPM change per second
		// Max velocity given in RPM
		int setMaxVelocityAndAcceleration(int motorID, int maxVelocity, int maxAcceleration);

		// Upload trajectory struct to buffer
		// When offset is false, the encoder offset is used as home reference
		int uploadTrajectory(int motorID, Trajectory tra, bool offset = false, bool usePDO = true);
		// When offset is false, the encoder offset is used as home reference
		int uploadTrajectoryMultiple(int motorID[], Trajectory tra[], int numMotors, bool offset = false, bool usePDO = true);

		// Currently more complex IPM operations will only work with usePDO = true
		// Reading trajectory file and adding to buffer
		// When offset is false, the encoder offset is used as home reference
		int readTrajectory(int motorID, std::string fileName, bool offset = false, bool usePDO = true);
		// When offset is false, the encoder offset is used as home reference
		int readTrajectoryMultiple(int motorID[], std::string fileName[], int numMotors, bool offset = false, bool usePDO = true);
		// Add function to interploate from current position to start of trajectory in file.
		// Encoder offset is used as home reference
		int readTrajectoryMultipleInterpolate(int motorID[], std::string fileName[], int maxVelocity[], int numMotors, bool usePDO = true);
		// Add function to add points into the buffer as they are consumed
		// function only returns when all the points are consumed
		// When offset is false, the encoder offset is used as home reference
		int readTrajectoryExtended(int motorID, std::string fileName, bool isStarted = false, bool offset = false, bool usePDO = true);
		// When offset is false, the encoder offset is used as home reference
		int readTrajectoryExtendedMultiple(int motorID[], std::string fileName[], int numMotors, bool isStarted = false, bool offset = false, bool usePDO = true);
		

		// PDO related commands
		int setCurrentReferencePDO(int motorID, int value);
		int setPositionReferencePDO(int motorID, int value);
		int setMotorControlModePDO(int motorID, int controlType);
		int startControlPDO(int motorID[], int numMotor, int controlType);
		int startControlPDO(int motorID, int controlType);
		int stopControlPDO(int motorID, int controlType);
		
		int updateAnalogInputPDO(int motorID);
		int updateActualPositionPDO(int motorID);
		int updateActualVelocityPDO(int motorID);
		int updateBufferSizePDO(int motorID);
		int updateAnalogInputMultiplePDO(int motorID[], int numMotors);
		int updateActualPositionMultiplePDO(int motorID[], int numMotors);
		int updateActualVelocityMultiplePDO(int motorID[], int numMotors);
		int updateBufferSizeMultiplePDO(int motorID[], int numMotors);
		void updateAnalogInputPDO();
		void updateActualPositionPDO();
		void updateBufferSizePDO();
		
		int clearFaultPDO(int motorID, int controlType);
		
		// For teaching and learning
		// To be used in PDO mode only as velocity and position information are
		// assumed to be updated via the PDO automatic updates.
		// Only the recording and uploading of trajectories will use encoder offsets for homing
		void pathRecordingThread();
		// Accounts for encoder offset
		void writeRecordEntry(int timeElapsed = 0, bool initialPoint = false);
		void startRecord(int motorID[], int numMotor);
		void stopRecord();
		
		void forceControlThread();
		void startForceControl(int motorID[], int minimumTension[], int numMotor);
		void stopForceControl();
		
		// Control commands
		int setMotorControlMode(int motorID, int controlType);
		int initializeMotorForControl(int motorID);
		int initializeMotorForControl(int motorID[], int motorNum);
		int setMotorControl(int motorID, int value, int controlType);
		int setControlValue(int motorID, int value, int controlType);
		int stopMotorControl(int motorID);
		int executeInterpolatedMode(int motorID);
		
		bool isMotorReached(int motorID, int value, int tolerance = 2000, bool usePDO = false);
		bool isMotorReachedBlocking(int motorID, int value, int tolerance = 2000, bool usePDO = false, int samplingNS = 250000000, int maxCycle = 10);

		// Handling Interpolated Position Mode Buffer Commands
		int addPositionWaypoint(int motorID, int time, int velocity, int position, bool usePDO = false);
		int clearInterpolatedBuffer(int motorID);
		int enableInterpolatedBuffer(int motorID);
		int resetInterpolatedBuffer(int motorID);
		bool isIPMBufferEmpty(bool usePDO = false);
		bool isIPMBufferEmpty(int motorID, bool usePDO = false);
		bool isIPMBufferEmpty(int motorID[], int numMotor, bool usePDO = false);

		// Motor Status Commands
		int updateParam(int motorID, int paramID, int maxBlockMs = DEFAULT_CAN_BUS_RESPONSE_WAIT_TIME_MS);
		int updateParamMultiple(int motorID, int paramID);
		int writeParam(int motorID, int paramID, int value, int maxBlockMs = DEFAULT_CAN_BUS_RESPONSE_WAIT_TIME_MS);
		int writeParamMultiple(int motorID, int paramID, int value);
		bool checkHealth(int motorID);
		
		// set maxBloackMs to -1 for infinite waiting time, 0 for no block
		// For handling of other SDOs
		int writeSDOSequence(int nodeID, int paramID, int address, int subIndex, int value, int dataSize, int maxBlockMs = DEFAULT_CAN_BUS_RESPONSE_WAIT_TIME_MS);
		int writeSDOSequence(int maxBlockMs = DEFAULT_CAN_BUS_RESPONSE_WAIT_TIME_MS);

		int readSDOSequence(int nodeID, int paramID, int address, int subIndex, int maxBlockMs = DEFAULT_CAN_BUS_RESPONSE_WAIT_TIME_MS);
		int readSDOSequence(int maxBlockMs = DEFAULT_CAN_BUS_RESPONSE_WAIT_TIME_MS);
		
		// NMT commands, set motorID = 0 for all nodes
		int enterPreOperational(int motorID=0);
		// Enter operational state
		int startNode(int motorID=0);
		// Enter stopped state
		int stopNode(int motorID=0);
		// Enter initialisation state
		int resetNode(int motorID=0);
		// Enter initialisation state
		int resetCommunication(int motorID=0);
		int setMotorEncoderValue(int motorID, int encoderValue);
		
		// Read digital inputs
		bool readDigitalInput(int motorID, int digitalInputNum);

		// Record the current joint positions into a file and 
		// the previous joint positions into another file
		void recordJointPositions();

		// For controlling the neck motors
		// Returns the desired encoder value
		neckMotorInfo setNeckAngle(int angle, int angleType, int velocity = 12000, int acceleration = 30000, bool initMotorForControl = false);
		
		neckMotorInfo setNeckAngle(neckMotionCommand c, bool initMotorForControl = false);
		
		int getNeckRoll() {return neckRollAngle;}; 
		int getNeckPitch() {return neckPitchAngle;};
		int getNeckYaw() {return neckYawAngle;};

	protected:
		void setMotorState(int motorID, int state);
		
		MotorParam addressAndSubIndexToMotorParam(int address, int subIndex);			
		ObjDictionaryParam motorParamIDToObjDictionaryParam(int motorParamID);

		int convertActualValue(int value, int n);
		void handleHeartbeat(TPCANRdMsg readMsg);
		void handleSDO(TPCANRdMsg readMsg);
		void handlePDO(TPCANRdMsg readMsg, unsigned int id);
		void handleEmergency(TPCANRdMsg readMsg);
		
		int updateMotorParamMultiple(int motorID[], int numMotors, int paramID, bool usePDO = false);
		int updateMotorParam(int motorID, int paramID, bool usePDO = false);
		
		int singleBlockingValue(int motorID, int paramID, int value, int maxBlockMs, int alternateValue = 0);
		int singleBlocking(int motorID, int paramID, int maxBlockMs);
		int multipleBlocking(vector<SDOElement> checkList, int maxBlockMs);
		
		
};

#endif
