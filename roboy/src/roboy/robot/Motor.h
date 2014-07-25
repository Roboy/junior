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
#ifndef _MOTORS_
#define _MOTORS_

#include "MaxonCANOpen.h"

#define TOTAL_PARAMETERS 100

// For lookup table to implement CPU side
// force to analog in conversion
#define INTERPOLATION_COUNT 1000
#define MAX_MILLI_VOLT 5000

#define ACTUAL_POSITION 0
#define ACTUAL_VELOCITY 1
#define ACTUAL_CURRENT 2
#define ACTUAL_FORCE_MV 3
#define ANALOG_1_MV 4
#define ANALOG_2_MV 5

#define DEMAND_POSITION 6
#define DEMAND_VELOCITY 7
#define DEMAND_CURRENT 8
#define DEMAND_FORCE_MV 9

#define DEMAND_FORCE_N 10
#define ACTUAL_FORCE_N 11

#define POSITION_CONTROL_P_GAIN 12
#define POSITION_CONTROL_I_GAIN 13
#define POSITION_CONTROL_D_GAIN 14
#define VELOCITY_CONTROL_P_GAIN 15
#define VELOCITY_CONTROL_I_GAIN 16
#define CURRENT_CONTROL_P_GAIN 17
#define CURRENT_CONTROL_I_GAIN 18
#define FORCE_CONTROL_P_GAIN 19
#define FORCE_CONTROL_I_GAIN 20
#define FORCE_CONTROL_D_GAIN 21

/*********** Parameters unique to Maxon CANOpen ************/


#define OPERATION_MODE 22
#define CONTROL_WORD 23

#define BUFFER_CLEAR_ENABLE 24
#define BUFFER_CURRENT_SIZE 25
#define BUFFER_REMAINING_MEMORY 26

#define HEARTBEAT_INTERVAL 27
#define MAX_PROFILE_VELOCITY 28
#define MAX_ACCELERATION 29

// Possible PDO entries up to 8 but
// define up to 3 as we only use up to 2

#define PDO1_TX_MAPPING_SIZE 30
#define PDO1_TX_MAPPING_ENTRY1 31
#define PDO1_TX_MAPPING_ENTRY2 32
#define PDO1_TX_MAPPING_ENTRY3 33
#define PDO1_TX_PARAMETER_COBID 34
#define PDO1_TX_PARAMETER_TYPE 35
#define PDO1_TX_PARAMETER_INHIBIT 36
#define PDO2_TX_MAPPING_SIZE 37
#define PDO2_TX_MAPPING_ENTRY1 38
#define PDO2_TX_MAPPING_ENTRY2 39
#define PDO2_TX_MAPPING_ENTRY3 40
#define PDO2_TX_PARAMETER_COBID 41
#define PDO2_TX_PARAMETER_TYPE 42
#define PDO2_TX_PARAMETER_INHIBIT 43
#define PDO3_TX_MAPPING_SIZE 44
#define PDO3_TX_MAPPING_ENTRY1 45
#define PDO3_TX_MAPPING_ENTRY2 46
#define PDO3_TX_MAPPING_ENTRY3 47
#define PDO3_TX_PARAMETER_COBID 48
#define PDO3_TX_PARAMETER_TYPE 49
#define PDO3_TX_PARAMETER_INHIBIT 50
#define PDO4_TX_MAPPING_SIZE 51
#define PDO4_TX_MAPPING_ENTRY1 52
#define PDO4_TX_MAPPING_ENTRY2 53
#define PDO4_TX_MAPPING_ENTRY3 54
#define PDO4_TX_PARAMETER_COBID 55
#define PDO4_TX_PARAMETER_TYPE 56
#define PDO4_TX_PARAMETER_INHIBIT 57

#define PDO1_RX_MAPPING_SIZE 58
#define PDO1_RX_MAPPING_ENTRY1 59
#define PDO1_RX_MAPPING_ENTRY2 60
#define PDO1_RX_MAPPING_ENTRY3 61
#define PDO1_RX_PARAMETER_COBID 62
#define PDO1_RX_PARAMETER_TYPE 63
#define PDO2_RX_MAPPING_SIZE 64
#define PDO2_RX_MAPPING_ENTRY1 65
#define PDO2_RX_MAPPING_ENTRY2 66
#define PDO2_RX_MAPPING_ENTRY3 67
#define PDO2_RX_PARAMETER_COBID 68
#define PDO2_RX_PARAMETER_TYPE 69
#define PDO3_RX_MAPPING_SIZE 70
#define PDO3_RX_MAPPING_ENTRY1 71
#define PDO3_RX_MAPPING_ENTRY2 72
#define PDO3_RX_MAPPING_ENTRY3 73
#define PDO3_RX_PARAMETER_COBID 74
#define PDO3_RX_PARAMETER_TYPE 75
#define PDO4_RX_MAPPING_SIZE 76
#define PDO4_RX_MAPPING_ENTRY1 77
#define PDO4_RX_MAPPING_ENTRY2 78
#define PDO4_RX_MAPPING_ENTRY3 79
#define PDO4_RX_PARAMETER_COBID 80
#define PDO4_RX_PARAMETER_TYPE 81

// For homing 
#define HOME_METHOD 82
#define HOME_POSITION 83

// For IPM status (Error Checking)
#define INTERPOLATION_MODE_STATUS 84

// For Digital Inputs/Outputs
#define DIGITAL_INPUT_1_CONFIGURATION 85 // Sets the digital input 1 functionalities for Maxon EPOS 2 boards
#define DIGITAL_INPUT_2_CONFIGURATION 86 // Sets the digital input 2 functionalities for Maxon EPOS 2 boards
#define DIGITAL_INPUT_3_CONFIGURATION 87 // Sets the digital input 3 functionalities for Maxon EPOS 2 boards
#define DIGITAL_INPUT_4_CONFIGURATION 88 // Sets the digital input 4 functionalities for Maxon EPOS 2 boards
#define DIGITAL_INPUT_5_CONFIGURATION 89 // Sets the digital input 5 functionalities for Maxon EPOS 2 boards
#define DIGITAL_INPUT_6_CONFIGURATION 90 // Sets the digital input 6 functionalities for Maxon EPOS 2 boards
#define DIGITAL_INPUT_STATES 91 // Digital input values, corresponding bits for the DIs are based on the configuration
#define DIGITAL_INPUT_MASKS 92 // Enabling the DIs based on the corresponding bits
#define DIGITAL_INPUT_POLARITY 93 // Setting the corresponding bits related to the DIs as active low or active high (0 for active high, 1 for active low)
#define DIGITAL_INPUT_EXECUTION_MASK 94 // Enabling the special DI functionalities, i.e. position limit switches 

#define DIGITAL_OUTPUT_1_CONFIGURATION 95 // Sets the digital output 1 (listed as 3) functionalities for Maxon EPOS 2 boards
#define DIGITAL_OUTPUT_2_CONFIGURATION 96 // Sets the digital output 2 (listed as 4) functionalities for Maxon EPOS 2 boards
#define DIGITAL_OUTPUT_STATES 97 // Digital output values, corresponding bits for the DOs are based on the configuration
#define DIGITAL_OUTPUT_MASKS 98 // Enabling the DOs based on the corresponding bits
#define DIGITAL_OUTPUT_POLARITY 99 // Setting the corresponding bits related to the DOs as active low or active high (0 for active high, 1 for active 

// For heart beat
#define HEARTBEAT_TOLERANCE_S 2
#define IS_ALIVE true
#define NOT_ALIVE false


/*********** Parameters unique to Maxon CANOpen ************/

// Trajectory based constants for waypoints
#define NUM_COLS_WAYPOINT_FILE 3
#define WAYPOINT_FILE_EXCESS_POINTS 2
#define WAYPOINT_TIME_STEP 250

struct TrajectoryElement {
	int time, velocity, position;
};

struct CubicSplineResult {
	bool success;
	int motorID;
	int maxVelocity, maxAcceleration;
};


// To change into a vector
struct Trajectory {
	TrajectoryElement t[MAX_BUFFER_SIZE];
	int numElements, maxAcceleration, maxVelocity;
};

struct PID {
	int P, I, D;
};

struct MotorParam {
	int paramID;
	bool isSigned;
};

class Motor {
	private:
		int MotorParameters[TOTAL_PARAMETERS];
		bool updated[TOTAL_PARAMETERS];
		float interpolatedForce[INTERPOLATION_COUNT];
		int lastHeartBeat, state;
		
		// Gives the bit "mask" for each assigned digital input
		// Will be set when the configuration for digital input is set on the computer side, i.e. setParameter function
		int digitalInputsAssignedBit[NUM_DIGITAL_INPUTS];
		
		// For converting between force and millivolt
		float springConstant, lengthAtRest, milliVoltTolengthChangeScale, halfPulleyDistance;
		
		// Based on MAX_MILLI_VOLT or Mechanical settings
		float maxForce;	
		int maxMilliVolt, milliVoltStep, minMilliVolt;	
		int interpolate(float larger, float smaller, float value);
		
	public:	
		int maxVelocity, maxAcceleration, countsPerTurn;
		int zeroForceAnalogValue;
		
		// Only logic true or false, Active low and active high determined at board level through the polarity setting
		bool digitalInputs[NUM_DIGITAL_INPUTS];
		
		// For lookup table to implement CPU side
		// force to analog in conversion	
		// May need to add in more parameters for other information
		Motor(float K, float L, float G, float a, int v, int zFAI, int cPT, int mV, int mA);
		void computeLookupTable(float K, float L, float G, float a, int v, int zFAI);
		int convertForceToMilliVolt(float force);
		float convertMilliVoltToForce(int milliVolt);
		
		void setMotorCharacteristics(int cPT, int mV, int mA);
		
		// Position in quarter encoder counts
		// velocity in RPM, timePeriod in seconds, timeStep in ms, must be smaller than 255
		// 
		Trajectory cubicSpline(int startPosition, int startVelocity, int endPosition, int endVelocity, double timePeriod, int timeStep, int cPT, int mV, int mA);
		Trajectory cubicSpline(int startPosition, int startVelocity, int endPosition, int endVelocity, double timePeriod, int timeStep);
		// For file containing waypoints
		// Array size of timeBetweenWaypoints is 1 less than numWaypoints
		// timeBetweenWaypoints is in terms of seconds
		CubicSplineResult cubicSpline(const char* inputFilePath, const char* outputFilePath, double timeBetweenWaypoints[], int numWaypoints);

		
		bool isAlive();
		void heartbeat(int s);
		void setState(int s);
		int getState(){return state;};
		
		void setParameter(int id, int value);
		void confirmUpdate(int id, int value);
		void confirmUpdate(int id);
		bool isUpdated(int id);
		int readParameter(int id);
		void requestParameter(int id);	
	
};

#endif
