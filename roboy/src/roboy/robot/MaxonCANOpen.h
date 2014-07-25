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
#ifndef _MAXON_CAN_OPEN_
#define _MAXON_CAN_OPEN_

#include "CANComms.h"
#include <vector>
#include "genFunc.h"

#define MONITOR_COMMS 0
#define NUM_DIGITAL_INPUTS 6

// Maxon CANOpen information
// Difference between 0x580 (reply from server (Slave)) and 0x600 (transmission from client (Master/CPU))
#define MAXON_SDO_TX_RX_DIFFERENCE 0x80
#define PDO_NUM 4
#define MAX_PDO_ENTRY 8

// Important Object Dictionary Entries address
// DEMAND in this program actually refers to the desired value/reference
// In the EPOS 2 firmware, demand is set internally while in here
// DEMAND actually refers to the "..." Mode Setting Value which is
// set by the user to achieve the desired value for the respective control
// methods.
#define DEMAND_POSITION_ADD 0x2062
#define DEMAND_VELOCITY_ADD 0x206B
#define DEMAND_CURRENT_ADD 0x2030

#define CONTROL_WORD_ADD 0x6040

#define ACTUAL_POSITION_ADD 0x6064
#define ACTUAL_VELOCITY_ADD 0x606C
#define ACTUAL_CURRENT_ADD 0x6078

#define POSITION_CONTROL_GAINS_ADD 0x60FB
#define CURRENT_CONTROL_GAINS_ADD 0x60F6
#define VELOCITY_CONTROL_GAINS_ADD 0x60F9

#define OPERATION_MODE_ADD 0x6060
#define ANALOG_INPUT_ADD 0x207c

#define INTERPOLATED_POSITION_BUFFER_ADD 0x20C1
#define INTERPOLATION_MODE_STATUS_ADD 0x20C4

#define BUFFER_CONFIG_ADD 0x60c4
#define HEARTBEAT_INTERVAL_ADD 0x1017

#define MAX_ACCELERATION_ADD 0x60c5
#define MAX_PROFILE_VELOCITY_ADD 0x607F

#define PDO1_TX_MAPPING_ADD 0x1A00
#define PDO1_TX_PARAMETER_ADD 0x1800
#define PDO2_TX_MAPPING_ADD 0x1A01
#define PDO2_TX_PARAMETER_ADD 0x1801
#define PDO3_TX_MAPPING_ADD 0x1A02
#define PDO3_TX_PARAMETER_ADD 0x1802
#define PDO4_TX_MAPPING_ADD 0x1A03
#define PDO4_TX_PARAMETER_ADD 0x1803

#define PDO1_RX_MAPPING_ADD 0x1600
#define PDO1_RX_PARAMETER_ADD 0x1400
#define PDO2_RX_MAPPING_ADD 0x1601
#define PDO2_RX_PARAMETER_ADD 0x1401
#define PDO3_RX_MAPPING_ADD 0x1602
#define PDO3_RX_PARAMETER_ADD 0x1402
#define PDO4_RX_MAPPING_ADD 0x1603
#define PDO4_RX_PARAMETER_ADD 0x1403

#define FIRST_PDO_CONFIG_ADD 0x1400
#define LAST_PDO_CONFIG_ADD 0x1A03

#define FIRST_PDO_RX_CONFIG_ADD 0x1400
#define LAST_PDO_RX_CONFIG_ADD 0x1603

#define FIRST_PDO_TX_CONFIG_ADD 0x1800
#define LAST_PDO_TX_CONFIG_ADD 0x1A03

// Homing
#define HOME_METHOD_ADD 0x6098
#define HOME_POSITION_ADD 0x2081

// Digital IO
#define DIGITAL_INPUT_CONFIGURATION_ADD 0x2070
#define DIGITAL_INPUT_FUNCTIONALITIES_ADD 0x2071
#define DIGITAL_OUTPUT_CONFIGURATION_ADD 0x2079
#define DIGITAL_OUTPUT_FUNCTIONALITIES_ADD 0x2078

// Important Object Dictionary Entries sub index

#define DEMAND_POSITION_SUBINDEX 0
#define DEMAND_VELOCITY_SUBINDEX 0
#define DEMAND_CURRENT_SUBINDEX 0

#define ACTUAL_POSITION_SUBINDEX 0
#define ACTUAL_VELOCITY_SUBINDEX 0
#define ACTUAL_CURRENT_SUBINDEX 0

#define POSITION_CONTROL_P_GAIN_SUBINDEX 1
#define POSITION_CONTROL_I_GAIN_SUBINDEX 2
#define POSITION_CONTROL_D_GAIN_SUBINDEX 3

#define VELOCITY_CONTROL_P_GAIN_SUBINDEX 1
#define VELOCITY_CONTROL_I_GAIN_SUBINDEX 2

#define CURRENT_CONTROL_P_GAIN_SUBINDEX 1
#define CURRENT_CONTROL_I_GAIN_SUBINDEX 2

#define OPERATION_MODE_SUBINDEX 0
#define CONTROL_WORD_SUBINDEX 0
#define ANALOG_INPUT_1_SUBINDEX 1
#define ANALOG_INPUT_2_SUBINDEX 2

#define INTERPOLATED_POSITION_BUFFER_SUBINDEX 0
#define INTERPOLATION_MODE_STATUS_SUBINDEX 1

#define BUFFER_CLEAR_ENABLE_SUBINDEX 6
#define BUFFER_REMAINING_MEMORY_SUBINDEX 2
#define BUFFER_CURRENT_SIZE_SUBINDEX 4

#define HEARTBEAT_INTERVAL_SUBINDEX 0

#define MAX_ACCELERATION_SUBINDEX 0
#define MAX_PROFILE_VELOCITY_SUBINDEX 0

// Possible up to 8 entries but implement only 3 to
// reduce data size
#define PDO_PARAMTER_COBID_SUBINDEX 1
#define PDO_PARAMTER_TYPE_SUBINDEX 2
#define PDO_PARAMTER_INHIBIT_SUBINDEX 3
#define PDO_MAPPING_SIZE_SUBINDEX 0
#define PDO_MAPPING_ENTRY1_SUBINDEX 1
#define PDO_MAPPING_ENTRY2_SUBINDEX 2
#define PDO_MAPPING_ENTRY3_SUBINDEX 3

// Homing
#define HOME_METHOD_SUBINDEX 0
#define HOME_POSITION_SUBINDEX 0

// Digital IO
#define DIGITAL_INPUT_1_CONFIGURATION_SUBINDEX 1
#define DIGITAL_INPUT_2_CONFIGURATION_SUBINDEX 2
#define DIGITAL_INPUT_3_CONFIGURATION_SUBINDEX 3
#define DIGITAL_INPUT_4_CONFIGURATION_SUBINDEX 4
#define DIGITAL_INPUT_5_CONFIGURATION_SUBINDEX 5
#define DIGITAL_INPUT_6_CONFIGURATION_SUBINDEX 6

#define DIGITAL_INPUT_STATE_SUBINDEX 1
#define DIGITAL_INPUT_MASK_SUBINDEX 2
#define DIGITAL_INPUT_POLARITY_SUBINDEX 3
#define DIGITAL_INPUT_EXECUTION_MASK_SUBINDEX 4

#define DIGITAL_OUTPUT_1_CONFIGURATION_SUBINDEX 3
#define DIGITAL_OUTPUT_2_CONFIGURATION_SUBINDEX 4

#define DIGITAL_OUTPUT_STATE_SUBINDEX 1
#define DIGITAL_OUTPUT_MASK_SUBINDEX 2
#define DIGITAL_OUTPUT_POLARITY_SUBINDEX 3

// Size definition for writable parameters only
#define CONTROL_WORD_DATA_SIZE UNSIGNED16
#define OPERATION_MODE_DATA_SIZE INTEGER8
#define DEMAND_POSITION_DATA_SIZE INTEGER32
#define DEMAND_VELOCITY_DATA_SIZE INTEGER32
#define DEMAND_CURRENT_DATA_SIZE INTEGER16

#define BUFFER_CLEAR_ENABLE_DATA_SIZE INTEGER8
#define BUFFER_REMAINING_MEMORY_DATA_SIZE INTEGER8
#define BUFFER_CURRENT_SIZE_DATA_SIZE INTEGER8

#define CONTROL_GAIN_DATA_SIZE INTEGER16

#define HEARTBEAT_INTERVAL_DATA_SIZE UNSIGNED16
#define MAX_ACCELERATION_DATA_SIZE UNSIGNED32
#define MAX_PROFILE_VELOCITY_DATA_SIZE UNSIGNED32

#define PDO_PARAMTER_COBID_DATA_SIZE UNSIGNED32
#define PDO_PARAMTER_TYPE_DATA_SIZE UNSIGNED8
#define PDO_PARAMTER_INHIBIT_DATA_SIZE UNSIGNED16
#define PDO_MAPPING_SIZE_DATA_SIZE UNSIGNED8
#define PDO_MAPPING_ENTRY_DATA_SIZE UNSIGNED32

// Homing
#define HOME_METHOD_DATA_SIZE INTEGER8
#define HOME_POSITION_DATA_SIZE INTEGER32

// Digital IO
#define DIGITAL_INPUT_CONFIGURATION_DATA_SIZE UNSIGNED16

#define DIGITAL_INPUT_STATE_DATA_SIZE UNSIGNED16
#define DIGITAL_INPUT_MASK_DATA_SIZE UNSIGNED16
#define DIGITAL_INPUT_POLARITY_DATA_SIZE UNSIGNED16
#define DIGITAL_INPUT_EXECUTION_MASK_DATA_SIZE UNSIGNED16

#define DIGITAL_OUTPUT_CONFIGURATION_DATA_SIZE UNSIGNED16

#define DIGITAL_OUTPUT_STATE_DATA_SIZE UNSIGNED16
#define DIGITAL_OUTPUT_MASK_DATA_SIZE UNSIGNED16
#define DIGITAL_OUTPUT_POLARITY_DATA_SIZE UNSIGNED16

// NMT Constants
#define ENTER_PREOPERATIONAL_CS 0x80
#define RESET_COMMUNICATION_CS 0x82
#define RESET_NODE_CS 0x81
#define START_REMOTE_NODE_CS 0x01
#define STOP_REMOTE_NODE_CS 0x02

// Maxon specific State Machine status
#define PREOPERATIONAL 0x7F
#define BOOTUP 0x00
#define OPERATIONAL 0x05
#define STOP 0x04

#define BYTE_SIZE 8
#define EXPEDIATED_TRANSFER 2
#define SIZE_INDICATED 1

// Control word setting
#define CONTROL_WORD_START_INTERPOLATED 0x001F
#define CONTROL_WORD_START 0x010F
#define CONTROL_WORD_STOP 0x0000
#define CONTROL_WORD_QUICK_STOP 0x0006
#define CONTROL_WORD_PRE_START 0x000f
#define CONTROL_WORD_CLEAR_FAULT 0x0080
#define CONTROL_WORD_START_HOMING 0x001f

// Status word information
#define STATUS_FOR_INTERPOLATED_CONTROL 0x1337
#define STATUS_READY_FOR_INTERPOLATED_CONTROL 0x0737
#define STATUS_READY_FOR_CONTROL	0x0337

// Operation mode parameters
#define OPERATION_MODE_POSITION -1
#define OPERATION_MODE_VELOCITY -2
#define OPERATION_MODE_CURRENT -3
#define OPERATION_MODE_INTERPOLATED_POSITION 7
#define OPERATION_MODE_HOMING 6

// PDO mapping parameters
#define PDO_PARAMETER_TYPE_SYNC 1
#define PDO_PARAMETER_TYPE_ASYNC_ON_RTR 253
#define PDO_PARAMETER_TYPE_ASYNC 255
#define PDO1_TX_ENABLE_RTR 0x180
#define PDO1_TX_DISABLE_RTR 0x40000180
#define PDO2_TX_ENABLE_RTR 0x280
#define PDO2_TX_DISABLE_RTR 0x40000280
#define PDO3_TX_ENABLE_RTR 0x380
#define PDO3_TX_DISABLE_RTR 0x40000380
#define PDO4_TX_ENABLE_RTR 0x480
#define PDO4_TX_DISABLE_RTR 0x40000480

// Buffer configuration settings
#define CLEAR_BUFFER 0
#define ENABLE_BUFFER 1

// Buffer infromation
#define MAX_BUFFER_SIZE 64
#define MAX_TIME_STEP 255

// COB ID Range, START refers to motor 0 which is not possible
// it is defined this way to facilitate coding, end is also 1+possible end
// so there are overlaps
#define SYNC 0x80
#define PDO1_START 0x0180
#define PDO1_END 0x0280
#define PDO1_TX_START 0x0180
#define PDO1_TX_END 0x0200
#define PDO1_RX_START 0x0200
#define PDO1_RX_END 0x0280

#define PDO2_START 0x0280
#define PDO2_END 0x0380
#define PDO2_TX_START 0x0280
#define PDO2_TX_END 0x0300
#define PDO2_RX_START 0x0300
#define PDO2_RX_END 0x0380

#define PDO3_START 0x0380
#define PDO3_END 0x0480
#define PDO3_TX_START 0x0380
#define PDO3_TX_END 0x0400
#define PDO3_RX_START 0x0400
#define PDO3_RX_END 0x0480

#define PDO4_START 0x0480
#define PDO4_END 0x0580
#define PDO4_TX_START 0x0480
#define PDO4_TX_END 0x0500
#define PDO4_RX_START 0x0500
#define PDO4_RX_END 0x0580

#define SDO_START 0x0580
#define SDO_END 0x0600
#define SDO_TX_START 0x0580
#define SDO_TX_END 0x0600
#define SDO_RX_START 0x0600
#define SDO_RX_END 0x0680

#define HEARTBEAT_START 0x0700
#define HEARTBEAT_END 0x0800

#define EMERGENCY_START 0x081
#define EMERGENCY_END 0x0ff

// Handling of non expediated messages
#define NON_EXPEDIATED_CHECKING_TIME_DELAY_NS 10000000

// Homing method constants
#define HOME_METHOD_ACTUAL_POSITION 35


//Emergency Message handling and logging
#define TOTAL_EMERGENCY_CODES 36
#define TOTAL_ERROR_REGISTER 8
#define EMERGENCY_MSG_LEN 8
#define NO_EMERGENCY 0

// Digital Input constants
#define DIGITAL_INPUT_GENERAL_PURPOSE_A 15
#define DIGITAL_INPUT_GENERAL_PURPOSE_B 14
#define DIGITAL_INPUT_GENERAL_PURPOSE_C 13
#define DIGITAL_INPUT_GENERAL_PURPOSE_D 12
#define DIGITAL_INPUT_GENERAL_PURPOSE_E 11
#define DIGITAL_INPUT_GENERAL_PURPOSE_F 10
#define DIGITAL_INPUT_GENERAL_PURPOSE_G 9
#define DIGITAL_INPUT_GENERAL_PURPOSE_H 8
#define DIGITAL_INPUT_GENERAL_PURPOSE_I 7
#define DIGITAL_INPUT_GENERAL_PURPOSE_J 6
#define DIGITAL_INPUT_QUICK_STOP 5
#define DIGITAL_INPUT_DEVICE_ENABLE 4
#define DIGITAL_INPUT_POSITION_MARKER 3
#define DIGITAL_INPUT_HOME_SWITCH 2
#define DIGITAL_INPUT_POSITIVE_LIMIT_SWITCH 1
#define DIGITAL_INPUT_NEGATIVE_LIMIT_SWITCH 0

#define DIGITAL_INPUT_1 0
#define DIGITAL_INPUT_2 1
#define DIGITAL_INPUT_3 2
#define DIGITAL_INPUT_4 3

#define ACTIVE_HIGH 0
#define ACTIVE_LOW 1

extern const char * ErrorCodeTranslate[TOTAL_EMERGENCY_CODES];
extern const int ErrorCodeList[TOTAL_EMERGENCY_CODES];
extern const char * ErrorCodeRegister[TOTAL_ERROR_REGISTER];

using std::vector;

class TPCANMsgNonExpediated {
	public:
		TPCANMsg CANMsg;
		int address;
		bool newMessage;
	
		TPCANMsgNonExpediated(TPCANMsg c, int a, bool n);
};


class MaxonCANOpen : public CANDataThread {

	public:	
		MaxonCANOpen();
		int readObjectSDO(int nodeID, int address, int subIndex, int maxBlockMs = 30);
		
		// For expediated transfer
		int writeObjectSDO(int nodeID, int address, int subIndex, int value, int dataSize, int maxBlockMs = 30);

		// For longer data, bytes in bytes[] are arranged from LS(Byte) to MS(Byte)
		// For example in interpolated position data send is [Time (8)] [Vel (24)] [Pos (32)]
		// The LSB of pos32 is in bytes[0] and Time is in bytes[7]
		// Use only if data is larger than 4 bytes
		int writeObjectSDONonExpediated(int nodeID, int address, int subIndex, int bytes[], int numBytes, int maxBlockMs = 30);


		// PDO handling
		int readObjectPDO(int nodeID, int refPDO);
		int writeObjectPDO(int nodeID, int refPDO, int bytes[], int dataLength);
		int writeObjectPDO(int nodeID, int refPDO, long value, int dataLength);

		
		// Send SYNC Command
		int sync();

	protected:
		vector<TPCANMsgNonExpediated> dataSendBuffer;
		
		bool waitOnNonExpediated(int nodeID, int maxBlockMs);
		
		int extractNodeID(int COB_ID);	
		bool inNonExpediatedTransfer(int nodeID);
		void readingThread();
  	int sendNMTCommand(int CS, int nodeID);
		void dataExtraction();
		void monitorComms(TPCANRdMsg readMsg);
		void searchNonExpediatedMessage(TPCANRdMsg readMsg);	

		virtual void handleHeartbeat(TPCANRdMsg readMsg) = 0;
		virtual void handleSDO(TPCANRdMsg readMsg) = 0;
		virtual void handlePDO(TPCANRdMsg readMsg, unsigned int id) = 0;
		virtual void handleEmergency(TPCANRdMsg readMsg) = 0;
		
};

#endif
