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
#include "MaxonCANOpen.h"

#include "debug.h"

const char * ErrorCodeTranslate[TOTAL_EMERGENCY_CODES] = {"Generic Error", "Overcurrent Error", "Overvoltage Error", "Undervoltage Error", "Overtemperature Error", "Logic Supply Voltage Too Low Error", "Supply Voltage Output Stage Too Low Error", "Internal Software Error", "Software Parameter Error", "Position Sensor Error", "CAN Overrun Error (Objects lost)", "CAN Overrun Error", "CAN Passive Mode Error", "CAN Life Guard Error", "CAN Transmit COB-ID Collision Error", "CAN Bus Off Error", "CAN Rx Queue Overflow Error", "CAN Tx Queue Overflow Error", "CAN PDO Length Error", "Following Error", "Hall Sensor Error", "Index Processing Error", "Encoder Resolution Error", "Hall Sensor not found Error", "Negative Limit Switch Error", "Positive Limit Switch Error", "Hall Angle Detection Error", "Software Position Limit Error", "Position Sensor Breach Error", "System Overloaded Error", "Interpolated Position Mode Error", "Auto Tuning Identification Error", "Gear Scaling Factor Error", "Controller Gain Error", "Main Sensor Direction Error", "Auxiliary Sensor Direction Error"};

const int ErrorCodeList[TOTAL_EMERGENCY_CODES] = {0x1000, 0x2310, 0x3210, 0x3220, 0x4210, 0x5113, 0x5114, 0x6100, 0x6320, 0x7320, 0x8110, 0x8111, 0x8120, 0x8130, 0x8150, 0x81FD, 0x81FE, 0x81FF, 0x8210, 0x8611, 0xFF01, 0xFF02, 0xFF03, 0xFF04, 0xFF06, 0xFF07, 0xFF08, 0xFF09, 0xFF0A, 0xFF0B, 0xFF0C, 0xFF0D, 0xFF0F, 0xFF10, 0xFF11, 0xFF12};

const char * ErrorCodeRegister[TOTAL_ERROR_REGISTER] = {"Generic error", "Current error", "Voltage error", "Temperature error", "Communication error", "Device profile-specific", "reserved", "Motion error"};

TPCANMsgNonExpediated::TPCANMsgNonExpediated(TPCANMsg c, int a, bool n) {
	int i;
	CANMsg.ID = c.ID;
	CANMsg.MSGTYPE = c.MSGTYPE;
	CANMsg.LEN = c.LEN;
	
	for(i = 0; i < 8; i++) CANMsg.DATA[i] = c.DATA[i];
	
	newMessage = n;
	address = a;
}

MaxonCANOpen::MaxonCANOpen() : CANDataThread() {

}


void MaxonCANOpen::readingThread() {
	PauseTimer timer(COMMS_SAMPLING_RATE_N);
	while(lineOpen) {
		dataExtraction();
		timer.wait();
	}
}

/**************** SDO Expediated Handling *******************/

int MaxonCANOpen::readObjectSDO(int nodeID, int address, int subIndex, int maxBlockMs) {
	TPCANMsg sendMsg;
	int i, SDOCmdSpecifier;
	if(!lineOpen) return ERROR_NOT_CONNECTED;
	
	SDOCmdSpecifier = INITIATE_DOMAIN_UPLOAD_CLIENT << 5;
	
	sendMsg.ID = 0x600 + nodeID;
	sendMsg.MSGTYPE = MSGTYPE_STANDARD;
	sendMsg.LEN = 8;
	sendMsg.DATA[0] = SDOCmdSpecifier;
	sendMsg.DATA[1] = 0xff & address;
	sendMsg.DATA[2] = address / 0x100;
	sendMsg.DATA[3] = subIndex;
	for(i = 4; i < 8; i++) sendMsg.DATA[i] = 0x00;

	waitOnNonExpediated(nodeID, maxBlockMs);
	return transmit(&sendMsg);
}

int MaxonCANOpen::writeObjectSDO(int nodeID, int address, int subIndex, int value, int dataSize, int maxBlockMs) {
	TPCANMsg sendMsg;
	int i, SDOCmdSpecifier;
	if(!lineOpen) return ERROR_NOT_CONNECTED;
	
	SDOCmdSpecifier = (INITIATE_DOMAIN_DOWNLOAD_CLIENT << 5) + (dataSize << 2) + EXPEDIATED_TRANSFER + SIZE_INDICATED;
	
	sendMsg.ID = 0x600 + nodeID;
	sendMsg.MSGTYPE = MSGTYPE_STANDARD;
	sendMsg.LEN = 8;
	sendMsg.DATA[0] = SDOCmdSpecifier;
	sendMsg.DATA[1] = 0xff & address;
	sendMsg.DATA[2] = address / 0x100;
	sendMsg.DATA[3] = subIndex;
	
	for(i = 4; i < 8 - dataSize; i++) {
		sendMsg.DATA[i] = (value >> (BYTE_SIZE*(i-4))) & 0xff;
	}
	for(i = 8 - dataSize; i < 8; i++) {
		sendMsg.DATA[i] = 0;
	}
	waitOnNonExpediated(nodeID, maxBlockMs);
	return transmit(&sendMsg);
}

/**************** SDO Expediated Handling *******************/

/**************** PDO Handling *******************/

int MaxonCANOpen::writeObjectPDO(int nodeID, int refPDO, int bytes[], int dataLength) {
	int i;
	TPCANMsg sendMsg;
	sendMsg.ID = refPDO + nodeID;
	sendMsg.LEN = dataLength;
	sendMsg.MSGTYPE = MSGTYPE_STANDARD;
	for(i = 0; i < dataLength; i++) {
		sendMsg.DATA[i] = bytes[i];
	}
	// PDO should not need to wait for non-epxediated commands	
	return transmit(&sendMsg);
}

int MaxonCANOpen::writeObjectPDO(int nodeID, int refPDO, long value, int dataLength) {
	int i;
	TPCANMsg sendMsg;
	sendMsg.ID = refPDO + nodeID;
	sendMsg.LEN = dataLength;
	sendMsg.MSGTYPE = MSGTYPE_STANDARD;
	for(i = 0; i < dataLength; i++) {
		sendMsg.DATA[i] = (value >> (BYTE_SIZE*(i))) & 0xff;
	}	
	// PDO should not need to wait for non-epxediated commands	
	return transmit(&sendMsg);
}

// Should send RTR command
int MaxonCANOpen::readObjectPDO(int nodeID, int refPDO) {
	TPCANMsg sendMsg;
	sendMsg.ID = refPDO + nodeID;
	sendMsg.LEN = 0;
	sendMsg.MSGTYPE = MSGTYPE_RTR;
	// PDO should not need to wait for non-epxediated commands	
	return transmit(&sendMsg);
}

/**************** PDO Handling *******************/

/**************** Non Expediated Handling *******************/

bool MaxonCANOpen::waitOnNonExpediated(int nodeID, int maxBlockMs) {
	PauseTimer waitTimer(NON_EXPEDIATED_CHECKING_TIME_DELAY_NS);
	int maxCycles, cycleNum;
	
	maxCycles = maxBlockMs*1000000/NON_EXPEDIATED_CHECKING_TIME_DELAY_NS;
	cycleNum = 0;

	while(inNonExpediatedTransfer(nodeID)) {
		waitTimer.wait();		
		if(cycleNum > maxCycles && maxCycles > 0) { 
			TRACE_RED_BOLD_LN("ERROR: Exceeding wait time on non-expediated messages. \nCorrupting previous non-expediated message!");
			return false;
		}
		cycleNum++;
	}
	return true;
}

// Currently ignores priority, will need to implement priority
bool MaxonCANOpen::inNonExpediatedTransfer(int nodeID) {
	int queueNodeID;
	unsigned int i;
	
	for(i = 0; i < dataSendBuffer.size(); i++) {
		queueNodeID = extractNodeID(dataSendBuffer[i].CANMsg.ID);
		if(nodeID == queueNodeID) return true;
	}
	return false;
}

int MaxonCANOpen::writeObjectSDONonExpediated(int nodeID, int address, int subIndex, int bytes[], int numBytes, int maxBlockMs) {
	TPCANMsg sendMsg, sendMsgSubsequent;
	int i, j, numMsg, remainBytes;
	if(!lineOpen) return ERROR_NOT_CONNECTED;

	waitOnNonExpediated(nodeID, maxBlockMs);
	
	sendMsg.ID = 0x600 + nodeID;
	sendMsg.MSGTYPE = MSGTYPE_STANDARD;
	sendMsg.LEN = 8;
	sendMsg.DATA[0] = 0x21;
	sendMsg.DATA[1] = 0xff & address;
	sendMsg.DATA[2] = address / 0x100;
	sendMsg.DATA[3] = subIndex;
	sendMsg.DATA[4] = numBytes;
	for(i = 5; i < 8; i++) sendMsg.DATA[i] = 0x00;

	numMsg = numBytes/NUM_BYTES_IN_EXTENDED_MESSAGE;
	remainBytes = numBytes - numMsg*NUM_BYTES_IN_EXTENDED_MESSAGE;
	if(remainBytes == 0) {
		numMsg = numMsg - 1;
		remainBytes = NUM_BYTES_IN_EXTENDED_MESSAGE;
	}

	sendMsgSubsequent.MSGTYPE = MSGTYPE_STANDARD;
	sendMsgSubsequent.LEN = 8;
	sendMsgSubsequent.ID = 0x600 + nodeID;
	for(i = 0; i < numMsg; i++) {
		if(i%2 == 0) {
			sendMsgSubsequent.DATA[0] = 0x00;
		} else {
			sendMsgSubsequent.DATA[0] = 0x10;
		}
		for(j = 1; j <= NUM_BYTES_IN_EXTENDED_MESSAGE; j++) {
			sendMsgSubsequent.DATA[j] = bytes[i*NUM_BYTES_IN_EXTENDED_MESSAGE + j - 1];
		}
		if(i == 0) {
			dataSendBuffer.push_back(TPCANMsgNonExpediated(sendMsgSubsequent, address, true));
		} else {
			dataSendBuffer.push_back(TPCANMsgNonExpediated(sendMsgSubsequent, address, false));
		}
	}
	if(i%2 == 0) {
		sendMsgSubsequent.DATA[0] = 0x0f - remainBytes*2;
	} else {
		sendMsgSubsequent.DATA[0] = 0x1f - remainBytes*2;
	}
	for(j = 1; j < 8; j++) {
		sendMsgSubsequent.DATA[j] = 0;
	}
	for(j = 1; j <= remainBytes; j++) {
		sendMsgSubsequent.DATA[j] = bytes[i*NUM_BYTES_IN_EXTENDED_MESSAGE + j - 1];
	}
	
	if(i == 0) {
		dataSendBuffer.push_back(TPCANMsgNonExpediated(sendMsgSubsequent, address, true));	
	} else {
		dataSendBuffer.push_back(TPCANMsgNonExpediated(sendMsgSubsequent, address, false));	
	}
	return transmit(&sendMsg);
}


void MaxonCANOpen::searchNonExpediatedMessage(TPCANRdMsg readMsg) {
	unsigned int i, COB_ID;
	int scs, address, error;
	
	COB_ID = readMsg.Msg.ID + MAXON_SDO_TX_RX_DIFFERENCE;
	scs = (0b11100000 & readMsg.Msg.DATA[0]) >> 5;
	if(scs != 1 && scs != 3) return;

	address = readMsg.Msg.DATA[1] + readMsg.Msg.DATA[2]*0x100;
	
	TRACE_LOG_COMMS_LN_RX("Current data send buffer size: %d, Msg Add: 0x%04x, scs: %d, search COB ID: 0x%03x", dataSendBuffer.size(), address, scs, COB_ID);
	
	for(i = 0; i < dataSendBuffer.size(); i++) {
		if(COB_ID == dataSendBuffer[i].CANMsg.ID) {
			if(scs == 3) {
				// Ensures that it is the correct address
				if(address != dataSendBuffer[i].address || dataSendBuffer[i].newMessage == false) continue;
			} else {
				// Ensures that subsequent transmission has the correct toggle bit
				if((dataSendBuffer[i].CANMsg.DATA[0] & 0b00010000) == (readMsg.Msg.DATA[0] & 0b00010000) || dataSendBuffer[i].newMessage == true) return;
			}
			
			error = transmit(&dataSendBuffer[i].CANMsg);
			if(error == SEND_SUCCESSFUL) {
				dataSendBuffer.erase(dataSendBuffer.begin() + i);
			}
			return;
		}
	}
}

/**************** Non Expediated Handling *******************/

int MaxonCANOpen::sync() {

	TPCANMsg sendMsg;
	if(!lineOpen) return ERROR_NOT_CONNECTED;
	
	sendMsg.ID = SYNC;
	sendMsg.MSGTYPE = MSGTYPE_STANDARD;
	sendMsg.LEN = 0;

	return transmit(&sendMsg);
}

int MaxonCANOpen::sendNMTCommand(int CS, int nodeID) {

	TPCANMsg sendMsg;
	if(!lineOpen) return ERROR_NOT_CONNECTED;
	
	sendMsg.ID = 0;
	sendMsg.MSGTYPE = MSGTYPE_STANDARD;
	sendMsg.LEN = 2;
	sendMsg.DATA[0] = CS;
	sendMsg.DATA[1] = nodeID;

	return transmit(&sendMsg);
}

void MaxonCANOpen::monitorComms(TPCANRdMsg readMsg) {
	int scs;

	if(readMsg.Msg.ID != 0x581 && readMsg.Msg.ID != 0x601) {
		traceCommsRX(readMsg.Msg);
		return;
	}
	scs = (0b11100000 & readMsg.Msg.DATA[0]) >> 5;

	if(readMsg.Msg.ID == 0x581) {
		if(scs == 3 || scs == 1) traceCommsRX(readMsg.Msg);
	} else {
		if(scs == 1 || scs == 0) traceCommsRX(readMsg.Msg);
	}
}

int MaxonCANOpen::extractNodeID(int COB_ID) {
	if(COB_ID > PDO1_TX_START && COB_ID < PDO1_TX_END) {
		return COB_ID - PDO1_TX_START; 
	}
	if(COB_ID > PDO1_RX_START && COB_ID < PDO1_RX_END) {
		return COB_ID - PDO1_RX_START; 
	}

	if(COB_ID > PDO2_TX_START && COB_ID < PDO2_TX_END) {
		return COB_ID - PDO2_TX_START; 
	}
	if(COB_ID > PDO2_RX_START && COB_ID < PDO2_RX_END) {
		return COB_ID - PDO2_RX_START; 
	}

	if(COB_ID > PDO3_TX_START && COB_ID < PDO3_TX_END) {
		return COB_ID - PDO3_TX_START; 
	}
	if(COB_ID > PDO3_RX_START && COB_ID < PDO3_RX_END) {
		return COB_ID - PDO3_RX_START; 
	}

	if(COB_ID > PDO4_TX_START && COB_ID < PDO4_TX_END) {
		return COB_ID - PDO1_TX_START; 
	}
	if(COB_ID > PDO4_RX_START && COB_ID < PDO4_RX_END) {
		return COB_ID - PDO1_RX_START; 
	}
	
	if(COB_ID > SDO_TX_START && COB_ID < SDO_TX_END) {
		return COB_ID - SDO_TX_START; 
	}
	if(COB_ID > SDO_RX_START && COB_ID < SDO_RX_END) {
		return COB_ID - SDO_RX_START; 
	}
	
	if(COB_ID > HEARTBEAT_START && COB_ID < HEARTBEAT_END) {
		return COB_ID - HEARTBEAT_START; 
	}
	return -1;	
}

// Prints out the message received and pass message to appropriate
// handlers in the robot object.
void MaxonCANOpen::dataExtraction() {
	TPCANRdMsg readMsg;
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
}



