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
#include <boost/filesystem.hpp>
#include "CANComms.h"
#include "debug.h"

CANLine::CANLine() {
	status = DISCONNECTED;
}

CANLine::~CANLine() {
	CAN_Close(CANHandle);
}

int CANLine::connect(const char *deviceName, WORD bitRate, int nCANMsgType) {
    // Should normally point to deviceName but can point to fallbacks
	const char *deviceNamePointer = deviceName;
	deviceNamePointer = deviceName;
	if(!boost::filesystem::exists(deviceName) && boost::filesystem::exists("/dev/pcan0"))
		deviceNamePointer = "/dev/pcan0";
	else if(!boost::filesystem::exists(deviceName) && boost::filesystem::exists("/dev/pcan32"))
		deviceNamePointer = "/dev/pcan32";
	CANHandle = LINUX_CAN_Open(deviceNamePointer, O_RDWR);
	
	if (!CANHandle) {
		printf("Cannot open [%s]\n", deviceNamePointer);
		status = DISCONNECTED;
	} else {
		status = CONNECTED;
		CAN_Init(CANHandle, bitRate, nCANMsgType);
	}
	
	return status;
}

TPCANMsg CANLine::convertToTPCANMsg(int id, char *buf, int len, int msgType) {
	TPCANMsg sendMsg;
	int i;

	if(len <= 0 || len > 8) {
		return sendMsg;
	}
	
	sendMsg.ID = id;
	sendMsg.MSGTYPE = msgType;
	sendMsg.LEN = len;
	
	for(i = 0; i < len; i++) {
		sendMsg.DATA[i] = buf[i];
	}
	return sendMsg;
}


int CANLine::write(int id, char *buf, int len, int timeOut, int msgType) {
	TPCANMsg sendMsg;
	sendMsg = convertToTPCANMsg(id, buf, len, msgType);

	return write(&sendMsg, timeOut);
}

int CANLine::write(TPCANMsg *sendMsg, int timeOut) {

	if(status != CONNECTED) {
		return ERROR_NOT_CONNECTED;
	}

	return LINUX_CAN_Write_Timeout(CANHandle, sendMsg, timeOut);
}


int CANLine::read(TPCANRdMsg *readMsg, int timeOut) {

	if(status != CONNECTED) {
		return ERROR_NOT_CONNECTED;
	}
	
	return LINUX_CAN_Read_Timeout(CANHandle, readMsg, timeOut);

}

void CANLine::disconnect() {
	if(status == CONNECTED) {
		CAN_Close(CANHandle);
		status = DISCONNECTED;
	}
}

CANDataThread::CANDataThread() : fileLogger(){
	commsLogFileIndex = addLogFile(COMMS_LOG_FILE_NAME);
	consolidatedFileIndex = addLogFile(COMMS_CONSOLIDATE_LOG_FILE_NAME, HTML_TEXT);
	lineOpen = false;
}

int CANDataThread::transmit(TPCANMsg *sendMsg, int timeOutUS) {
	if(!lineOpen) return ERROR_LINE_NOT_OPEN;
	traceCommsTX(*sendMsg);
	return line.write(sendMsg, timeOutUS);
}

int CANDataThread::transmit(int id, char *buf, int len, int timeOutUS, int msgType) {
	TPCANMsg sendMsg;
	if(!lineOpen) return ERROR_LINE_NOT_OPEN;
	sendMsg = line.convertToTPCANMsg(id, buf, len, msgType);
	
	return line.write(&sendMsg, timeOutUS);
}

bool CANDataThread::initialize(const char *deviceName, WORD bitRate, int nCANMsgType) {

  if(line.connect(deviceName,bitRate,nCANMsgType) == DISCONNECTED) {
		lineOpen = false;
    printf("Error in opening line.\n");
    return lineOpen;
  }		
	lineOpen = true;
	
	// pThread
	//pthread_create(&thread_ID, NULL, threadStarter, (void *) this);
	threadStarter();
	
	return lineOpen;
}

void CANDataThread::uninitialize() {
	// pThread
	// void *exit_status;
	lineOpen = false;
	
	/* pthread
	pthread_join(thread_ID, &exit_status);
	*/
	
	readThread->join();
	
	line.disconnect();
}

void CANDataThread::traceCommsTX(TPCANMsg msg) {
	int i;
	TRACE_LOG_COMMS_TIME_TX("Send:\t\t");
	TRACE_LOG_COMMS_TX(" [0x%03x]--(%d): ", msg.ID, msg.LEN);
	for(i = 0; i < msg.LEN; i++) {
		TRACE_LOG_COMMS_TX("-[0x%02x]", msg.DATA[i]);
	}
	TRACE_LOG_COMMS_END_LINE;
}

void CANDataThread::traceCommsRX(TPCANMsg msg) {
	int i;
	TRACE_LOG_COMMS_TIME_RX("Receive:\t\t");
	TRACE_LOG_COMMS_RX(" [0x%03x]--(%d): ", msg.ID, msg.LEN);
	for(i = 0; i < msg.LEN; i++) {
		TRACE_LOG_COMMS_RX("-[0x%02x]", msg.DATA[i]);
	}
	TRACE_LOG_COMMS_END_LINE;
}
/* pthread
void *CANDataThread::threadStarter(void *a) {
	CANDataThread *b = (CANDataThread *)a;

	b->readingThread();
	return NULL;
}
*/

void CANDataThread::threadStarter() {
	readThread = new boost::thread(&CANDataThread::readingThread, this);
}


