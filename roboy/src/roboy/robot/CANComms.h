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
#ifndef _CAN_COMMS_
#define _CAN_COMMS_
#include "GlobalParams.h"
#include "fileLogger.h"
#include "PauseTimer.h"
#include "genFunc.h"

#include <fcntl.h>
#include <errno.h>
#include <stddef.h>
#include <libpcan.h>

#define DISCONNECTED 0
#define CONNECTED 1

#define SEND_SUCCESSFUL 0

#define ERROR_NOT_CONNECTED -2
#define ERROR_DATA_LEN_INVALID -3
#define ERROR_LINE_NOT_OPEN -5

#define COMMS_SAMPLING_RATE_N 10000000

#define NO_DATA_READ CAN_ERR_QRCVEMPTY

// CANOpen information
#define NUM_BYTES_IN_EXTENDED_MESSAGE 7

#define INITIATE_DOMAIN_UPLOAD 2
#define INITIATE_DOMAIN_DOWNLOAD 3
#define DOWNLOAD_DOMAIN_SEGMENT 1
#define UPLOAD_DOMAIN_SEGMENT 0

// Client indicates the side that is sending
// In most cases this means the CPU side
#define INITIATE_DOMAIN_UPLOAD_CLIENT 2
#define INITIATE_DOMAIN_DOWNLOAD_CLIENT 1
#define UPLOAD_DOMAIN_SEGMENT_CLIENT 3
#define DOWNLOAD_DOMAIN_SEGMENT_CLIENT 0

// Size definition
#define INTEGER8 3
#define INTEGER16 2
#define INTEGER24 1
#define INTEGER32 0
#define UNSIGNED8 3
#define UNSIGNED16 2
#define UNSIGNED32 0

#define COMMS_LOG_FILE_NAME (const char *) "CommsLogFile.txt"
#define COMMS_CONSOLIDATE_LOG_FILE_NAME (const char *) "CommsConsolidatedLogFile.html"

#define TRACE_LOG_CONSOLIDATED_LN(a...) do{{TRACE_TIME(consolidatedFileIndex); TRACE_LOC_LOG(consolidatedFileIndex, a);TRACE_LOG(consolidatedFileIndex, "<BR>\n");}}while(0)
#define TRACE_LOG_CONSOLIDATED(a...) do{{TRACE_LOG(consolidatedFileIndex, a);}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME(a...) do{{TRACE_TIME(consolidatedFileIndex);TRACE_LOG(consolidatedFileIndex, a);}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_LOC(a...) do{{TRACE_TIME(consolidatedFileIndex);TRACE_LOC_LOG(consolidatedFileIndex, a);}}while(0)

#define TRACE_LOG_CONSOLIDATED_LN_RED_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_RED_BOLD); TRACE_TIME(consolidatedFileIndex); TRACE_LOC_LOG(consolidatedFileIndex, a);TRACE_LOG(consolidatedFileIndex, "</font></b><BR>\n");}}while(0)
#define TRACE_LOG_CONSOLIDATED_RED_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_RED_BOLD); TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_RED_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_RED_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_LOC_RED_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_RED_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOC_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)

#define TRACE_LOG_CONSOLIDATED_LN_BLUE_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_BLUE_BOLD); TRACE_TIME(consolidatedFileIndex); TRACE_LOC_LOG(consolidatedFileIndex, a);TRACE_LOG(consolidatedFileIndex, "</font></b><BR>\n");}}while(0)
#define TRACE_LOG_CONSOLIDATED_BLUE_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_BLUE_BOLD); TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_BLUE_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_BLUE_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_LOC_BLUE_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_BLUE_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOC_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)

#define TRACE_LOG_CONSOLIDATED_LN_CYAN_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_CYAN_BOLD); TRACE_TIME(consolidatedFileIndex); TRACE_LOC_LOG(consolidatedFileIndex, a);TRACE_LOG(consolidatedFileIndex, "</font></b><BR>\n");}}while(0)
#define TRACE_LOG_CONSOLIDATED_CYAN_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_CYAN_BOLD); TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_CYAN_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_CYAN_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_LOC_CYAN_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_CYAN_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOC_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)

#define TRACE_LOG_CONSOLIDATED_LN_GREEN_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_GREEN_BOLD); TRACE_TIME(consolidatedFileIndex); TRACE_LOC_LOG(consolidatedFileIndex, a);TRACE_LOG(consolidatedFileIndex, "</font></b><BR>\n");}}while(0)
#define TRACE_LOG_CONSOLIDATED_GREEN_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_GREEN_BOLD); TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_GREEN_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_GREEN_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_LOC_GREEN_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_GREEN_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOC_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)

#define TRACE_LOG_CONSOLIDATED_LN_PURP_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_PURP_BOLD); TRACE_TIME(consolidatedFileIndex); TRACE_LOC_LOG(consolidatedFileIndex, a);TRACE_LOG(consolidatedFileIndex, "</font></b><BR>\n");}}while(0)
#define TRACE_LOG_CONSOLIDATED_PURP_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_PURP_BOLD); TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_PURP_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_PURP_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)
#define TRACE_LOG_CONSOLIDATED_TIME_LOC_PURP_BOLD(a...) do{{TRACE_LOG(consolidatedFileIndex, HTML_PURP_BOLD);TRACE_TIME(consolidatedFileIndex);TRACE_LOC_LOG(consolidatedFileIndex, a); TRACE_LOG(consolidatedFileIndex, "</font></b>");}}while(0)

#define TRACE_LOG_COMMS_TX(a...) do{{TRACE_LOG(commsLogFileIndex, a);TRACE_LOG_CONSOLIDATED_PURP_BOLD(a);}}while(0)
#define TRACE_LOG_COMMS_LN_TX(a...) do{{TRACE_TIME(commsLogFileIndex); TRACE_LOC_LOG(commsLogFileIndex, a);TRACE_LOG(commsLogFileIndex, "\n"); TRACE_LOG_CONSOLIDATED_LN_PURP_BOLD(a);}}while(0)
#define TRACE_LOG_COMMS_TIME_TX(a...) do{{TRACE_TIME(commsLogFileIndex);TRACE_LOG(commsLogFileIndex, a);TRACE_LOG_CONSOLIDATED_TIME_PURP_BOLD(a);}}while(0)

#define TRACE_LOG_COMMS_RX(a...) do{{TRACE_LOG(commsLogFileIndex, a);TRACE_LOG_CONSOLIDATED_BLUE_BOLD(a);}}while(0)
#define TRACE_LOG_COMMS_LN_RX(a...) do{{TRACE_TIME(commsLogFileIndex); TRACE_LOC_LOG(commsLogFileIndex, a);TRACE_LOG(commsLogFileIndex, "\n"); TRACE_LOG_CONSOLIDATED_LN_BLUE_BOLD(a);}}while(0)
#define TRACE_LOG_COMMS_TIME_RX(a...) do{{TRACE_TIME(commsLogFileIndex);TRACE_LOG(commsLogFileIndex, a);TRACE_LOG_CONSOLIDATED_TIME_BLUE_BOLD(a);}}while(0)

#define TRACE_LOG_COMMS_END_LINE do{{TRACE_LOG(commsLogFileIndex, "\n"); TRACE_LOG_CONSOLIDATED("<BR>\n");}}while(0)

struct ObjDictionaryParam {
	int address, subIndex, dataSize;
};

class CANLine {
  private:
			HANDLE CANHandle;
			int status;
  public:
  	CANLine();
  	~CANLine();

		int connect(const char *deviceName, WORD bitRate = CAN_BAUD_1M, int nCANMsgType = CAN_INIT_TYPE_ST);
		TPCANMsg convertToTPCANMsg(int id, char *buf, int len, int msgType);
		void disconnect();
		int write(TPCANMsg *sendMsg, int timeOut);
		int write(int id, char *buf, int len, int timeOut, int msgType = MSGTYPE_STANDARD);
		int read(TPCANRdMsg *readMsg, int timeOut);
};

class CANDataThread : public fileLogger{
  private:
		/* pThread 
		pthread_t thread_ID;
		static void *threadStarter(void *a);
		*/
		boost::thread* readThread;
		void threadStarter();
		
	protected:
  	CANLine line;
		bool lineOpen;
		int startTime;
		int commsLogFileIndex;
		int consolidatedFileIndex;
		
  public:
		CANDataThread();
		
		bool initialize(const char *deviceName, WORD bitRate = CAN_BAUD_1M, int nCANMsgType = CAN_INIT_TYPE_ST);

		// Default timeout of 10 millisecond
		int transmit(TPCANMsg *sendMsg, int timeOutUS = 30000);
		int transmit(int id, char *buf, int len, int timeOutUS = 30000, int msgType = MSGTYPE_STANDARD);

		void traceCommsTX(TPCANMsg msg);
		void traceCommsRX(TPCANMsg msg);

		void uninitialize();

  protected:
		virtual void readingThread() = 0;

};



#endif
