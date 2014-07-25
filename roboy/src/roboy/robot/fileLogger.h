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
#ifndef _FILE_LOGGER_
#define _FILE_LOGGER_

#include "GlobalParams.h"
#define MAX_LOG_FILES 10
#include <sys/stat.h>
#include <time.h>
#if PLATFORM_TYPE==LINUX
	#include <sys/time.h>
#endif

#define ERROR_EXCEED_MAX_LOG_FILES -1
#define ERROR_CANNOT_OPEN_FILE -2

#define PLAIN_TEXT 1
#define HTML_TEXT 2

#define LOG_FILES_FOLDER_ABSOLUTE_PATH "/tmp/roboy/"

#define HTML_RED "<font color=\"ff0000\">"
#define HTML_PURP "<font color=\"990099\">"
#define HTML_BLUE "<font color=\"000099\">"
#define HTML_GREEN "<font color=\"009900\">"
#define HTML_CYAN "<font color=\"00ffff\">"

#define HTML_RED_BOLD "<b><font color=\"ff0000\">"
#define HTML_PURP_BOLD "<b><font color=\"990099\">"
#define HTML_BLUE_BOLD "<b><font color=\"000099\">"
#define HTML_GREEN_BOLD "<b><font color=\"009900\">"
#define HTML_CYAN_BOLD "<b><font color=\"00aaaa\">"

#define LOGGER_FOLDER_PREFIX (const char *) "LogFilesFolder"

// Currently does not account for day change
#define TRACE_TIME(i) do{{ if(i >= 0 && i < MAX_LOG_FILES) {if(logFile[i] != NULL) {gettimeofday(&logCurTime, NULL); fprintf(logFile[i], "%.6f:", (logCurTime.tv_sec - logStartTime.tv_sec) + ((float) logCurTime.tv_usec - logStartTime.tv_usec)/1000000.0); }}}}while(0)
#define TRACE_LOG(i, a...) do{{ if(i >= 0 && i < MAX_LOG_FILES) {if(logFile[i] != NULL) { fprintf(logFile[i], a); }}}}while(0)
#define TRACE_LOC_LOG(i, a...) do{{ if(i >= 0 && i < MAX_LOG_FILES) {if(logFile[i] != NULL) {fprintf(logFile[i], "%s:%d: In Function %s(): ", __FILE__, __LINE__, __FUNCTION__); fprintf(logFile[i], a);}}}}while(0)

extern struct timeval logStartTime;

class fileLogger {
	protected:
		FILE *logFile[MAX_LOG_FILES];
		int fileType[MAX_LOG_FILES];
		int curLogNum;
		struct timeval logCurTime;

	public:	
		fileLogger();
		int addLogFile(const char *fileName, int type = PLAIN_TEXT);
		~fileLogger();
};


#endif
