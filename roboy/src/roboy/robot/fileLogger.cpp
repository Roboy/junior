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

#include "fileLogger.h"

struct timeval logStartTime;
bool folderCreated = false;
std::string fileLoggerFolderName;

fileLogger::fileLogger() {
	std::stringstream ss;
	time_t tm = time(0);
	char time_string[15];
	int i;
	struct tm *ttime;
  struct stat st;
	
	curLogNum = 0;
	for(i = 0; i < MAX_LOG_FILES; i++) {
		logFile[i] = NULL;
		fileType[i] = PLAIN_TEXT;
	}
	if(folderCreated) return;
	folderCreated = true;
	
	ttime = localtime(&tm);
	strftime(time_string,15,"%Y%m%d%H%M%S",ttime); 	

  if(stat(LOG_FILES_FOLDER_ABSOLUTE_PATH, &st) == 0) {
    if(st.st_mode & S_IFDIR != 0) {
      printf(" %s is present\n", LOG_FILES_FOLDER_ABSOLUTE_PATH);	
		} else {
      printf("[%s] st_mode value: %d\n",  LOG_FILES_FOLDER_ABSOLUTE_PATH, st.st_mode);	
    }
  } else {
    mkdir(LOG_FILES_FOLDER_ABSOLUTE_PATH, S_IRWXU|S_IRGRP|S_IXGRP);
  }
	ss << LOG_FILES_FOLDER_ABSOLUTE_PATH << LOGGER_FOLDER_PREFIX << time_string;
	mkdir(ss.str().c_str(), S_IRWXU|S_IRGRP|S_IXGRP);

	fileLoggerFolderName = ss.str();
	
	gettimeofday(&logStartTime, NULL);
	
}

int fileLogger::addLogFile(const char *fileName, int type) {
	std::stringstream ss;
	
	if(curLogNum >= MAX_LOG_FILES) return ERROR_EXCEED_MAX_LOG_FILES;
	ss << fileLoggerFolderName << "/" << fileName;
	
	logFile[curLogNum] = fopen(ss.str().c_str(),"w");
	if(logFile[curLogNum] == NULL) return ERROR_CANNOT_OPEN_FILE;
	
	fileType[curLogNum] = type;
	if(type == HTML_TEXT) {
		fprintf(logFile[curLogNum], "<html>\n<head>\n</head>\n<body>\n<p style=\"font-size:small;\">\n");
		fflush(logFile[curLogNum]);
	}
	
	curLogNum++;
	return curLogNum - 1;
}

fileLogger::~fileLogger() {
	int i;
	for(i = 0; i < MAX_LOG_FILES; i++) {
		if(logFile[i] != NULL) {
			if(fileType[i] == HTML_TEXT) {
				fprintf(logFile[i], "\n</p>\n</body>\n</html>\n");
				fflush(logFile[i]);
			}
			fclose(logFile[i]);
		}
	}
}

