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

#ifndef _ROBOY_STARMIND_CONNECTOR_H_
#define _ROBOY_STARMIND_CONNECTOR_H_


#include <string>
#include <list>
#include <queue>
#include <boost/thread.hpp>
#include "RoboyVision.h"

namespace Roboy {

class RoboyStarmindConnector
{

public:

	RoboyStarmindConnector(RoboyVision* vision);

	virtual ~RoboyStarmindConnector();

	void
	start();

	void
	stop();

	bool
	isStarmindConnected();

//	std::queue<std::string>
//	getTextQueue();

	std::string
	getNextText();

	char
	getNextMode();

	int*
	getNextHead();

	std::string
	getNextFacial();

	std::string
	getNextBody();

protected:

	bool isConnectorActive;
	bool runConnector;
	bool isStarmindActive;

	boost::thread* starmindConnectorThread;

private:

	RoboyVision* vision;
	std::queue<std::string> textes;
	std::queue<char> modes;
	std::queue<int*> heads;
	std::queue<std::string> facials;
	std::queue<std::string> bodys;
	int port;

	void
	executeConnector();

	std::string
	execCmd(char* cmd);

};

}

#endif  // _ROBOY_STARMIND_CONNECTOR_H_
