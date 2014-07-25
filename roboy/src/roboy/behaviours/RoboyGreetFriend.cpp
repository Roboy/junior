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



#include "RoboyGreetFriend.h"

namespace Roboy
{

RoboyGreetFriend::RoboyGreetFriend(std::vector<RoboyBodyActivity*> bodyActivities): RoboyBehaviour(bodyActivities)
{

}

RoboyGreetFriend::~RoboyGreetFriend()
{

}

void
RoboyGreetFriend::init()
{
	std::cout << " GREETING FRIEND INIT " << std::endl;

//	bodyActivities[RoboyBodyActivity::TEXT_TO_SPEECH]->init("OLA HUGO");
//	std::cout << " GREETING FRIEND INIT end" << std::endl;

}

void
RoboyGreetFriend::execute()
{
	std::cout << " GREETING FRIEND EXECUTE " << std::endl;
	bodyActivities[RoboyBodyActivity::TEXT_TO_SPEECH]->start();

	while(!bodyActivities[RoboyBodyActivity::TEXT_TO_SPEECH]->isFinished())
	{
		usleep(10000);
	}

	isBehaviourFinished = true;
}

void
RoboyGreetFriend::terminate()
{

	//      alutExit ();
	bodyActivities[RoboyBodyActivity::TEXT_TO_SPEECH]->stop();
	std::cout << " GREETING FRIEND STOP " << std::endl;
}


std::string
RoboyGreetFriend::getInfo()
{
	return "Greeting Friend";
}

}
