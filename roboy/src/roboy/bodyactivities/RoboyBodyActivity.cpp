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

#include <iostream>

#include "RoboyBodyActivity.h"

namespace Roboy
{

RoboyBodyActivity::RoboyBodyActivity() 
{
	isBodyActivityActive = false;
	isBodyActivityFinished = true;

	std::cout << "BODY ACTIVITY INTERFACE CONSTRUCTOR" << std::endl;

	activityThread = new boost::thread( boost::bind(&RoboyBodyActivity::execute, this) );

}


RoboyBodyActivity::~RoboyBodyActivity() 
{

}


// Note that body activities have to be initiated separately. This is to allow them all to start at the same time.

void 
RoboyBodyActivity::start()
{
	if(isBodyActivityActive)
		return;
	
	isBodyActivityActive = true;
	isBodyActivityFinished = false;
	//activityThread = new boost::thread( boost::bind(&RoboyBodyActivity::execute, this) );
}


void 
RoboyBodyActivity::stop()
{
	if(!isBodyActivityActive)
		return;

	activityThread->interrupt();
	terminate();
	isBodyActivityActive = false;
	isBodyActivityFinished = false; //FIXME: Not exactly right. Some (or all?) BodyActivities, when terminated, just run until they are finished. So, they are finished when terminated.
}


bool
RoboyBodyActivity::isActive()
{
	return isBodyActivityActive;
}

bool
RoboyBodyActivity::isFinished()
{
	return isBodyActivityFinished;
}

}
