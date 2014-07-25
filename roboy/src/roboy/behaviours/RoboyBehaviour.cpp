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

#include "RoboyBehaviour.h"

namespace Roboy
{

  RoboyBehaviour::RoboyBehaviour(std::vector<RoboyBodyActivity*> bodyActivities) 
  {
    isBehaviourActive = false;
    isBehaviourFinished = true;
    
    this->bodyActivities = bodyActivities;
    
    speech = dynamic_cast<RoboySpeechProduction*>(bodyActivities[RoboyBodyActivity::SPEECH_PRODUCTION]);
    face = dynamic_cast<RoboyFacialExpression*>(bodyActivities[RoboyBodyActivity::FACIAL_EXPRESSION]);
    std::cout << " OLE FROM BEHAVIOURS " << bodyActivities.size() << std::endl;
    textToSpeech = dynamic_cast<RoboyTextToSpeech*>(bodyActivities[RoboyBodyActivity::TEXT_TO_SPEECH]);
    bodyMovement = dynamic_cast<RoboyBodyMovement*> (bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]);
	listenToSound = dynamic_cast<RoboyListenToSound*> (bodyActivities[RoboyBodyActivity::LISTEN_TO_SOUND]);

    std::cout << "BEHAVIOUR INTERFACE CONSTRUCTOR" << std::endl;
  }


  RoboyBehaviour::~RoboyBehaviour() 
  {

  }

  void 
  RoboyBehaviour::start()
  {
    if(isBehaviourActive)
      return;

    init();
		isBehaviourFinished = false;
		isBehaviourActive = true;
    behaviourThread = new boost::thread( boost::bind(&RoboyBehaviour::execute, this) );      
  }


  void 
  RoboyBehaviour::stop()
  {
    if(!isBehaviourActive)
      return;

    behaviourThread->interrupt();
    terminate();
    isBehaviourActive = false;
    isBehaviourFinished = false; 	//FIXME: Not exactly right. Some Behaviours may run until they are finished when they are terminated. So, they ARE finished when terminated.
  }


  bool
  RoboyBehaviour::isActive()
  {
    return isBehaviourActive;
  }

  bool
  RoboyBehaviour::isFinished()
  {
    return isBehaviourFinished;
  }

  bool
  RoboyBehaviour::allActivitiesFinished()
  {
  
	  for(int i=0; i<bodyActivities.size(); i++)
		  if(!bodyActivities[i]->isFinished())
			  return false;
	  
	  return true;
 
  } 
  
  
  
}
