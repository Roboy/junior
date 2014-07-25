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



#include "RoboyGiveHandBehaviour.h"

namespace Roboy
{
  
	RoboyGiveHandBehaviour::RoboyGiveHandBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities)
	{
	std::cout << "RoboyGiveHandBehaviour CONSTRUCTOR" << std::endl;
	canBus = canBus_passed;
	}
	
    RoboyGiveHandBehaviour::~RoboyGiveHandBehaviour() 
    {

    }

    void
    RoboyGiveHandBehaviour::init()
    {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		     
    }

    void 
    RoboyGiveHandBehaviour::execute()
    {         
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyGiveHandBehaviour EXECUTING" << std::endl;

		// LATER: here not only one bodyActivity has to be started! Also headMovement, facialExpression, etc!

		((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->setFolderToPlay("GivingHand_5");
    bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->start();

		// LATER: here not only one bodyActivity has to be tested for beeing finished! Also headMovement, facialExpression, etc!
		while(!bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isFinished()){
			usleep(10000);
		}
		// wait for pressure Sensor before Closing the Hand
		pressureSensorHandPushed_right = false;
		while(pressureSensorHandPushed_right == false){
			usleep(10000);
			pressureSensorHandPushed_right = canBus->readDigitalInput(25, 0);
		}

		// Close Right Hand
		((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->moveHand(false, true, true); // left hand = false, close = true, blocking (=waiting until finished) = true

		// LATER: here not only one bodyActivity has to be tested for beeing finished! Also headMovement, facialExpression, etc!

		((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->setFolderToPlay("ShakingHand_5");
    bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->start();

		// LATER: here not only one bodyActivity has to be tested for beeing finished! Also headMovement, facialExpression, etc!
		while(!bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isFinished()){
			usleep(10000);
		}

		// wait for pressure Sensor being untouched before releasing the hand
		pressureSensorHandPushed_right = true;
		while(pressureSensorHandPushed_right == true){
			usleep(10000);
			pressureSensorHandPushed_right = canBus->readDigitalInput(25, 0);
		}
		
		// Open right hand
		((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->moveHand(false, false, true); // left hand = false, close = false, blocking (=waiting until finished) = true

		// LATER: here not only one bodyActivity has to be tested for beeing finished! Also headMovement, facialExpression, etc!

		((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->setFolderToPlay("DrawingBack_5");
    bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->start();

		// LATER: here not only one bodyActivity has to be tested for beeing finished! Also headMovement, facialExpression, etc!
		while(!bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isFinished()){
			usleep(10000);
		}


		isBehaviourFinished = true;
    }

    void
    RoboyGiveHandBehaviour::terminate()
    {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyGiveHandBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyGiveHandBehaviour::getInfo()
    {
      return "RoboyGiveHandBehaviour";      
    }

}
