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



#include "RoboyTestLegSwingBehaviour.h"

namespace Roboy {
	RoboyTestLegSwingBehaviour::RoboyTestLegSwingBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities) {
		std::cout << "Roboy Test Leg Swing Behaviour CONSTRUCTOR" << std::endl;
		currentStatus.str("Test Leg Swing Behaviour Constructed");
		canBus = canBus_passed;
	}

  void RoboyTestLegSwingBehaviour::init() {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		currentStatus.str("Test Leg Swing Behaviour Initialized");
  }

  void RoboyTestLegSwingBehaviour::execute() {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the execute() we have to say which bodyActivities should be played.
		std::cout << "Test Leg Swing Behaviour EXECUTING" << std::endl;
		currentStatus.str("Test Leg Swing Behaviour Executing");

    bodyMovement->setLegSwingBehavior(10000000, 3000, 4000, 100000, 60, 3);
    
    usleep(15000000);
    bodyMovement->setLegSwingBehavior(0, 2000, 3000, 0, 60, 0);
    usleep(5000000);

		currentStatus.str("Test Leg Swing Behaviour Completed"); 

		isBehaviourFinished = true;
  }

  void RoboyTestLegSwingBehaviour::terminate() {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyGiveHandBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyTestLegSwingBehaviour::getInfo()
    {
      return currentStatus.str();      
    }

}
