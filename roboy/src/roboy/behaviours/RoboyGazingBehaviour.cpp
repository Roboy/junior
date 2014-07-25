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



#include "RoboyGazingBehaviour.h"

namespace Roboy
{
  
	RoboyGazingBehaviour::RoboyGazingBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities)
	{
		std::cout << "Roboy Gazing Behaviour CONSTRUCTOR" << std::endl;
		currentStatus.str("Gazing Behaviour Constructed");
		canBus = canBus_passed;
	}
	
    RoboyGazingBehaviour::~RoboyGazingBehaviour() 
    {
		
    }

    void
    RoboyGazingBehaviour::init()
    {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyGazingBehaviour INITIALIZATION." << std::endl;
		currentStatus.str("RoboyGazingBehaviour Behaviour INITIALIZATION");
            
    }

    void 
    RoboyGazingBehaviour::execute()
    {         
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyGazingBehaviour EXECUTING" << std::endl;
		currentStatus.str("RoboyGazingBehaviour Behaviour EXECUTING");


		// move from Arms Normal to Arms stretched AND Turn Head to left Hand
		bodyMovement->setFolderToPlay("GazingBothHands_3a");
    bodyMovement->start();
    bodyMovement->setLegSwingBehavior(10000000, 3000, 4000, 100000, 120, 2);
    usleep(5500000); // wait, because it takes some time until the movement starts
    

    bodyMovement->moveHead(0, 10, 10, true); // (roll, pitch, yaw, blocking)

		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}

    textToSpeech->setText("oooooo");
    textToSpeech->start();
	  face->init("surprise");

		usleep(4500000);

    textToSpeech->setText("I can feel my arms.");
    textToSpeech->start();

		face->init("speak");

		// Close both Hands
		///////////////////bodyMovement->moveHand(true, 600000, false); // (isLeftHand, value, blocking)
		bodyMovement->moveHand(false, 600000, false); // (isLeftHand, value, blocking)

		usleep(1500000);
    face->backToNormalFast();
		
		// Turn Head to right hand, turn fore Arms, Open Hands
		bodyMovement->moveHead(0, 15, -10, false); // (roll, pitch, yaw, blocking)


		bodyMovement->moveForeArm(true, -180000, 3000, 20000, false); // (isLeftArm, value, blocking)
		bodyMovement->moveForeArm(false, 180000, 3000, 20000, false); // (isLeftArm, value, blocking)

		face->init("smile");

		///////////////////bodyMovement->moveHand(true, 0, false); // (isLeftHand, value, blocking)
		bodyMovement->moveHand(false, 0, true); // (isLeftHand, value, blocking)

		// lift left hand up, turn head to it, turn back fore arm

		bodyMovement->setFolderToPlay("GazingBothHands_3b");
    bodyMovement->start();

		bodyMovement->moveHead(3, 15, 10, false); // (roll, pitch, yaw, blocking)

    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
		face->init("speak");
		//usleep(500000);

    textToSpeech->setText("uiiii, and I like my hands.");
    textToSpeech->start();

		bodyMovement->moveForeArm(true, 0, 3000, 20000, false); // (isLeftArm, value, blocking)
		bodyMovement->moveForeArm(false, 0, 3000, 20000, true); // (isLeftArm, value, blocking)

    face->backToNormalFast();

		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}

	  //usleep(2000000);

		// turn head to neutral position, take hands back

		bodyMovement->setFolderToPlay("GazingBothHands_3c");
    bodyMovement->start();

	  usleep(3000000);

    bodyMovement->moveHead(0, 0, 0, false); // (roll, pitch, yaw, blocking)

//		face->backToNormalFast();

		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}


    //textToSpeech->setText("Hello and welcome to the Lab!");
    //face->init("speak");
    //textToSpeech->start();
    //while(textToSpeech->isActive()) usleep(50000);
    //face->backToNormalFast();
    

		currentStatus.str("Gazing Behaviour Completed"); 

    bodyMovement->setLegSwingBehavior(0, 3000, 4000, 0, 120, 2);
		isBehaviourFinished = true;
    }

    void
    RoboyGazingBehaviour::terminate()
    {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyGazingBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyGazingBehaviour::getInfo()
    {
      return "RoboyGazingBehaviour";      
    }

}
