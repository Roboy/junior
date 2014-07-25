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



#include "RoboyIntroductionBehaviour.h"

namespace Roboy {
	RoboyIntroductionBehaviour::RoboyIntroductionBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities) {
		std::cout << "Roboy Introduction Behaviour CONSTRUCTOR" << std::endl;
		currentStatus.str("Introduction Behaviour Constructed");
		canBus = canBus_passed;
	}

  void RoboyIntroductionBehaviour::init() {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		currentStatus.str("Introduction Behaviour Initialized");
  }

  void RoboyIntroductionBehaviour::execute() {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the execute() we have to say which bodyActivities should be played.
		std::cout << "Introduction Behaviour EXECUTING" << std::endl;
		currentStatus.str("Introduction Behaviour Executing");

    neckMotionCommand nMC;
  
    nMC.neckRollAngle = 0;
    nMC.neckPitchAngle = 0;
    nMC.neckYawAngle = 0;
    nMC.rollVelocity = 3000;
    nMC.pitchVelocity = 3000; 
    nMC.yawVelocity = 3000;
    nMC.rollAcceleration = 10000;
    nMC.pitchAcceleration = 10000; 
    nMC.yawAcceleration = 10000;
	


     // while(((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->canBus->readDigitalInput(25, 0) == false) {
		//	usleep(100000);
    // }
		// ((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->moveHand(true, true, true);
		// ((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->moveHand(true, false, true);

    bodyMovement->setLegSwingBehavior(10000000, 4000, 4000, 80000, 120, 2);
		bodyMovement->setFolderToPlay("Introduction");
    bodyMovement->start();
    sleep(5);
    nMC.neckYawAngle = -12;
    bodyMovement->moveHead(nMC, true);  
    nMC.neckYawAngle = 12;
    bodyMovement->moveHead(nMC, true);  
    nMC.neckYawAngle = -12;
    bodyMovement->moveHead(nMC, true);  
    nMC.neckYawAngle = 0;
    bodyMovement->moveHead(nMC, false);  

    textToSpeech->setText("Hello and welcome to the exhibition!");
    face->init("speak");
    textToSpeech->start();
    usleep(2000000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    sleep(2);
    textToSpeech->setText("I am roboy. It is a real pleasure to see you...");
    face->init("speak");
    textToSpeech->start();
    usleep(3500000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    sleep(1);
    textToSpeech->setText("Have you already seen me on the theater?");
    face->init("speak");
    textToSpeech->start();
		currentStatus.str("Waiting on Speech"); 
    usleep(2500000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
		currentStatus.str("Waiting on Motion"); 

		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}

		bodyMovement->setFolderToPlay("BackToNormal");
    bodyMovement->start();


		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}
		currentStatus.str("Introduction Behaviour Completed"); 

    bodyMovement->setLegSwingBehavior(0, 4000, 4000, 0, 120, 2);
		isBehaviourFinished = true;
  }

  void RoboyIntroductionBehaviour::terminate() {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyGiveHandBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyIntroductionBehaviour::getInfo()
    {
      return currentStatus.str();      
    }

}
