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



#include "RoboyComeToMeBehaviour.h"

namespace Roboy
{
  
	RoboyComeToMeBehaviour::RoboyComeToMeBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities)
	{
		std::cout << "Roboy ComeToMe Behaviour CONSTRUCTOR" << std::endl;
		currentStatus.str("ComeToMe Behaviour Constructed");
		canBus = canBus_passed;
	}
	
    RoboyComeToMeBehaviour::~RoboyComeToMeBehaviour() 
    {
		
    }

    void
    RoboyComeToMeBehaviour::init()
    {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyComeToMeBehaviour INITIALIZATION." << std::endl;
		currentStatus.str("RoboyComeToMeBehaviour Behaviour INITIALIZATION");
            
    }

    void 
    RoboyComeToMeBehaviour::execute()
    {         
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyComeToMeBehaviour EXECUTING" << std::endl;
		currentStatus.str("RoboyComeToMeBehaviour Behaviour EXECUTING");


		// move from Arms Normal to Arms stretched AND lay head to the side
		bodyMovement->setFolderToPlay("GazingBothHands_3a");
    bodyMovement->start();
    bodyMovement->setLegSwingBehavior(10000000, 3000, 4000, 100000, 120, 2);
    usleep(5500000); // wait, because it takes some time until the movement starts
    

    bodyMovement->moveHead(3, 7, 0, true); // (roll, pitch, yaw, blocking)

		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}

    textToSpeech->setText("Hey, come to me and take a look how I am built.");
    textToSpeech->start();
	  face->init("speak");

    bodyMovement->setLegSwingBehavior(10000000, 3000, 4000, 100000, 120, 2);

		usleep(2000000);

    bodyMovement->moveHead(0, 0, 0, false); // (roll, pitch, yaw, blocking)

    usleep(1500000);
    
    face->backToNormalFast();
    
		
		// turn head to neutral position, take hands back

		bodyMovement->setFolderToPlay("GazingBothHands_3c");
    bodyMovement->start();

	  usleep(3000000);

    //		face->backToNormalFast();

		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}

    textToSpeech->setText("I have 48 Motors.");
    textToSpeech->start();

		face->init("speak");
    usleep(1900000);
    
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);

    textToSpeech->setText("I am only two months old. I got my legs in February.");
    textToSpeech->start();

		face->init("speak");
    usleep(4250000);
    
    face->backToNormalFast();

    while(textToSpeech->isActive()) usleep(50000);

    textToSpeech->setText("I've got bones and muscles as humans. Most of my parts are 3 D printed. My creators are very proud of me.");
    textToSpeech->start();

		face->init("speak");
    usleep(8500000);
    
    face->backToNormalFast();
    

    //textToSpeech->setText("Hello and welcome to the Lab!");
    //face->init("speak");
    //textToSpeech->start();
    //while(textToSpeech->isActive()) usleep(50000);
    //face->backToNormalFast();
    

		currentStatus.str("ComeToMe Behaviour Completed"); 

    bodyMovement->setLegSwingBehavior(0, 3000, 4000, 0, 120, 2);
		isBehaviourFinished = true;
    }

    void
    RoboyComeToMeBehaviour::terminate()
    {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyComeToMeBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyComeToMeBehaviour::getInfo()
    {
      return "RoboyComeToMeBehaviour";      
    }

}
