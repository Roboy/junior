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



#include "RoboyHugRolfBehaviour.h"

namespace Roboy
{
  
	RoboyHugRolfBehaviour::RoboyHugRolfBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities)
	{
		std::cout << "Roboy HugRolf Behaviour CONSTRUCTOR" << std::endl;
		currentStatus.str("HugRolf Behaviour Constructed");
		canBus = canBus_passed;
	}
	
    RoboyHugRolfBehaviour::~RoboyHugRolfBehaviour() 
    {
		
    }

    void
    RoboyHugRolfBehaviour::init()
    {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyHugRolfBehaviour INITIALIZATION." << std::endl;
		currentStatus.str("RoboyHugRolfBehaviour Behaviour INITIALIZATION");
            
    }

    void 
    RoboyHugRolfBehaviour::execute()
    {         
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyHugRolfBehaviour EXECUTING" << std::endl;
		currentStatus.str("RoboyHugRolfBehaviour Behaviour EXECUTING");

    textToSpeech->setText("Rolf.");
    textToSpeech->start();
	  face->init("speak");
    usleep(500000);
    face->backToNormalFast();

    while(textToSpeech->isActive()) usleep(50000);
    textToSpeech->setText("Hey Rolf.");
    textToSpeech->start();
	  face->init("speak");
    usleep(1200000);
    face->backToNormalFast();


		// move from Arms Normal to Arms stretched AND lay head to the side
		bodyMovement->setFolderToPlay("GazingBothHands_3a");
    bodyMovement->start();

    usleep(5500000); // wait, because it takes some time until the movement starts

    while (canBus->readDigitalInput(25, 0) == false) {
			usleep(100000);
      }
    
    textToSpeech->setText("Rolf. Please hug me. I want to thank you");
    textToSpeech->start();
	  face->init("speak");
    usleep(3400000);
    face->backToNormalFast();

    face->init("kiss");

    usleep(4000000);

    bodyMovement->moveHead(3, 7, 0, true); // (roll, pitch, yaw, blocking)

		usleep(1000000);

		bodyMovement->setFolderToPlay("GazingBothHands_3c");
    bodyMovement->start();

    face->backToNormalFast();
    
		// turn head to neutral position, take hands back

    bodyMovement->moveHead(0, 0, 0, false); // (roll, pitch, yaw, blocking)


    //		face->backToNormalFast();

		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}

    std::string str = "I think I will call you daddy.";
    textToSpeech->setText(str);
    textToSpeech->start();

		face->init("speak");
    usleep(str.length()*80000);
    
    face->backToNormalFast();

    while(textToSpeech->isActive()) usleep(50000);

    str = "I would like to give you really nice anniversary cake.";
    textToSpeech->setText(str);
    textToSpeech->start();

		face->init("speak");
    usleep(str.length()*80000);
    
    face->backToNormalFast();

    while(textToSpeech->isActive()) usleep(50000);

    str = "I did not bake it myself, but I organized it for you. Congratulations!";
    textToSpeech->setText(str);
    textToSpeech->start();

		face->init("speak");
    usleep(str.length()*80000);
    
    face->backToNormalFast();
    

    //textToSpeech->setText("Hello and welcome to the Lab!");
    //face->init("speak");
    //textToSpeech->start();
    //while(textToSpeech->isActive()) usleep(50000);
    //face->backToNormalFast();
    

		currentStatus.str("HugRolf Behaviour Completed"); 

		isBehaviourFinished = true;
    }

    void
    RoboyHugRolfBehaviour::terminate()
    {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyHugRolfBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyHugRolfBehaviour::getInfo()
    {
      return "RoboyHugRolfBehaviour";      
    }

}
