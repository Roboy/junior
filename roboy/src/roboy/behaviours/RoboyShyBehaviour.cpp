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



#include "RoboyShyBehaviour.h"

namespace Roboy
{
  
	RoboyShyBehaviour::RoboyShyBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities)
	{
		std::cout << "Roboy Shy Behaviour CONSTRUCTOR" << std::endl;
		currentStatus.str("Shy Behaviour Constructed");
		canBus = canBus_passed;
	}
	
    RoboyShyBehaviour::~RoboyShyBehaviour() 
    {
		
    }

    void
    RoboyShyBehaviour::init()
    {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyShyBehaviour INITIALIZATION." << std::endl;
		currentStatus.str("RoboyShyBehaviour Behaviour INITIALIZATION");
            
    }

    void 
    RoboyShyBehaviour::execute()
    {         
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyShyBehaviour EXECUTING" << std::endl;
		currentStatus.str("RoboyShyBehaviour Behaviour EXECUTING");
    

    bodyMovement->moveHead(0, 0, 10, true); // (roll, pitch, yaw, blocking)


    textToSpeech->setText("Oh");
    textToSpeech->start();
	  face->init("speak");
    usleep(300000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);

    bodyMovement->moveHead(4, 0, -10, true);

    textToSpeech->setText("I am confused");
    textToSpeech->start();
	  face->init("speak");
    usleep(1100000);
    face->backToNormalFast();

    while(textToSpeech->isActive()) usleep(50000);

    bodyMovement->moveHead(0, 0, 0, true);

    textToSpeech->setText("Why are all these humans here?");
    textToSpeech->start();
	  face->init("speak");
    usleep(1800000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);


    textToSpeech->setText("Maybe I am a little afraid");
    textToSpeech->start();
	  face->init("speak");
    usleep(1900000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    bodyMovement->setLegSwingBehavior(10000000, 3000, 4000, 100000, 120, 2);
    bodyMovement->moveHead(3, 8, -10, false);
    face->init("shy");
    usleep(8000000);
    bodyMovement->moveHead(0, 0, 10, true);
    usleep(1500000);
    bodyMovement->moveHead(0, 0, -10, true);
    usleep(1500000);
    bodyMovement->moveHead(3, 8, 10, true);
    face->init("shy");
    usleep(8000000);
    std::cout << "Keeping Head down to finish Shy Face" << std::endl; 
    //face->backToNormalFast();
    bodyMovement->moveHead(0, 0, 0, true);
    std::cout << "Moving Head back" << std::endl; 
		
    while(textToSpeech->isActive()) usleep(50000);
    std::cout << "Done with checking if textToSpeech is not active anymore" << std::endl;

    //textToSpeech->setText("Hello and welcome to the Lab!");
    //face->init("speak");
    //textToSpeech->start();
    //while(textToSpeech->isActive()) usleep(50000);
    //face->backToNormalFast();
    while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
      std::cout << "Waiting BodyMovement to finish" << std::endl; 
			usleep(100000);
		}

		currentStatus.str("Shy Behaviour Completed"); 

    bodyMovement->setLegSwingBehavior(0, 3000, 4000, 0, 120, 2);
		isBehaviourFinished = true;
    }

    void
    RoboyShyBehaviour::terminate()
    {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyShyBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyShyBehaviour::getInfo()
    {
      return "RoboyShyBehaviour";      
    }

}
