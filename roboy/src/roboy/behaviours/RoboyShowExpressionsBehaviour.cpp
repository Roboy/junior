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



#include "RoboyShowExpressionsBehaviour.h"

namespace Roboy
{
  
	RoboyShowExpressionsBehaviour::RoboyShowExpressionsBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities)
	{
		std::cout << "Roboy ShowExpressions Behaviour CONSTRUCTOR" << std::endl;
		currentStatus.str("ShowExpressions Behaviour Constructed");
		canBus = canBus_passed;
	}
	
    RoboyShowExpressionsBehaviour::~RoboyShowExpressionsBehaviour() 
    {
		
    }

    void
    RoboyShowExpressionsBehaviour::init()
    {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyShowExpressionsBehaviour INITIALIZATION." << std::endl;
		currentStatus.str("RoboyShowExpressionsBehaviour Behaviour INITIALIZATION");
            
    }

    void 
    RoboyShowExpressionsBehaviour::execute()
    {         
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyShowExpressionsBehaviour EXECUTING" << std::endl;
		currentStatus.str("RoboyShowExpressionsBehaviour Behaviour EXECUTING");


    bodyMovement->setLegSwingBehavior(10000000, 3000, 4000, 100000, 120, 2);
    

    bodyMovement->moveHead(0, 0, 10, true); // (roll, pitch, yaw, blocking)


    textToSpeech->setText("Hey!");
    textToSpeech->start();
	  face->init("speak");
    usleep(300000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);

    bodyMovement->moveHead(0, 0, -10, true);

    textToSpeech->setText("Hey! Listen.");
    textToSpeech->start();
	  face->init("speak");
    usleep(700000);
    face->backToNormalFast();

    while(textToSpeech->isActive()) usleep(50000);

    bodyMovement->moveHead(0, 0, 0, true);

    textToSpeech->setText("I want to show you how I can feel.");
    textToSpeech->start();
	  face->init("speak");
    usleep(2500000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);

    textToSpeech->setText("I can be happy");
    textToSpeech->start();
	  face->init("speak");
    usleep(1100000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    face->init("smile");
    usleep(3000000);
    face->backToNormalFast();
		while(textToSpeech->isActive()) usleep(50000);

    textToSpeech->setText("I can be shy");
    textToSpeech->start();
	  face->init("speak");
    usleep(1100000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    bodyMovement->moveHead(3, 8, -10, false);
    face->init("shy");
    usleep(6000000);
    face->backToNormalFast();
    bodyMovement->moveHead(0, 0, 0, false);
		while(textToSpeech->isActive()) usleep(50000);

    textToSpeech->setText("I can be surprised");
    textToSpeech->start();
	  face->init("speak");
    usleep(1300000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    textToSpeech->setText("ooo");
    textToSpeech->start();
    face->init("surprise");
    usleep(3000000);
    face->backToNormalFast();
		while(textToSpeech->isActive()) usleep(50000);

    textToSpeech->setText("I can be angry");
    textToSpeech->start();
	  face->init("speak");
    usleep(1200000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    bodyMovement->moveHead(0, 5, 0, false);
    face->init("angry");
    usleep(5000000);
    face->backToNormalFast();
    bodyMovement->moveHead(0, 0, 0, false);
		while(textToSpeech->isActive()) usleep(50000);

    textToSpeech->setText("I can blink with my eye");
    textToSpeech->start();
	  face->init("speak");
    usleep(1700000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    face->init("smileblink");
    bodyMovement->moveHead(4, 0, 0, true);
    bodyMovement->moveHead(0, 0, 0, true);
		while(textToSpeech->isActive()) usleep(50000);
    usleep(2000000);

    textToSpeech->setText("I can be tired and go to sleep.");
    textToSpeech->start();
	  face->init("speak");
    usleep(2200000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    bodyMovement->moveHead(0, 12, 0, false);
    face->init("sleep");
    usleep(5000000);
    face->backToNormalFast();
    bodyMovement->moveHead(0, 0, 0, true);
    while(textToSpeech->isActive()) usleep(50000);

    textToSpeech->setText("And I can give you a kiss");
    textToSpeech->start();
	  face->init("speak");
    usleep(1600000);
    face->backToNormalFast();
    while(textToSpeech->isActive()) usleep(50000);
    face->init("kiss");
    usleep(2000000);
    textToSpeech->setText("ma");
    textToSpeech->start();
    usleep(3000000);

		while(textToSpeech->isActive()) usleep(50000);
		
    

    //textToSpeech->setText("Hello and welcome to the Lab!");
    //face->init("speak");
    //textToSpeech->start();
    //while(textToSpeech->isActive()) usleep(50000);
    //face->backToNormalFast();
    

		currentStatus.str("ShowExpressions Behaviour Completed"); 

    bodyMovement->setLegSwingBehavior(0, 3000, 4000, 0, 120, 2);
		isBehaviourFinished = true;
    }

    void
    RoboyShowExpressionsBehaviour::terminate()
    {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyShowExpressionsBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyShowExpressionsBehaviour::getInfo()
    {
      return "RoboyShowExpressionsBehaviour";      
    }

}
