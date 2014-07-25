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



#include "RoboyGiveCloseShakeOpenDrawbackBehaviour.h"

namespace Roboy
{
  
	RoboyGiveCloseShakeOpenDrawbackBehaviour::RoboyGiveCloseShakeOpenDrawbackBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed): RoboyBehaviour(bodyActivities)
	{
	std::cout << "Roboy GiveCloseShakeOpenDrawback Behaviour CONSTRUCTOR" << std::endl;
		currentStatus.str("GiveCloseShakeOpenDrawback Behaviour Constructed");
		canBus = canBus_passed;
    initSentences();
    srand(time(NULL));
	}
	
    RoboyGiveCloseShakeOpenDrawbackBehaviour::~RoboyGiveCloseShakeOpenDrawbackBehaviour() 
    {

    }

    void
    RoboyGiveCloseShakeOpenDrawbackBehaviour::init()
    {
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyGiveCloseShakeOpenDrawbackBehaviour INITIALIZATION." << std::endl;
		currentStatus.str("RoboyGiveCloseShakeOpenDrawbackBehaviour Behaviour INITIALIZATION");
            
    }

    void 
    RoboyGiveCloseShakeOpenDrawbackBehaviour::execute()
    {         
      int counter;
    bodyMovement->setLegSwingBehavior(0, 4000, 4000, 0, 120, 2);
		// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
		// Therefore in the init() and execute() we have to say which bodyActivities should be played.
		std::cout << "RoboyGiveCloseShakeOpenDrawbackBehaviour EXECUTING" << std::endl;
		currentStatus.str("RoboyGiveCloseShakeOpenDrawbackBehaviour Behaviour EXECUTING");

		int wait_time = 2500000;

		// Say Hi
		 //textToSpeech->setText("Hey You! Give me your hand!");
      std::string sentence = introSentences[rand() % introSentences.size()];
textToSpeech->setText(sentence);
    face->init("speak");
    textToSpeech->start();
   // usleep(2200000);
      
    usleep(sentence.length()*80000);
    face->backToNormalFast();

    // Give Hand
		bodyMovement->setFolderToPlay("GivingHand_5");
    bodyMovement->start();

   // int remaining_time = sentence.length()*80000 - wait_time;

    //if (remaining_time > 0)
    //  usleep(remaining_time);




		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}
		// wait on sensor
    counter = 0;
		while (canBus->readDigitalInput(25, 0) == false) {
			usleep(100000);
      counter++;
      if(counter > 50) {
        break;
      }
		}
    if(counter < 50) {
  		// Close Hand
  		bodyMovement->moveHand(false, 600000, true); // (isLeftHand, value, blocking)

  		// Shake Hand
  		bodyMovement->setFolderToPlay("ShakingHand_5");
      bodyMovement->start();

      sentence = handShakeSentences[rand() % handShakeSentences.size()];
      textToSpeech->setText(sentence);
      face->init("speak");
      textToSpeech->start();
      usleep(sentence.length()*80000);
      face->backToNormalFast();

		  while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			  usleep(100000);
		  }
  		// Open and Drawback Hand
  		bodyMovement->moveHand(false, 0, false); // (isLeftHand, value, blocking)
  		bodyMovement->setFolderToPlay("DrawingBack_5");
      bodyMovement->start();
    } else {

  		bodyMovement->setFolderToPlay("DrawingBack_5");
      bodyMovement->start();
      usleep(2000000);
		  
      sentence = noHandSentences[rand() % noHandSentences.size()];
      textToSpeech->setText(sentence);
      face->init("speak");
      textToSpeech->start();
      usleep(sentence.length()*80000);
      face->backToNormalFast();

      face->init("smileblink");
    }

      
    sentence = goodbyeSentences[rand() % goodbyeSentences.size()];
    textToSpeech->setText(sentence);
    face->init("speak");
    textToSpeech->start();
    usleep(sentence.length()*80000);
    face->backToNormalFast();


		while(bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->isActive()) {
			usleep(100000);
		}

		currentStatus.str("RoboyGiveCloseShakeOpenDrawback Behaviour Completed"); 

		isBehaviourFinished = true;
    }

    void
    RoboyGiveCloseShakeOpenDrawbackBehaviour::terminate()
    {

		// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

      std::cout << "RoboyGiveCloseShakeOpenDrawbackBehaviour STOP " << std::endl;  
			bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
			// setting back the isFinished and isActive Values happens in the stop() function.
    
    }


    std::string
    RoboyGiveCloseShakeOpenDrawbackBehaviour::getInfo()
    {
      return "RoboyGiveCloseShakeOpenDrawbackBehaviour";      
    }


    void 
    RoboyGiveCloseShakeOpenDrawbackBehaviour::initSentences()
    {
      introSentences.push_back("What is your name? Yeah. Well. Blah blah blah blah.");
	    introSentences.push_back("Huh! What am I doing here?");
	    introSentences.push_back("Hello, stranger! Do you know that I have muscles, like you do? I am Roboy by the way.");
	    introSentences.push_back("Hi! My name is Roboy! Do you know who my father is? Is it my Professor?");
	    introSentences.push_back("Hello You! I am Roboy. I would like to have a brother or perhaps a sister. I am manufactured, so I might have one in the near future.");
	    introSentences.push_back("Hello You! I am Roboy! I can talk, but I do not understand what I am saying.");
	    introSentences.push_back("Hello! I am Roboy. I would like to run around. But I am not strong yet. Will I be able to run in the future?");
	    introSentences.push_back("Hi You! I do not recognize you. I find it very difficult to recognize human faces because the same face looks different every time. They call me Roboy, by the way.");
	    introSentences.push_back("Hi You! I am Roboy! I find my head is too big, and my legs too short – I hope this will change soon.");
	    introSentences.push_back("Hey You stranger! I am Roboy. I am a machine. And so are you! How are you?");
      introSentences.push_back("I can talk, but I don't understand what I am saying. They call me Roboy.");
      introSentences.push_back("Humans call me Roboy. They say that robots cannot be creative. But then most humans aren't very creative either");
      introSentences.push_back("I am Roboy. Because I only have batteries I don't know what it feels like to be thirsty. How is it like to drink a beer?");
      introSentences.push_back("I would like a beer. But I am a robot. They call me Roboy.");
      introSentences.push_back("Hello, for dinner I like to have a nice twenty four volt power supply. Please shake my hand.");
      introSentences.push_back("I, Roboy, can move almost like a human. I have 48 muscles. It is a pleasure.");
      introSentences.push_back("My name is Roboy. My programmers are not finished with me yet. Am I a robot already?");


	    handShakeSentences.push_back("Wiggle wiggle wiggle wiggle wiggle. yeah!");
	    handShakeSentences.push_back("Nice to meet you");
	    handShakeSentences.push_back("It is a pleasure.");
	    handShakeSentences.push_back("It is a real pleasure.");
	    handShakeSentences.push_back("Uuaauu. I like it.");
	    handShakeSentences.push_back("oo ooooo oooo o");
	    handShakeSentences.push_back("blah blah blah blah");


      noHandSentences.push_back("Oh. come on.");
      noHandSentences.push_back("Hello.");
      noHandSentences.push_back("I do not bite you know?");
      noHandSentences.push_back("Well then. Wait in line");
      noHandSentences.push_back("I am a sweet robot you know?");
      noHandSentences.push_back("Well shaking or not shaking. I cannot feel the difference.");
	    // giveHandSentences.push_back("Give me your hand!");

	    goodbyeSentences.push_back("Goodbye");
	    goodbyeSentences.push_back("I think that was enough? Bye bye");
	    goodbyeSentences.push_back("I think I need a rest. Bye bye");
	    goodbyeSentences.push_back("Tchaaau.");
	    goodbyeSentences.push_back("See you later. Aligator.");
      goodbyeSentences.push_back("See you in a while. Crocodile.");
      goodbyeSentences.push_back("Nehehxt.");



/*
	    facialExpressions.push_back("smile");
	    facialExpressions.push_back("smileblink");
	    facialExpressions.push_back("shy");
	    facialExpressions.push_back("surprise");
	    facialExpressions.push_back("kiss");
*/
}





}
