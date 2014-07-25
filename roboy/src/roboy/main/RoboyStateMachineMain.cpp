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

#include <string>
#include <vector>
#include <iostream>

#include "RoboyFileWriter.h"
#include "RoboyFileSystem.h"
#include "RoboyFileParser.h"
#include "RoboyPerson.h"
#include "RoboyVision.h"
#include "RoboyGUI.h"
#include "RoboyTimer.h"
#include "RoboyBodyActivityRepertoire.h"
#include "RoboyBehaviouralRepertoire.h"
#include "RoboyState.h"
#include "RoboyStarmindConnector.h"

// FIXME: there should be a header file
#define CAN_BUS_NODE (const char *)"/dev/pcan32"

using namespace Roboy;



/* Roboy file parser methods and datastructures */

//RoboyFileParser* parser;  // vector of Roboy's behaviours

void initFileParserInfo();


/* Roboy body activity methods and datastructures */


RoboySpeechProduction* speechProductionActivity;
RoboyFacialExpression* facialExpressionActivity;
RoboyTextToSpeech* textToSpeechActivity;
RoboyBodyMovement* bodyMovementActivity;
RoboyListenToSound* listenToSoundActivity;

std::vector<RoboyBodyActivity*> bodyActivities;  // vector of Roboy's behaviours

void initBodyActivityInfo();


/* Roboy vision methods and datastructures */

RoboyVision* vision;

bool faceRecognitionStarted;

std::vector< cv::Rect_<int> > faces;

void initVisionInfo();


/** Roboy behaviour methods and datastructures */

int behaviourIndex;

RoboyDoNothing* roboyDoNothing;

RoboyShowExpressionsBehaviour* roboyShowExpressions;
RoboySpeakingRandomBehaviour*	roboySpeakingRandom;
RoboyStarmind* roboyStarmind;
RoboyGazingBehaviour* roboyGazing;
RoboyGiveCloseShakeOpenDrawbackBehaviour* roboyGiveCloseShakeOpenDrawback;
RoboyIntroductionBehaviour* roboyIntroduction;
RoboyFollowingFaceBehaviour* roboyFollowingFace;
RoboyComeToMeBehaviour* roboyComeToMe;
RoboyShyBehaviour* roboyShy;
RoboyHugRolfBehaviour* roboyHugRolf;

RoboyBehaviour* cBehaviour = NULL;

std::vector<RoboyBehaviour*> behaviours;  // vector of Roboy's behaviours

void initBehaviourInfo();


/** Roboy configuration files and datastructures */

std::vector<RoboyState*> states;

void initStateInfo();


// Roboy gui methods and datastructures

RoboyGUI* gui;

void initGUIInfo();


/* Roboy configuration files and datastructures */

std::string configurationFile;

std::vector<RoboyPerson*> persons;

void initDatabaseInfo();


/* Roboy Starmind Integration */

RoboyStarmindConnector* starmind;
void initStarmindConnector();

/* Roboy CanBus */

bool useRobot;
Robot *canBus;
void initRobot();

bool faceExpressionFullScreen;


/* ROBOY STATE MACHINE (main) */

int main(int argc,char *argv[])
{
	if (argc != 2)
	{
		std::cerr << "[Error] configuration file required. " << std::endl;
		std::cerr << "Example: ./RoboyStateMachine ../../../../roboy/database/configuration/RoboyConfigurationFile.conf" << std::endl;
		return 0;
	}

	configurationFile = argv[1];

	std::cout << "Loading " << configurationFile << std::endl; 


	std::cout << " MAIN STATE MACHINE FOR ROBOY " << std::endl;
	int cstate = RoboyState::DEFAULT;

	int vstate = RoboyVision::DEFAULT;

  std::cout << "CALLING initFileParserInfo()" << std::endl;
	initFileParserInfo();
	std::cout << "CALLING initDatabaseInfo()" << std::endl;
	initDatabaseInfo();
	std::cout << "CALLING initRobot()" << std::endl;
	initRobot();
	std::cout << "CALLING initVisionInfo()" << std::endl;
	initVisionInfo();
	std::cout << "CALLING initStarmindConnector()" << std::endl;
	initStarmindConnector();
	std::cout << "CALLING initBodyActivityInfo()" << std::endl;
	initBodyActivityInfo();
	std::cout << "CALLING initBehaviourInfo()" << std::endl;
	initBehaviourInfo();
	std::cout << "CALLING initStateInfo()" << std::endl;
	initStateInfo();
	std::cout << "CALLING initGUIInfo()" << std::endl;
	initGUIInfo();

	//RoboyPerson* person = new RoboyPerson(1,"Hugo","Hugo.avi");
	//persons.push_back(person);


	cv::Mat frame, g;


	int recognitionIndex = -1;
	char keyPressed = 48;

	while(1)
	{

		//std::cout << "Capturing frame" << std::endl;
		vision->captureFrame();

		//cstate = keyPressed;
		//		frame = vision->getScaledFrame();
		//		faces = vision->detectFaces();
		//		vision->recognizeFaces();

		//		if (key ==


		switch(cstate)
		{

		// DEFAULT
		case RoboyState::DEFAULT:
			std::cout << "Default state" << std::endl;
			break;


			// VISION
		case RoboyState::VISION:

//			frame = vision->getScaledFrame();
			faces = vision->detectFaces();

			g = vision->getGrayFrame();
			cv::imshow("tmp", g);

			vision->recognizeFaces();

			std::cout << "Vision state: " << vstate << " , " << faces.size() << ",  " << vision->getAverageNumberofFacesDetected() << std::endl;


			switch(vstate)
			{

			case RoboyVision::DEFAULT:

				if(vision->getAverageNumberofFacesDetected() >= 1)
					vstate = RoboyVision::FACE_DETECTED;

				if(vision->getTimeSinceLastFaceDetected() >= 5)
					vstate = RoboyVision::NO_FACE_FOR_A_WHILE;


				faceRecognitionStarted = false;

				break;

			case RoboyVision::FACE_DETECTED:

				if(!vision->useFaceRecognition)
				{
					vstate = RoboyVision::FACE_NOT_RECOGNIZED;
					textToSpeechActivity->setText("Hello, stranger !! What can I do for you? ");

					break;
				}


				if(!faceRecognitionStarted)
				{
					vision->resetRecognitionRecord();
					faceRecognitionStarted = true;
				}

				vision->updateRecognitionRecord();

				recognitionIndex = vision->getRecognizedPerson();

				if(recognitionIndex != -1)
				{
					vstate = RoboyVision::FACE_RECOGNIZED;
					std::cout << " Person recognized: " << recognitionIndex << std::endl;
					std::cout << " Person recognized: " << persons[recognitionIndex]->getName() << std::endl;
					textToSpeechActivity->setText("Hello, " + persons[recognitionIndex]->getName() + "!! How are you? ");
				}

				break;


			case RoboyVision::NO_FACE_FOR_A_WHILE:
				vstate = RoboyVision::DEFAULT;

				break;


			case RoboyVision::FACE_RECOGNIZED:


//				std::cout << " name: " << persons[recognitionIndex]->getName() << std::endl;
//				textToSpeechActivity->init(persons[recognitionIndex]->getName());

//				textToSpeechActivity->init("");
				cBehaviour = behaviours[RoboyBehaviour::GIVE_CLOSE_SHAKE_OPEN_DRAWBACK_BEHAVIOUR];
				cBehaviour->start();

				if(cBehaviour->isFinished())
				{
					cBehaviour->stop();
					vstate = RoboyVision::DEFAULT;
					persons[recognitionIndex]->setNotRecognized();
				}

				break;

			case RoboyVision::FACE_NOT_RECOGNIZED:

				std::cout << " face not recognized " << std::endl;
				cBehaviour = behaviours[RoboyBehaviour::GIVE_CLOSE_SHAKE_OPEN_DRAWBACK_BEHAVIOUR];
				cBehaviour->start();

				if(cBehaviour->isFinished())
				{
					cBehaviour->stop();
					vstate = RoboyVision::DEFAULT;
				}

				break;
			}

			break;

			case RoboyState::RESERVE_STATE:
				std::cout << "RESERVE_STATE" << std::endl;
				break;


			case RoboyState::SHOW_EXPRESSIONS_STATE:
				std::cout << "SHOW_EXPRESSIONS_STATE" << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;

				break;

			case RoboyState::MOUTH_FOLLOWNIG_SOUND_STATE:
				std::cout << "MOUTH_FOLLOWNIG_SOUND_STATE" << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;

			case RoboyState::STARMIND:
				//std::cout << "Starmind state " << std::endl;

				if(starmind->isStarmindConnected()){

					if(cBehaviour==NULL){
						cBehaviour = states[cstate]->selectBehaviour();
						cBehaviour->start();
					} else if (!cBehaviour->isActive()){
						cBehaviour = states[cstate]->selectBehaviour();
						cBehaviour->start();
					}

					if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
					{
						cBehaviour->stop();
						cstate = RoboyState::DEFAULT;
					}

					//std::cout << "1" << std::endl;
					//cBehaviour = behaviours[RoboyBehaviour::STARMIND];
					//std::cout << "2" << std::endl;
					//cBehaviour->start();
					//std::cout << "3" << std::endl;
					//if(cBehaviour->isFinished())
					//{
					//	cBehaviour->stop();
					//}
				}else{
					std::cout << "Starmind disconnected " << std::endl;
				}
				break;


			case RoboyState::GAZING_STATE:
				std::cout << "Gazing state " << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;


			case RoboyState::GIVE_CLOSE_SHAKE_OPEN_DRAWBACK_STATE:
				std::cout << "GIVE_CLOSE_SHAKE_OPEN_DRAWBACK_STATE" << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;

				case RoboyState::INTRODUCTION_STATE:
				std::cout << "INTRODUCTION_STATE " << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;

				case RoboyState::FOLLOWING_FACE_STATE:
				std::cout << "FOLLOWING_FACE_STATE " << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;

				case RoboyState::COME_TO_ME_STATE:
				std::cout << "COME_TO_ME_STATE " << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to st./RoboyStateMachine ../../../../roboy/database/configuration/RoboyConfigurationFile.confop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;

				case RoboyState::SHY_STATE:
				std::cout << "SHY_STATE " << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;

				case RoboyState::HUG_ROLF_STATE:
				std::cout << "HUG_ROLF_STATE " << std::endl;

				// if there is no behaviour yet OR if the behaviour is not active, start this state's new behaviour
				if(cBehaviour==NULL){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				} else if (!cBehaviour->isActive()){
				cBehaviour = states[cstate]->selectBehaviour();
				cBehaviour->start();
				}

				if(cBehaviour->isFinished()) // if the behviour's execution has finished it will still be active. One has to stop the behaviour.
				{
					cBehaviour->stop();
					cstate = RoboyState::DEFAULT;
				}
				break;



				// DEFAULT
				//			case RoboyState::DEFAULT:

				//				cBehaviour = states[cstate]->selectBehaviour();

				//			if(vision->getAverageNumberofFacesDetected() >= 1)
				//				cstate = RoboyState::FACE_DETECTED;

				//			if(vision->getTimeSinceLastFaceDetected() >= 5)
				//				cstate = RoboyState::NO_FACE_FOR_A_WHILE;

				//				break;



				//		case RoboyState::FACE_DETECTED:

				//			cBehaviour = states[cstate]->selectBehaviour();
				//			cBehaviour->start();

				//			if(vision->getAverageNumberofFacesDetected() == 0)
				//			{
				//				cBehaviour->stop();
				//				cstate = RoboyState::FACE_LEFT;
				//			}
				//
				//			break;


				// FACE DETECTED
				//		case RoboyState::FACE_LEFT:
				//
				//			cBehaviour = behaviours[RoboyBehaviour::FAREWELL];
				//			cBehaviour->start();
				//
				//			if(cBehaviour->isFinished())
				//			{
				//				cBehaviour->stop();
				//				cstate = RoboyState::DEFAULT;
				//			}
				//
				//			break;
				//
				//		case RoboyState::NO_FACE_FOR_A_WHILE:
				//
				//			cBehaviour = states[cstate]->selectBehaviour();
				//			cBehaviour->start();
				//
				//			if(vision->getAverageNumberofFacesDetected() >= 1)
				//			{
				//				cBehaviour->stop();
				//				cstate = RoboyState::DEFAULT;
				//			}
				//
				//			break;

		}


		// display gui
		char key = gui->show(cBehaviour);

		char starmindKey = starmind->getNextMode();
		if(starmindKey != ' '){
			key = starmindKey;
		}

		if(key == RoboyGUI::KEY_ESC) // Exit
			break;

		// if a state change is forced by pressing a key, terminate the running behaviour first
		if(key != RoboyGUI::KEY_NONE) {

			if(cBehaviour != NULL) {
				if(!cBehaviour->isFinished()) {cBehaviour->stop();}
			}

		}

		//FIXME: GET THE (int) ASCII CODE instead
		if(key == RoboyGUI::KEY_0)
			cstate = RoboyState::DEFAULT;
		else if (key == RoboyGUI::KEY_1)
			cstate = RoboyState::VISION;
		else if (key == RoboyGUI::KEY_2)
			cstate = RoboyState::RESERVE_STATE;
		else if (key == RoboyGUI::KEY_3)
			cstate = RoboyState::SHOW_EXPRESSIONS_STATE;
		else if (key == RoboyGUI::KEY_4)
			cstate = RoboyState::MOUTH_FOLLOWNIG_SOUND_STATE;
		else if (key == RoboyGUI::KEY_5)
			cstate = RoboyState::STARMIND;
		else if (key == RoboyGUI::KEY_6)
			cstate = RoboyState::GAZING_STATE;
		else if (key == RoboyGUI::KEY_7)
			cstate = RoboyState::GIVE_CLOSE_SHAKE_OPEN_DRAWBACK_STATE;
		else if (key == RoboyGUI::KEY_8)
			cstate = RoboyState::INTRODUCTION_STATE;
		else if (key == RoboyGUI::KEY_9)
			cstate = RoboyState::FOLLOWING_FACE_STATE;
		else if (key == RoboyGUI::KEY_i)
			cstate = RoboyState::COME_TO_ME_STATE;
		else if (key == RoboyGUI::KEY_o)
			cstate = RoboyState::SHY_STATE;
		else if (key == RoboyGUI::KEY_p)
			cstate = RoboyState::HUG_ROLF_STATE;



		//if(key != RoboyGUI::KEY_NONE)
		//	keyPressed = key;

		//	std::cout << " KEY PRESSED: " << key << "   state: " << cstate << "   KEY NONE: " << RoboyGUI::KEY_NONE << std::endl;

	}
    for(unsigned int i=0; i<bodyActivities.size(); i++)
        delete bodyActivities[i];

	return 1;
}


void
initFileParserInfo()
{
	RoboyFileParser::parser = new RoboyFileParser(configurationFile);
}


void
initBodyActivityInfo()
{

	// !! Pushing back the BodyActivities has to happen in the same order as they are defined in RoboyBodyActivity.h !!
	// Because the body activity vector will be used by the behaviour classes in their init() and execute() with the numbering definition from RoboyBodyActivity.h

	speechProductionActivity = new RoboySpeechProduction();
	bodyActivities.push_back(speechProductionActivity);

	facialExpressionActivity = new RoboyFacialExpression();
	bodyActivities.push_back(facialExpressionActivity);

	textToSpeechActivity = new RoboyTextToSpeech();
	textToSpeechActivity->init("");
	bodyActivities.push_back(textToSpeechActivity);

	if(useRobot){
		bodyMovementActivity = new RoboyBodyMovement(canBus);
	}
	bodyActivities.push_back(bodyMovementActivity);

	listenToSoundActivity = new RoboyListenToSound();
	listenToSoundActivity->init("");
	bodyActivities.push_back(listenToSoundActivity);

}

void
initVisionInfo()
{
	vision = new RoboyVision();

  if (vision->useVision){
		std::cout << "USE_VISION set to YES -> training Face detector, training face recognizer, setting People List" << std::endl;
		vision->trainFaceDetector();
		vision->trainFaceRecognizer();
		vision->setPeopleList(persons);
	}

}


void initBehaviourInfo()
{

	// !! Pushing back the behaviours has to happen in the same order as they are defined in RoboyBehaviour.h !!
	//std::cout << "entering initBehaviourInfo" << std::endl;
	//if(useRobot) std::cout << "using Robot" << std::endl;
	roboyDoNothing = new RoboyDoNothing(bodyActivities);
	//std::cout << "constructed RoboyDoNothing" << std::endl;

	if(useRobot) roboyShowExpressions = new RoboyShowExpressionsBehaviour(bodyActivities, canBus);
	std::vector<std::string> sentences;
    sentences.push_back("What is the airspeed-velocity of an unladen swallow?");
    sentences.push_back("Hello.");
    sentences.push_back("Bye bye.");
	roboySpeakingRandom = new RoboySpeakingRandomBehaviour(bodyActivities, sentences);
	
	if(useRobot) roboyStarmind = new RoboyStarmind(bodyActivities, starmind, canBus);
	if(useRobot) roboyGazing = new RoboyGazingBehaviour(bodyActivities, canBus);
	if(useRobot) roboyGiveCloseShakeOpenDrawback = new RoboyGiveCloseShakeOpenDrawbackBehaviour(bodyActivities, canBus);

	if(useRobot) roboyIntroduction = new RoboyIntroductionBehaviour(bodyActivities, canBus);
	if(useRobot) roboyFollowingFace = new RoboyFollowingFaceBehaviour(bodyActivities, canBus, vision);
	if(useRobot) roboyComeToMe = new RoboyComeToMeBehaviour(bodyActivities, canBus);
	if(useRobot) roboyShy = new RoboyShyBehaviour(bodyActivities, canBus);
	if(useRobot) roboyHugRolf = new RoboyHugRolfBehaviour(bodyActivities, canBus);

	behaviours.push_back(roboyDoNothing);
if(useRobot) {
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyShowExpressions);
	behaviours.push_back(roboySpeakingRandom);
	behaviours.push_back(roboyStarmind);
	behaviours.push_back(roboyGazing);
	behaviours.push_back(roboyGiveCloseShakeOpenDrawback);
	behaviours.push_back(roboyIntroduction);
	behaviours.push_back(roboyFollowingFace);
	behaviours.push_back(roboyComeToMe);
	behaviours.push_back(roboyShy);
	behaviours.push_back(roboyHugRolf);
} else {
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboySpeakingRandom);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
	behaviours.push_back(roboyDoNothing);
}
 
}


void
initStateInfo()
{

	// !! Pushing back the states has to happen in the same order as they are defined in RoboyState.h !!

	// DEFAULT
	RoboyState* state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::DO_NOTHING]);
	states.push_back(state);

	// VISION
	state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::VISION_DO_NOTHING]);
	states.push_back(state);

	// RESERVE_STATE
	state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::RESERVE_DO_NOTHING]);
	states.push_back(state);

	// SHOW_EXPRESSIONS_STATE
	state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::SHOW_EXPRESSIONS_BEHAVIOUR]);
	states.push_back(state);

	// MOUTH_FOLLOWNIG_SOUND_STATE
	state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::MOUTH_FOLLOWNIG_SOUND_BEHAVIOUR]);
	states.push_back(state);

	// STARMIND
	state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::STARMIND]);
	states.push_back(state);

	// GAZING_STATE
	state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::GAZING_BEHAVIOUR]);
	states.push_back(state);

  // GIVE_CLOSE_SHAKE_OPEN_DRAWBACK_STATE
  state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::GIVE_CLOSE_SHAKE_OPEN_DRAWBACK_BEHAVIOUR]);
	states.push_back(state);

  // INTRODUCTION_STATE
  state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::INTRODUCTION_BEHAVIOUR]);
	states.push_back(state);

  // FOLLOWING_FACE_STATE
  state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::FOLLOWING_FACE_BEHAVIOUR]);
	states.push_back(state);

  // COME_TO_ME_STATE
  state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::COME_TO_ME_BEHVAIOUR]);
	states.push_back(state);

  // SHY_STATE
  state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::SHY_BEHAVIOUR]);
	states.push_back(state);

  // HUG_ROLF_STATE
  state = new RoboyState();
	state->addBehaviour(behaviours[RoboyBehaviour::HUG_ROLF_BEHAVIOUR]);
	states.push_back(state);

}


void
initGUIInfo()
{
	gui = new RoboyGUI("Roboy Face Recognition", vision);
	gui->setPersonList(persons);
}



void
initDatabaseInfo()
{
	RoboyFileSystem::removeDirectory("/tmp/roboy");
	RoboyFileSystem::createDirectory("/tmp/roboy");
	RoboyFileSystem::createDirectory("/tmp/roboy/people");

	std::string useRobotStr = RoboyFileParser::parser->get<std::string>("robot", "USE_ROBOT");
	useRobot = !useRobotStr.compare("YES");

	std::cout << " use robot: " << useRobot << std::endl;

//	std::string csvFile = RoboyFileParser::parser->get<std::string>("vision", "FACE_RECOGNITION_TRAINING_SET_FILE");
	std::string csvFile = "/tmp/roboy/RoboyAlbum.csv";
	RoboyFileWriter::createFile(csvFile);

	int faceWidth = RoboyFileParser::parser->get<int>("vision", "RECOGNITION_FACE_WIDTH");
	int faceHeight = RoboyFileParser::parser->get<int>("vision", "RECOGNITION_FACE_HEIGHT");
	std::string databasePath = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH");

	std::cout << " DATA BASE PATH: " << databasePath << std::endl;

	std::string peopleDirectory = databasePath + "/people";
	std::vector<std::string> personList = RoboyFileSystem::getDirectoryList(peopleDirectory);

	std::cout << " #persons: " << personList.size() << std::endl;
	int personCount = 0;

	//for(int i=0; i<personList.size(); i++)
	for(int i=personList.size()-1; i>=0; i--)
	{
		std::string personName = RoboyFileSystem::getDirectoryName(personList[i]) ;

		if(personName.compare("./svn") != 0)
		{
			std::string personTmp = "/tmp/roboy/people/"+personName;
			RoboyFileSystem::createDirectory(personTmp);

			std::string personDirectory = personList[i] + "/photos/";
			std::cerr << "TESTING: " << personDirectory << std::endl;
			std::vector<std::string> photos = RoboyFileSystem::getFileList(personDirectory);;

			RoboyPerson* rPerson = new RoboyPerson(personCount, personName, "");
			persons.push_back(rPerson);

			for(int j=0; j<photos.size(); j++)
			{

				std::string photoPath = "/tmp/roboy/people/"+personName+"/" + RoboyFileSystem::getFileName(photos[j]);
				std::cout << "PHOTO PATH "<< photoPath << std::endl;
				cv::Mat photoResized;
				cv::Mat photo = cv::imread(photos[j], CV_LOAD_IMAGE_GRAYSCALE);
				cv::resize(photo, photoResized, cv::Size(faceWidth, faceHeight), 1.0, 1.0, cv::INTER_CUBIC);
				cv::imwrite(photoPath, photoResized);

				std::stringstream idStr; idStr << personCount;
				std::string photoRow = photoPath + ";" + idStr.str();


				Roboy::RoboyFileWriter::writeLine(csvFile, photoRow);
				std::cout << "FILE: " << csvFile << ", " << photoRow << std::endl;
			}

			personCount++;
		}
	}

	std::cout << " FINISH CONF FILE: " << databasePath << std::endl;

}


void
initStarmindConnector()
{
	starmind = new RoboyStarmindConnector(vision);
	starmind->start();
}

void initRobot(){


  if(useRobot == false) return;

	canBus = new Robot("../../../../roboy/", false);
	canBus->initialize(CAN_BUS_NODE);
	canBus->enterPreOperational();
	for(int i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		canBus->clearFault(i);
		canBus->initializePDORXMapping(i);
		canBus->initializePDOTXMapping(i);
		canBus->resetInterpolatedBuffer(i);
		canBus->initializeMotorForControl(i);
	}	
	canBus->initializeDigitalInput(RIGHT_HAND_SENSOR_ID, DIGITAL_INPUT_1, DIGITAL_INPUT_GENERAL_PURPOSE_A, ACTIVE_HIGH);
	canBus->initializeDigitalInput(RIGHT_HAND_SENSOR_ID, DIGITAL_INPUT_2, DIGITAL_INPUT_GENERAL_PURPOSE_B, ACTIVE_HIGH);
	canBus->initializeDigitalInput(RIGHT_HAND_SENSOR_ID, DIGITAL_INPUT_3, DIGITAL_INPUT_GENERAL_PURPOSE_C, ACTIVE_HIGH);
	canBus->initializeDigitalInput(RIGHT_HAND_SENSOR_ID, DIGITAL_INPUT_4, DIGITAL_INPUT_GENERAL_PURPOSE_D, ACTIVE_HIGH);

	canBus->initializeDigitalInput(LEFT_HAND_SENSOR_ID, DIGITAL_INPUT_1, DIGITAL_INPUT_GENERAL_PURPOSE_A, ACTIVE_HIGH);
	canBus->initializeDigitalInput(LEFT_HAND_SENSOR_ID, DIGITAL_INPUT_2, DIGITAL_INPUT_GENERAL_PURPOSE_B, ACTIVE_HIGH);
	canBus->initializeDigitalInput(LEFT_HAND_SENSOR_ID, DIGITAL_INPUT_3, DIGITAL_INPUT_GENERAL_PURPOSE_C, ACTIVE_HIGH);
	canBus->initializeDigitalInput(LEFT_HAND_SENSOR_ID, DIGITAL_INPUT_4, DIGITAL_INPUT_GENERAL_PURPOSE_D, ACTIVE_HIGH);
  canBus->startNode();

}

