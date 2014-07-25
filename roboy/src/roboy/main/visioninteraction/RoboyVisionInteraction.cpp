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
#include <cstdlib>

#include "RoboyFileWriter.h"
#include "RoboyFileSystem.h"
#include "RoboyFileParser.h"
#include "RoboyPerson.h"
#include "RoboyVision.h"
#include "RoboyVisionInteractionGUI.h"
#include "RoboyTimer.h"
#include "RoboyBodyActivityRepertoire.h"
#include "RoboyBehaviouralRepertoire.h"
#include "time.h"

// FIXME: there should be a header file
#define CAN_BUS_NODE (const char *)"/dev/pcan32"

using namespace Roboy;

enum state {DEFAULT, FACE_DETECTED, NO_FACE_FOR_A_WHILE, FACE_RECOGNIZED, FACE_NOT_RECOGNIZED, GIVE_HAND, CLOSE_AND_SHAKE_HAND, OPEN_AND_DRAWBACK_HAND, PREPARE_SLEEP, SLEEP, SAY_COOL_STUFF, SAY_SHORT_STUFF};

std::vector<std::string> introSentences;
std::vector<std::string> handShakeSentences;
std::vector<std::string> giveHandSentences;
std::vector<std::string> goodbyeSentences;
std::vector<std::string> facialExpressions;
std::vector<std::string> coolStuffSentences;
std::vector<std::string> shortStuffSentences;


void initSentences();

/* Roboy file parser methods and datastructures */

//RoboyFileParser* parser;  // vector of Roboy's behaviours

void initFileParserInfo();


/* Roboy body activity methods and datastructures */


RoboySpeechProduction* speechProductionActivity;
RoboyFacialExpression* facialExpressionActivity;
RoboyTextToSpeech* textToSpeechActivity;
RoboyBodyMovement* bodyMovementActivity;

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
RoboyGreetStranger* roboyGreetStranger;
RoboyFarewell* roboyFarwell;
RoboyCry* roboyCry;
RoboyGreetFriend* roboyGreetFriend;
RoboyGreetAndShakeHandBehaviour* handshakingBehaviour;

RoboyFacialSpeakingBehaviour* speakingBehaviour;
RoboyFacialExpressionBehaviour* facialBehaviour;

RoboyBehaviour* cBehaviour = NULL;
RoboyBehaviour* cBehaviour2 = NULL;

void initBehaviourInfo();


// Roboy gui methods and datastructures

RoboyVisionInteractionGUI* gui;

void initGUIInfo();


/* Roboy configuration files and datastructures */

std::string configurationFile;

std::vector<RoboyPerson*> persons;

void initDatabaseInfo();


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
		std::cerr << "Example: ./RoboyStateMachine ../../../../../roboy/database/configuration/RoboyConfigurationFile.conf" << std::endl;
		return 0;
	}

	configurationFile = argv[1];

	std::cout << "Loading " << configurationFile << std::endl; 


	std::cout << " MAIN STATE MACHINE FOR ROBOY " << std::endl;

	int vstate = DEFAULT;

	std::cout << "CALLING initSentences()" << std::endl;
	initSentences();
	std::cout << "CALLING initFileParserInfo()" << std::endl;
	initFileParserInfo();
	std::cout << "CALLING initDatabaseInfo()" << std::endl;
	initDatabaseInfo();
	std::cout << "CALLING initRobot()" << std::endl;
	initRobot();
	std::cout << "CALLING initVisionInfo()" << std::endl;
	initVisionInfo();
	std::cout << "CALLING initBodyActivityInfo()" << std::endl;
	initBodyActivityInfo();
	std::cout << "CALLING initBehaviourInfo()" << std::endl;
	initBehaviourInfo();
	std::cout << "CALLING initGUIInfo()" << std::endl;
	initGUIInfo();

	//RoboyPerson* person = new RoboyPerson(1,"Hugo","Hugo.avi");
	//persons.push_back(person);

	int recognitionIndex = -1;
	char keyPressed = 48;

  std::srand(time(NULL));
  bool startbehaviour = false;
  int sentenceIndex = 0;

  int coolCounter = 1;
  int shortCounter = 1;

	while(1)
	{

		vision->captureFrame();


		//			frame = vision->getScaledFrame();
		faces = vision->detectFaces();
		vision->recognizeFaces();

		std::cout << "Vision state: " << vstate << " , " << faces.size() << ",  " << vision->getAverageNumberofFacesDetected() << std::endl;

		switch(vstate)
		{

		case DEFAULT:
    std::cout << " default state" << std::endl;
			if(vision->getAverageNumberofFacesDetected() >= 1)
				vstate = FACE_DETECTED;

//			if(vision->getTimeSinceLastFaceDetected() >= 5)
//				vstate = NO_FACE_FOR_A_WHILE;


			faceRecognitionStarted = false;

			break;

		case FACE_DETECTED:

			if(!vision->useFaceRecognition)
			{
				vstate = FACE_NOT_RECOGNIZED;

				int sentenceIndex = std::rand() % introSentences.size();

				speakingBehaviour->setText(introSentences[sentenceIndex]);

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
				vstate = FACE_RECOGNIZED;
				std::cout << " Person recognized: " << recognitionIndex << std::endl;
				std::cout << " Person recognized: " << persons[recognitionIndex]->getName() << std::endl;
				speakingBehaviour->setText("Hello, " + persons[recognitionIndex]->getName() + "!! How are you? ");
			}

			break;


		case NO_FACE_FOR_A_WHILE:
			vstate = DEFAULT;

			break;


		case FACE_RECOGNIZED:


			//				std::cout << " name: " << persons[recognitionIndex]->getName() << std::endl;
			//				textToSpeechActivity->init(persons[recognitionIndex]->getName());

      // only set Behaviour and start it when state is newly entered
      startbehaviour = false;
      if(cBehaviour == NULL){
        startbehaviour = true;
      } else if(!cBehaviour->isActive()) {
        startbehaviour = true;
      }

      if (startbehaviour) {
        cBehaviour = roboyGreetFriend;
			  cBehaviour->start();
      }

			if(cBehaviour->isFinished())
			{
				cBehaviour->stop();
				vstate = DEFAULT;
				persons[recognitionIndex]->setNotRecognized();
			}

			break;

		case FACE_NOT_RECOGNIZED:

			std::cout << " face not recognized " << std::endl;

      // only set Behaviour and start it when state is newly entered
      startbehaviour = false;
      if(cBehaviour == NULL){
        startbehaviour = true;
      } else if(!cBehaviour->isActive()) {
        startbehaviour = true;
      }

      if (startbehaviour) {
        cBehaviour = speakingBehaviour;
			  cBehaviour->start();
      }


      if(cBehaviour->isFinished()) {
				cBehaviour->stop();
				vstate = GIVE_HAND;
      }

			break;

		case GIVE_HAND:

			std::cout << " give hand " << std::endl;

      // only set Behaviour and start it when state is newly entered
      // only set Behaviours and start them when state is newly entered
      startbehaviour = false;
      if(cBehaviour == NULL){
        startbehaviour = true;
      } else if(!cBehaviour->isActive()) {
        startbehaviour = true;
      }
      if (startbehaviour) {
      startbehaviour = false;
      if(cBehaviour2 == NULL){
        startbehaviour = true;
      } else if(!cBehaviour2->isActive()) {
        startbehaviour = true;
      }
      }

      if (startbehaviour) {
        handshakingBehaviour->setPhase(RoboyGreetAndShakeHandBehaviour::GIVE_HAND_PHASE);
        cBehaviour2 = handshakingBehaviour;
			  cBehaviour2->start();

        usleep(2000000);

        sentenceIndex = std::rand() % giveHandSentences.size();
			  speakingBehaviour->setText(giveHandSentences[sentenceIndex]);
			  cBehaviour = speakingBehaviour;
			  cBehaviour->start();
      }

      // don't leave state until hand gets touched
			if(((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->canBus->readDigitalInput(25, 0) == true && cBehaviour->isFinished() && cBehaviour2->isFinished())
			{
				cBehaviour->stop();
				cBehaviour2->stop();
				vstate = CLOSE_AND_SHAKE_HAND;
			}

			break;

//			if(!facialBehaviour->isActive())
//			{
//				int index = rand() % facialExpressions.size();
//				facialBehaviour->setFacialExpression("kiss"); //facialExpressions[index]
//			}

//			facialBehaviour->start();

//			if(facialBehaviour->isFinished())
//			{
//				facialBehaviour->stop();
//			vstate = HAND_SHAKE;
//			}

			break;



		case  CLOSE_AND_SHAKE_HAND:
			std::cout << " hand shake" << speakingBehaviour->isActive() << std::endl;

      // only set Behaviours and start them when state is newly entered
      startbehaviour = false;
      if(cBehaviour == NULL){
        startbehaviour = true;
      } else if(!cBehaviour->isActive()) {
        startbehaviour = true;
      }
      if (startbehaviour) {
      startbehaviour = false;
      if(cBehaviour2 == NULL){
        startbehaviour = true;
      } else if(!cBehaviour2->isActive()) {
        startbehaviour = true;
      }
      }

      if (startbehaviour) {
        handshakingBehaviour->setPhase(RoboyGreetAndShakeHandBehaviour::CLOSE_AND_SHAKE_HAND_PHASE);
        cBehaviour2 = handshakingBehaviour;
			  cBehaviour2->start();

			  sentenceIndex = std::rand() % handShakeSentences.size();
			  speakingBehaviour->setText(handShakeSentences[sentenceIndex]);
			  cBehaviour = speakingBehaviour;
			  cBehaviour->start();
      }

      // don't leave state until hand got released
			if(((RoboyBodyMovement*) bodyActivities[RoboyBodyActivity::BODY_MOVEMENT] )->canBus->readDigitalInput(25, 0) == false && cBehaviour->isFinished() && cBehaviour2->isFinished())
			{
				cBehaviour->stop();
				cBehaviour2->stop();
				vstate = OPEN_AND_DRAWBACK_HAND;
			}

			break;


		case  OPEN_AND_DRAWBACK_HAND:
			std::cout << " open and draw back" << speakingBehaviour->isActive() << std::endl;

      // only set Behaviour and start it when state is newly entered
      startbehaviour = false;
      if(cBehaviour2 == NULL){
        startbehaviour = true;
      } else if(!cBehaviour2->isActive()) {
        startbehaviour = true;
      }

      if (startbehaviour) {
        handshakingBehaviour->setPhase(RoboyGreetAndShakeHandBehaviour::OPEN_AND_DRAWBACK_HAND_PHASE);
        cBehaviour2 = handshakingBehaviour;
			  cBehaviour2->start();
      }

			if(cBehaviour2->isFinished())
			{
				cBehaviour2->stop();
				vstate = PREPARE_SLEEP;
			}

			break;


		case PREPARE_SLEEP:
			std::cout << " prepare sleep " << std::endl;

			startbehaviour = false;
      if(cBehaviour == NULL){
        startbehaviour = true;
      } else if(!cBehaviour->isActive()) {
        startbehaviour = true;
      }

      if (startbehaviour) {
				int sentenceIndex = std::rand() % goodbyeSentences.size();
				speakingBehaviour->setText(goodbyeSentences[sentenceIndex]);
        cBehaviour = speakingBehaviour;
			  cBehaviour->start();
      }

			if(cBehaviour->isFinished())
			{
				cBehaviour->stop();
				vstate = SLEEP;
			}

			break;



		case SLEEP:
			std::cout << " sleep " << std::endl;
      startbehaviour = false;
      if(cBehaviour == NULL){
        startbehaviour = true;
      } else if(!cBehaviour->isActive()) {
        startbehaviour = true;
      }

      if (startbehaviour) {
        std::cout << "starting the Sleep Face" << std::endl;
        facialBehaviour->setFacialExpression("sleep");
        cBehaviour = facialBehaviour;
			  cBehaviour->start();
      }

			break;


		case SAY_COOL_STUFF:
			std::cout << " saying cool stuff " << std::endl;

			startbehaviour = false;
      if(cBehaviour == NULL){
        startbehaviour = true;
      } else if(!cBehaviour->isActive()) {
        startbehaviour = true;
      }

      if (startbehaviour) {
				int sentenceIndex = coolCounter % coolStuffSentences.size();
				speakingBehaviour->setText(coolStuffSentences[sentenceIndex]);
        cBehaviour = speakingBehaviour;
			  cBehaviour->start();
        coolCounter++;
      }

			if(cBehaviour->isFinished())
			{
				cBehaviour->stop();
				vstate = SLEEP;
			}

			break;


		case SAY_SHORT_STUFF:
			std::cout << " saying short stuff " << std::endl;

			startbehaviour = false;
      if(cBehaviour == NULL){
        startbehaviour = true;
      } else if(!cBehaviour->isActive()) {
        startbehaviour = true;
      }

      if (startbehaviour) {
				int sentenceIndex = shortCounter % shortStuffSentences.size();
				speakingBehaviour->setText(shortStuffSentences[sentenceIndex]);
        cBehaviour = speakingBehaviour;
			  cBehaviour->start();
        shortCounter++;
      }

			if(cBehaviour->isFinished())
			{
				cBehaviour->stop();
				vstate = SLEEP;
			}

			break;





//			cBehaviour = speakingBehaviour;
//			cBehaviour->start();
//
//			if(cBehaviour->isFinished())
//			{
//				cBehaviour->stop();
//				vstate = RoboyVision::DEFAULT;
//			}


		}



		// display gui
		char key = gui->show(cBehaviour);

		if(key == RoboyVisionInteractionGUI::KEY_ESC) // Exit
			break;


		// if a state change is forced by pressing a key, terminate the running behaviour first
		if(key != RoboyVisionInteractionGUI::KEY_NONE) {

			if(cBehaviour != NULL) {
        std::cout << "Pressing any Key -> stopping the behaviour" << std::endl;
				if(!cBehaviour->isFinished()) {cBehaviour->stop();}
			}

		}

		if(key == RoboyVisionInteractionGUI::KEY_0)
		{
			if(vstate == SLEEP)
			{
				vstate = DEFAULT;
			}
		}
    if(key == RoboyVisionInteractionGUI::KEY_1)
		{
			if(vstate == DEFAULT || SLEEP)
			{
				vstate = SAY_COOL_STUFF;
			}
		}
    if(key == RoboyVisionInteractionGUI::KEY_2)
		{
			if(vstate == DEFAULT || SLEEP)
			{
				vstate = SAY_SHORT_STUFF;
			}
		}
	}
  printf("Before calling destructors.\n");

	for(unsigned int i=0; i<bodyActivities.size(); i++) {
    printf("Before calling destructor of %d.\n", i);
		delete bodyActivities[i];
  }

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

	bodyMovementActivity = new RoboyBodyMovement(canBus);
	bodyActivities.push_back(bodyMovementActivity);

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

	speakingBehaviour = new RoboyFacialSpeakingBehaviour(bodyActivities);
	facialBehaviour = new RoboyFacialExpressionBehaviour(bodyActivities);

	roboyDoNothing = new RoboyDoNothing(bodyActivities);
	roboyGreetStranger = new RoboyGreetStranger(bodyActivities);
	roboyFarwell = new RoboyFarewell(bodyActivities);
	roboyCry = new RoboyCry(bodyActivities);
	roboyGreetFriend = new RoboyGreetFriend(bodyActivities);
  handshakingBehaviour = new RoboyGreetAndShakeHandBehaviour(bodyActivities, canBus);
  

//	std::vector<std::string> sentences;
//	sentences.push_back("What is the airspeed-velocity of an unladen swallow?");
//	sentences.push_back("Hello.");
//	sentences.push_back("Bye bye.");
//	roboySpeakingRandomBehaviour = new RoboySpeakingRandomBehaviour(bodyActivities, sentences);

}



void
initGUIInfo()
{
	gui = new RoboyVisionInteractionGUI("Roboy Face Recognition", vision);
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


void initRobot(){

	canBus = new Robot("../../../../../roboy/", false);
	canBus->initialize(CAN_BUS_NODE);
	canBus->enterPreOperational();
	for(int i = 1; i < TOTAL_MOTORS_IN_ROBOT; i++) {
		canBus->clearFault(i);
		canBus->initializePDORXMapping(i);
		canBus->initializePDOTXMapping(i);
		canBus->resetInterpolatedBuffer(i);
		canBus->initializeMotorForControl(i);
	}	
  // FIXME: Set proper defines
	canBus->initializeDigitalInput(25, 0, DIGITAL_INPUT_GENERAL_PURPOSE_A, ACTIVE_HIGH);
	canBus->initializeDigitalInput(25, 1, DIGITAL_INPUT_GENERAL_PURPOSE_B, ACTIVE_HIGH);
	canBus->initializeDigitalInput(25, 2, DIGITAL_INPUT_GENERAL_PURPOSE_C, ACTIVE_HIGH);
	canBus->initializeDigitalInput(25, 3, DIGITAL_INPUT_GENERAL_PURPOSE_D, ACTIVE_HIGH);
  canBus->startNode();


}


void initSentences()
{

	introSentences.push_back("Hello, stranger! Do you know that I have muscles, like you do? I am Roboy.");
	introSentences.push_back("Hi! I am Roboy! Do you know who my father is? Is it my Professor?");
	introSentences.push_back("Hello You! I am Roboy. I would like to have a brother or perhaps a sister. I am manufactured, so I might have one in the near future.");
	introSentences.push_back("Hello You! I am Roboy! I can talk, but I do not understand what I am saying.");
	introSentences.push_back("Hello! I am Roboy. I would like to run around. But I am not strong yet. Will I be able to run in the future?");
	introSentences.push_back("Hi You! I do not recognize you. I find it very difficult to recognize human faces because the same face looks different every time. I am Roboy.");
	introSentences.push_back("Hi You! I am Roboy! I find my head is too big, and my legs too short – I hope this will change soon.");
	introSentences.push_back("Hey You stranger! I am Roboy. I am a machine. And so are you! How are you?");


	handShakeSentences.push_back("Nice to meet you");
	handShakeSentences.push_back("It is a pleasure.");

	giveHandSentences.push_back("Give me your hand!");

	goodbyeSentences.push_back("It was a real pleasure. Goodbye");
	goodbyeSentences.push_back("I think I need a rest. Bye bye");

  coolStuffSentences.push_back("I can talk, but I don't understand what I am saying");
  coolStuffSentences.push_back("One says robots cannot be creative. But then most people aren't very creative either");
  coolStuffSentences.push_back("Because I only have batteries I don't know what it feels like to be thirsty. How is it to drink a beer? For me it's wonderful to sit at the charging station.");
  coolStuffSentences.push_back("I can move almost like a human. I have 48 muscles.");
  coolStuffSentences.push_back("My programmers are not finished with me yet. Am I a robot already?");
  coolStuffSentences.push_back("Can I have a friend? I like the big printer in this room. He is my friend now.");

  shortStuffSentences.push_back("bla bla bla");
  shortStuffSentences.push_back("You there!");


	facialExpressions.push_back("smile");
	facialExpressions.push_back("smileblink");
	facialExpressions.push_back("shy");
	facialExpressions.push_back("surprise");
	facialExpressions.push_back("kiss");

}


