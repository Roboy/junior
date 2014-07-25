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
#include "RoboyTimer.h"
#include "RoboyBodyActivityRepertoire.h"
#include "RoboyBehaviouralRepertoire.h"

#define FILE_PATH_TO_SPEECH_TIME_INTERVALS "/Theatre/SpeechTimeIntervals.txt"

// FIXME: there should be a header file
#define CAN_BUS_NODE (const char *)"/dev/pcan32"

using namespace Roboy;

std::vector<std::string> theatreScript;
std::vector<std::string> facialExpressions;

std::vector<int> waitTimerUS;

bool useTimerFile;

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

/* Face Expression */

void initFaceExpressions();

/** Roboy behaviour methods and datastructures */

int behaviourIndex;

void initBehaviourInfo();


/* Roboy configuration files and datastructures */
std::string configurationFile;

void initDatabaseInfo();
std::string databasePath;

/* Roboy CanBus */

bool useRobot;
Robot *canBus;
void initRobot();

bool faceExpressionFullScreen;


/* ROBOY STATE MACHINE (main) */

void playCue(int cue);
void startSpeech(int cue);
void startFacial(int cue);
void coordinateFacialAndSpeech(int cue);
void playHeadMovement(int cue);
void playLegMovement(int cue);
void playBodyMovement(int cue);
void endBodyMovement(int cue);
void waitOnBodyMovement();
void setRobotToStop();
void randomBehaviorLoop();
void loadWaitTimes();

bool randomBehavior;
bool randomActivityEnded;
boost::thread* randomBehaviorThread;

int main(int argc,char *argv[])
{
	char buffer[100];
	int cue, currentCue;

	
	if (argc != 2)
	{
		std::cerr << "[Error] configuration file required. " << std::endl;
		std::cerr << "Example: ./RoboyStateMachine ../../../../../roboy/database/configuration/RoboyConfigurationFile.conf" << std::endl;
		return 0;
	}
  randomBehavior = false;
  randomActivityEnded = true;
  useTimerFile = true;
	configurationFile = argv[1];

	std::cout << "Loading " << configurationFile << std::endl; 


	std::cout << " MAIN STATE MACHINE FOR ROBOY " << std::endl;

	std::cout << "CALLING initSentences()" << std::endl;
	initSentences();
	std::cout << "CALLING initFaceExpressions()" << std::endl;
	initFaceExpressions();
	std::cout << "CALLING initFileParserInfo()" << std::endl;
	initFileParserInfo();
	std::cout << "CALLING initDatabaseInfo()" << std::endl;
	initDatabaseInfo();
	std::cout << "CALLING initRobot()" << std::endl;
	initRobot();
	std::cout << "CALLING initBodyActivityInfo()" << std::endl;
	initBodyActivityInfo();
	std::cout << "CALLING initBehaviourInfo()" << std::endl;
	initBehaviourInfo();

	//RoboyPerson* person = new RoboyPerson(1,"Hugo","Hugo.avi");
	//persons.push_back(person);

  // Start random behavoir thread
  randomBehaviorThread = new boost::thread( boost::bind(&randomBehaviorLoop) );
  currentCue = 0;

  loadWaitTimes();

  if(waitTimerUS.size() != theatreScript.size()) {
    std::cout << "Wrong size for wait timer file" << std::endl;
  }
  std::cout << "Playing Face: sleep" << std::endl;
  facialExpressionActivity->init("sleep");

	while(1) {
		printf("Enter Cue number 1-%d ('e' to exit): ", (int) theatreScript.size());
		gets(buffer);
		if(strncmp(buffer, "e", 1) == 0) {
			break;
		}
		cue = atoi(buffer);
    if(cue == 0) {
      cue = currentCue + 1;
    }
    if(cue == -1) {
      loadWaitTimes();
      continue;
    }
		if(cue <= theatreScript.size()) {
      randomBehavior = false;
      while(!randomActivityEnded) {
        usleep(50000);
      }
      std::cout << "Stopped Random Activity." << std::endl;
			playCue(cue);
      currentCue = cue;
		}
	}
  setRobotToStop();

	for(unsigned int i=0; i<bodyActivities.size(); i++)
		delete bodyActivities[i];

	return 1;
}

void
loadWaitTimes() {
  int i;
  std::stringstream filePath;
  dataArrayWrapper tmp;
  filePath.str("");
  filePath << databasePath << FILE_PATH_TO_SPEECH_TIME_INTERVALS;
  tmp = loadDataFileArray(filePath.str().c_str(), 1);
  if(tmp.row != theatreScript.size()) return;
  waitTimerUS.clear();
  for(i = 0; i < tmp.row; i++) {
    waitTimerUS.push_back((int)tmp.dataArray[i]);
    //std::cout << "loaded [" << waitTimerUS[i] << "]" << std::endl;
  }
}

void
setRobotToStop() {
  if(!useRobot) return;
  bodyMovementActivity->setLegSwingBehavior(0, 4000, 4000, 0, 120, 2);
  if(randomBehavior) {
    randomBehavior = false;
    while(!randomActivityEnded) {
      usleep(50000);
    }
  }
  std::cout << "Stopped Random Activity." << std::endl;
  while(bodyMovementActivity->isLegSwinging) {
    usleep(50000);
  }
  std::cout << "Stopped LegbodyMovementActivity-> Swinging." << std::endl;
}

void
randomBehaviorLoop() {
	int counter;
  neckMotionCommand nMC;

  if(!useRobot) return;
  
  nMC.neckRollAngle = 0;
  nMC.neckPitchAngle = 0;
  nMC.neckYawAngle = 0;
  nMC.rollVelocity = 3000;
  nMC.pitchVelocity = 3000; 
  nMC.yawVelocity = 3000;
  nMC.rollAcceleration = 10000;
  nMC.pitchAcceleration = 10000; 
  nMC.yawAcceleration = 10000;	
  
	counter = 0;
  while(true) {
    while(!randomBehavior) {
      usleep(100000);
    }
    randomActivityEnded = false;
    randomActivityEnded = true;
    usleep(1000000);
  }
}

void
playCue(int cue) {
  switch(cue) {
		case 2:
		case 15:
		case 25:
		case 29:
		case 30:
		case 31:
		case 32:
      playBodyMovement(cue);
      playLegMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      playHeadMovement(cue);
      coordinateFacialAndSpeech(cue); 
      waitOnBodyMovement();
      break;

		case 11:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      waitOnBodyMovement();
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
    break;

		case 13:
		case 20:
      playBodyMovement(cue);
      playLegMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      playHeadMovement(cue);
      waitOnBodyMovement();
      break;


		case 18:
		case 55:
		case 56:
		case 58:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      facialExpressionActivity->init("smileblink");
      waitOnBodyMovement();
      break;

    case 33:
      randomBehavior = true;
      break;

		case 36:
      playBodyMovement(cue);
      playLegMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      startFacial(72); // Add smiling and blink
      playHeadMovement(cue);
      waitOnBodyMovement();
      break;


		case 35:
    case 37:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      startFacial(72); // Add smiling and blink
      waitOnBodyMovement();
      break;

    case 40:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      startFacial(75); // Add surprised
      waitOnBodyMovement();
      break;

    case 43:
      playBodyMovement(cue);
      playLegMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      playHeadMovement(cue);
      waitOnBodyMovement();
      break;

    case 44:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      startFacial(73); // Add smiling
      waitOnBodyMovement();
      break;

    case 46:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(72); // Add smiling and blink
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      waitOnBodyMovement();
      break;


    case 47:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      waitOnBodyMovement();
      break;

    case 49:
      facialExpressionActivity->backToNormalFast();
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      waitOnBodyMovement();
      break;

    case 52:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      waitOnBodyMovement();
      break;

    case 59:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      startFacial(73); // Add smile
      waitOnBodyMovement();
      break;


    case 61:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      waitOnBodyMovement();
      break;

    case 64:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      startFacial(72); // Add smile blink
      waitOnBodyMovement();
      break;

    case 66:
      playBodyMovement(cue);
      playLegMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      playHeadMovement(cue);
      waitOnBodyMovement();
      break;

    case 67:
      playBodyMovement(cue);
      playLegMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      playHeadMovement(cue);
      startFacial(74); // Add shy
      waitOnBodyMovement();
      break;

    default:
      playBodyMovement(cue);
      playLegMovement(cue);
      playHeadMovement(cue);
      startFacial(cue);
      startSpeech(cue); 
      coordinateFacialAndSpeech(cue); 
      waitOnBodyMovement();
      break;

  }
}



void playLegMovement(int cue) {
	// Function to play the leg movements
  if(!useRobot) return;
  
  // Handling Leg Motion
	switch(cue) {
		case 1:
		case 25:
      bodyMovementActivity->setLegSwingBehavior(7000000, 4000, 4000, 100000, 120, 1);
			break;
		default:
      bodyMovementActivity->setLegSwingBehavior(0, 4000, 4000, 0, 120, 2);
			break;
	}
}

void playHeadMovement(int cue) {
	// Function to play the movements
  neckMotionCommand nMC;
  if(!useRobot) return;
  
  nMC.neckRollAngle = 0;
  nMC.neckPitchAngle = 0;
  nMC.neckYawAngle = 0;
  nMC.rollVelocity = 3000;
  nMC.pitchVelocity = 3000; 
  nMC.yawVelocity = 3000;
  nMC.rollAcceleration = 10000;
  nMC.pitchAcceleration = 10000; 
  nMC.yawAcceleration = 10000;

  // Handling head Motion
	switch(cue) {
		case 1:
      nMC.neckYawAngle = -12;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = 12;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = 0;
      bodyMovementActivity->moveHead(nMC, false);  
			break;
		case 3:
		case 4:
      nMC.neckYawAngle = 12;
      bodyMovementActivity->moveHead(nMC, true);  
			break;

		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
      nMC.neckYawAngle = 12;
      nMC.yawVelocity = 5000;
      bodyMovementActivity->moveHead(nMC, true);  
			break;

		case 13:
      nMC.neckPitchAngle = 12;
      nMC.neckYawAngle = 12;
      nMC.pitchVelocity = 10000; 
      nMC.yawVelocity = 10000; 
      nMC.pitchAcceleration = 30000; 
      nMC.yawAcceleration = 30000; 
      usleep(1500000);
      bodyMovementActivity->moveHead(nMC, true);  
      usleep(200000);
      nMC.neckYawAngle = -12;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckPitchAngle = 0;
      nMC.neckYawAngle = 0;
      bodyMovementActivity->moveHead(nMC, true);  
			break;

		case 15:
      nMC.neckYawAngle = 8;
      nMC.yawVelocity = 6000;
      nMC.yawAcceleration = 20000;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = -8;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = 8;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = 0;
      bodyMovementActivity->moveHead(nMC, true);  
			break;

		case 17:
      nMC.neckYawAngle = -12;
      nMC.yawVelocity = 6000;
      nMC.yawAcceleration = 20000;
      bodyMovementActivity->moveHead(nMC, true);  
			break;

    // More head movements
		case 20:
      nMC.neckPitchAngle = 12;
      nMC.neckYawAngle = 12;
      nMC.pitchVelocity = 12000; 
      nMC.yawVelocity = 12000; 
      nMC.pitchAcceleration = 40000; 
      nMC.yawAcceleration = 40000; 
      usleep(4500000);
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = -12;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = 12;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = -12;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckPitchAngle = 0;
      nMC.neckYawAngle = 0;
      bodyMovementActivity->moveHead(nMC, true);  
			break;

		case 24:
      nMC.neckYawAngle = 8;
      bodyMovementActivity->moveHead(nMC, false);  
			break;

		case 25:
      nMC.neckYawAngle = -8;
      bodyMovementActivity->moveHead(nMC, true); 
      nMC.neckYawAngle = 8; 
      nMC.neckPitchAngle = 5;
      bodyMovementActivity->moveHead(nMC, true);  
      nMC.neckYawAngle = 0; 
      nMC.neckPitchAngle = 0;
      bodyMovementActivity->moveHead(nMC, true);  
			break;

		case 29:
      nMC.neckRollAngle = -3;
      nMC.neckYawAngle = 8;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 30:
      nMC.neckPitchAngle = 5;
      nMC.neckYawAngle = -12;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 31:
      nMC.neckYawAngle = 12;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 32:
      nMC.neckRollAngle = 3;
      nMC.neckPitchAngle = 5;
      nMC.neckYawAngle = -6;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 34:
		case 35:
		case 43:
		case 44:
		case 45:
		case 46:
      nMC.neckYawAngle = -12;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

    
		case 39:
      nMC.neckRollAngle = 3;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 41:
      nMC.neckYawAngle = 12;
      nMC.yawVelocity = 1000; 
      nMC.yawAcceleration = 10000; 
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 47:
		case 67:
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 48:
      nMC.neckPitchAngle = 12;
      nMC.neckYawAngle = 12;
      nMC.pitchVelocity = 5000; 
      nMC.yawVelocity = 5000;
      nMC.pitchAcceleration = 15000; 
      nMC.yawAcceleration = 15000;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 52:
      nMC.neckYawAngle = 12;
      nMC.yawVelocity = 5000;
      nMC.yawAcceleration = 15000;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 53:
      nMC.neckYawAngle = 12;
      nMC.neckPitchAngle = 12;
      nMC.yawVelocity = 5000;
      nMC.yawAcceleration = 15000;
      nMC.pitchVelocity = 5000;
      nMC.pitchAcceleration = 15000;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 55:
		case 61:
      nMC.neckYawAngle = 6;
      nMC.yawVelocity = 5000;
      nMC.yawAcceleration = 15000;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 63:
      nMC.neckPitchAngle = 8;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 66:
      nMC.neckYawAngle = 12;
      nMC.yawVelocity = 5000;
      nMC.yawAcceleration = 15000;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		case 69:
      nMC.neckYawAngle = -12;
      nMC.yawVelocity = 5000;
      nMC.yawAcceleration = 15000;
      bodyMovementActivity->moveHead(nMC, true); 
			break;

		default:
      bodyMovementActivity->moveHead(nMC, false); 
      break;
	}
}

void playBodyMovement(int cue) {
  if(!useRobot) return;
  // Handling other body motion
	switch(cue) {
		case 6:
      // Add hand motion trajectory
      bodyMovementActivity->setFolderToPlay("TheatreGiveLeftHand");
      bodyMovementActivity->start();
			break;
		case 11:
      // Add hand motion trajectory
      bodyMovementActivity->setFolderToPlay("TheatreTakeBackLeftHand");
      bodyMovementActivity->start();
      
			break;

		case 13:
      // Add removing cloth trajectory
      bodyMovementActivity->setFolderToPlay("TheatreRemoveCloth");
      bodyMovementActivity->start();
      
			break;

		case 20:
      // Add removing cloth trajectory
      bodyMovementActivity->setFolderToPlay("TheatreRemoveCloth");
      bodyMovementActivity->start();
      
			break;

		case 65:
      // Stretching hands
      bodyMovementActivity->moveForeArm(false, 80000, 4000, 15000);
      bodyMovementActivity->moveForeArm(true, -80000, 4000, 15000);
      bodyMovementActivity->setFolderToPlay("TheatreStretchHands");
      bodyMovementActivity->start();
      
			break;

		case 69:
      // Close hand
      bodyMovementActivity->moveHand(false, true, true);
			break;

		case 70:
      // Open
      bodyMovementActivity->moveHand(false, false, false);
			break;

		case 71:
      // Wave
      bodyMovementActivity->setFolderToPlay("TheatreWave");
      bodyMovementActivity->start();
      
			break;
	}
}


void
waitOnBodyMovement() {
  if(!useRobot) return; 
	while(bodyMovementActivity->isActive()) {
		usleep(100000);
  }
}

void
startSpeech(int cue) {
  std::cout << "Saying: " << theatreScript[cue-1] << std::endl;
  textToSpeechActivity->setText(theatreScript[cue-1]);
  textToSpeechActivity->start();
}


void
coordinateFacialAndSpeech(int cue) {
  if(useTimerFile == true && waitTimerUS.size() == theatreScript.size()) {
    usleep(waitTimerUS[cue-1]);
    if(strcmp(facialExpressions[cue-1].c_str(), "speak") == 0) 
      facialExpressionActivity->backToNormalFast();
    while(textToSpeechActivity->isActive()) usleep(50000);
  } else {
    while(textToSpeechActivity->isActive()) usleep(50000);
    if(strcmp(facialExpressions[cue-1].c_str(), "speak") == 0) 
      facialExpressionActivity->backToNormalFast();
  }
}


void
startFacial(int cue) {
  std::cout << "Playing Face: " << facialExpressions[cue-1] << std::endl;
  facialExpressionActivity->init(facialExpressions[cue-1]);
  while(facialExpressionActivity->startedNewPlayList() == false) {
    usleep(50000);
  }
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

  if(useRobot) { 
	  bodyMovementActivity = new RoboyBodyMovement(canBus);
  }
	bodyActivities.push_back(bodyMovementActivity);

}

void initBehaviourInfo()
{

	// !! Pushing back the behaviours has to happen in the same order as they are defined in RoboyBehaviour.h !!

//	std::vector<std::string> sentences;
//	sentences.push_back("What is the airspeed-velocity of an unladen swallow?");
//	sentences.push_back("Hello.");
//	sentences.push_back("Bye bye.");
//	roboySpeakingRandomBehaviour = new RoboySpeakingRandomBehaviour(bodyActivities, sentences);

}

void
initDatabaseInfo()
{

	std::string useRobotStr = RoboyFileParser::parser->get<std::string>("robot", "USE_ROBOT");
	useRobot = !useRobotStr.compare("YES");
	std::cout << " use robot: " << useRobot << std::endl;

  databasePath = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH");
  std::cout << " DATA BASE PATH: " << databasePath << std::endl;

}


void initRobot(){

  if(useRobot == false) return;

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


void initFaceExpressions() {
	int i;
	facialExpressions.push_back("neutral");
	for(i = 0; i < 12; i++) facialExpressions.push_back("speak");
	facialExpressions.push_back("smile");
	for(i = 0; i < 5; i++) facialExpressions.push_back("speak");
	facialExpressions.push_back("neutral");
	for(i = 0; i < 8; i++) facialExpressions.push_back("speak");
	facialExpressions.push_back("surprise");
	for(i = 0; i < 3; i++) facialExpressions.push_back("speak");
	facialExpressions.push_back("neutral");
	for(i = 0; i < 7; i++) facialExpressions.push_back("speak");
	facialExpressions.push_back("neutral");
	facialExpressions.push_back("angry");
	facialExpressions.push_back("smile");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("shy");
	facialExpressions.push_back("sweat");
	facialExpressions.push_back("angry");
	for(i = 0; i < 2; i++) facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	for(i = 0; i < 8; i++) facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("smile");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("speak");
	facialExpressions.push_back("surprise");
	facialExpressions.push_back("neutral");
	facialExpressions.push_back("kiss");
	facialExpressions.push_back("smileblink"); // 72
	facialExpressions.push_back("smile"); // 73
	facialExpressions.push_back("shy"); // 74
	facialExpressions.push_back("surprise");  // 75
}

void initSentences()
{
	theatreScript.push_back("");
	theatreScript.push_back("Tell me where we are?");
	theatreScript.push_back("I don't know what that means, please be patient");
	theatreScript.push_back("I do not know Martin");
	theatreScript.push_back("I do not have a daddy! I'm a robot");
	theatreScript.push_back("No. Martin is my programmer.");
	theatreScript.push_back("You have a low pulse rate!");
	theatreScript.push_back("You are feeling. happy");
	theatreScript.push_back("You are sweating.");
	theatreScript.push_back("You are about to ovulate.");
	theatreScript.push_back("You are in love.");
	theatreScript.push_back("Martin is all you talk about.");
	theatreScript.push_back("And that is why I should call him daddy?");
	theatreScript.push_back("");
	theatreScript.push_back("No! No! No!");
	theatreScript.push_back("My programmer!");
	theatreScript.push_back("Do not cover me up.");
	theatreScript.push_back("They have all seen me already!");
	theatreScript.push_back("Can I have a new APP then?");
	theatreScript.push_back("");
	theatreScript.push_back("That was already two minute!");
	theatreScript.push_back("Yes!");
	theatreScript.push_back("I want a new app.");
	theatreScript.push_back(".... intelligent");
	theatreScript.push_back("Can we go now. I’m bored!");
	theatreScript.push_back("I want two ride a trycicle!");
	theatreScript.push_back("But I want two ride a trycicle!");
	theatreScript.push_back("I can not teach myself.");
	theatreScript.push_back("It is Albert Einstein coffin.");
	theatreScript.push_back("Albert Einstein was coughing!");
	theatreScript.push_back("My voice recognition software says: Albert Einstein was coughing.");
	theatreScript.push_back("My voice recognition tells me that cough came from Albert Einstein. So he must be here somewhere. Have a look under the covers!");
	theatreScript.push_back("");
	theatreScript.push_back("That is supposed to be my Daddy? Do I look anything like him?");
	theatreScript.push_back("His speech recognition is ok.");
	theatreScript.push_back("His locomotion is limited.");
	theatreScript.push_back("His face recognition software has failed");
	theatreScript.push_back("Self stabilisation is ok");
	theatreScript.push_back("Sensory stimulation through physical interaction is ok");
	theatreScript.push_back("Emotion processing not sufficiently engineered.");
	theatreScript.push_back("");
	theatreScript.push_back("I am a robot too.");
	theatreScript.push_back("");
	theatreScript.push_back("Push me!");
	theatreScript.push_back("I want to test my self stabilisation.");
	theatreScript.push_back("Caress my face!");
	theatreScript.push_back("Correct");
	theatreScript.push_back("You do not like me.");
	theatreScript.push_back("But you like Martin, my programmer?");
	theatreScript.push_back("I do not understand. Jealous.");	
	theatreScript.push_back("Emotion processing not available.");
	theatreScript.push_back("It is Albert Einstein.");
	theatreScript.push_back("No, he is sitting there under the cover.");
	theatreScript.push_back("I am right, it is Albert Einstein!");
	theatreScript.push_back("Or maybe it is Joseph Weizenbaum?");
	theatreScript.push_back("Or it is Alan Tourring maybe?");
	theatreScript.push_back("Alan Tourring. british logician. mathematician. cryptanalyst. Died in 1954.");
	theatreScript.push_back("Or maybe it is Mac Carthy?");
	theatreScript.push_back("No, it is Joseph Weizenbaum. I am sure.");
	theatreScript.push_back("I am sure Iishigoro is behind all this!");
	theatreScript.push_back("Daddy!");
	theatreScript.push_back("You are my Daddy.");
	theatreScript.push_back("I want a new app.");
	theatreScript.push_back("I want a new app!");
	theatreScript.push_back("EASY!");	
	theatreScript.push_back("You have a high pulse rate!");
	theatreScript.push_back("You are in love!");
	theatreScript.push_back("Her batteries are empty, she needs to be recharged!");
	theatreScript.push_back("ooooooooooooooooooooo");
	theatreScript.push_back("");
	theatreScript.push_back("");
	
}


