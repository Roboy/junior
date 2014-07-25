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

//#include "RoboyFileWriter.h"
#include "RoboyFileSystem.h"
#include "RoboyFileParser.h"
#include "RoboyTimer.h"
#include "RoboyBodyActivityRepertoire.h"
#include "RoboyBehaviouralRepertoire.h"

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


/** Roboy behaviour methods and datastructures */

// RoboyGazingBehaviour* gazingBehaviour;

RoboyFollowingFaceBehaviour* followingBehaviour;

void initBehaviourInfo();


/* Roboy configuration files and datastructures */

std::string configurationFile;

void initDatabaseInfo();


/* Roboy init vision */

RoboyVision* vision;

void initVision();

char visionShow();


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
		std::cerr << "Example: ./RoboyIntroduction ../../../../../roboy/database/configuration/RoboyConfigurationFileRobotVisionInteraction.conf" << std::endl;
		return 0;
	}

	configurationFile = argv[1];

	std::cout << "Loading " << configurationFile << std::endl; 

	std::cout << "CALLING initFileParserInfo()" << std::endl;
	initFileParserInfo();
	std::cout << "CALLING initDatabaseInfo()" << std::endl;
	initDatabaseInfo();
	std::cout << "CALLING initVision()" << std::endl;
	initVision();
	std::cout << "CALLING initRobot()" << std::endl;
	initRobot();
	std::cout << "CALLING initBodyActivityInfo()" << std::endl;
	initBodyActivityInfo();
	std::cout << "CALLING initBehaviourInfo()" << std::endl;
	initBehaviourInfo();
std::cout << "CALLING initBehaviourInfo()" << std::endl;

	char keyPressed = 48;

	followingBehaviour->start();
	

	while(!followingBehaviour->isFinished())
	{
	//	vision->captureFrame();
	//	vision->detectFaces();

		char key = visionShow();

		//std::cout << "KEY: " << key << std::endl;


		if(key == 27)
		{
			std::cout << " behaviour terminating " << std::endl;
			followingBehaviour->stop();
			std::cout << " behaviour terminate " << std::endl;
		}
//	while(!followingBehaviour->isFinished()) {
//		printf("Current status: %s\n", followingBehaviour->getInfo().c_str());
//		usleep(500000);
//	}

	}

RoboyListenToSound* listenToSoundActivity;
	std::cout << " behaviour terminated " << std::endl;

	for(unsigned int i=0; i<bodyActivities.size(); i++)
        delete bodyActivities[i];
	
//  usleep(1500000);

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

	if(useRobot)
		bodyMovementActivity = new RoboyBodyMovement(canBus);

	bodyActivities.push_back(bodyMovementActivity);

	listenToSoundActivity = new RoboyListenToSound();
	listenToSoundActivity->init("");
	bodyActivities.push_back(listenToSoundActivity);
}


void initBehaviourInfo()
{

	followingBehaviour = new RoboyFollowingFaceBehaviour(bodyActivities, canBus, vision);
}


void
initDatabaseInfo()
{
	std::string databasePath = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH");

	std::cout << " DATA BASE PATH: " << databasePath << std::endl;

	std::string useRobotStr = RoboyFileParser::parser->get<std::string>("robot", "USE_ROBOT");
		useRobot = !useRobotStr.compare("YES");

		std::cout << " use robot: " << useRobot << std::endl;


}

void initVision()
{
	vision = new RoboyVision();
	vision->trainFaceDetector();

}


void initRobot(){

	if(useRobot == false) return;

	canBus = new Robot("../../../../../roboy/", false);
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

char
visionShow()
{

	cv::Mat guiFrame = vision->getGUIFrame();

	std::vector< cv::Rect_<int> > faces = vision->getFaces();

	for(int i=0; i<faces.size(); i++)
	{

//			int x0 = origX0 - (scaledX0 - faces[i].tl().x) * scaleFactor;
//			int y0 = origY0 - (scaledY0 - faces[i].tl().y) * scaleFactor;
//			int w = faces[i].size().width * scaleFactor;
//			int h = faces[i].size().height * scaleFactor;
//			cv::Rect_<int> faceRect ( x0, y0, w, h ) ;

//			rectangle(guiFrame, faceRect, CV_RGB(0, 255,0), 1);

//			std::string sizeStr = cv::format("%s: %1.2f", name.c_str(), vision->getRecognitionConfidence(i));

		cv::Rect_<int> face = vision->convertFromScaledFrameToOriginalFrameCoordinates(faces[i]);
		cv::rectangle(guiFrame, face, CV_RGB(0, 255,0), 1);

		std::string imgSize = cv::format("%d x %d", faces[i].width, faces[i].height);
		cv::putText(guiFrame, imgSize, cv::Point(face.x, face.y+face.height+12), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

//			cv::putText(guiFrame, textRecognition, cv::Point(textX, textY), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
		}

	//std::cout << "vision display " << guiFrame.size().width << std::endl;

	cv::imshow("Face Detection", guiFrame);

	return (char) cv::waitKey(10);

}
