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



#include <fstream>
#include <sstream>
#include <iostream>

#include "RoboyVisionInteractionGUI.h"
#include "RoboyFileSystem.h"

namespace Roboy{

RoboyVisionInteractionGUI::RoboyVisionInteractionGUI(std::string name, RoboyVision* vision)
{
	this->name = name;
	this->vision = vision;
	this->showDebugInfo = true;
	this->showHelp = true;
	this->showDemoInfo = false;
	this->showBehaviouralInfo = false;
	this->showVisionInfo = true;
}

RoboyVisionInteractionGUI::~RoboyVisionInteractionGUI()
{

}


char
RoboyVisionInteractionGUI::show(RoboyBehaviour* behaviour)
{

	// FIXME: this needs cleanup the original frame is not really needed here
	cv::Mat scaledFrame = vision->getScaledFrame();
	int scaledX0 = scaledFrame.size().width / 2; 
	int scaledY0 = scaledFrame.size().height / 2; 

	cv::Mat guiFrame = vision->getGUIFrame();
	int origX0 = guiFrame.size().width / 2; 
	int origY0 = guiFrame.size().height / 2; 
	float scaleFactor = 1 / vision->getFrameScale();

//	std::cerr << "sca: " << scaledX0 << "," << scaledY0 << "  .:.  orig: " << origX0 << "," << origY0 << std::endl;

	if(showDebugInfo)
	{
		// show number of faces detected
		std::string textNumberOfFaces = cv::format("faces: %2.1f", vision->getAverageNumberofFacesDetected());
		cv::putText(guiFrame, textNumberOfFaces, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(0,255,0), 1.0);

		// show time elapsed without faces
		std::string textNoFaceTime = cv::format("time since last face: %4.1f", (float) vision->getTimeSinceLastFaceDetected());
		cv::putText(guiFrame, textNoFaceTime, cv::Point(5, 30), cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(0,255,0), 1.0);

		// show frame rate 
		std::string textFrameRate = cv::format("FR: %1.1f Hz", (float) vision->getFrameRate());
		cv::putText(guiFrame, textFrameRate, cv::Point(5, 200), cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(0,255,0), 1.0);

	}

	if(showDemoInfo)
	{
		// show number of faces detected
		std::string textNumberOfFaces = cv::format("NO FACES DETECTED");
		cv::putText(guiFrame, textNumberOfFaces, cv::Point(50, 200), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB(0,255,0), 2.0);

	}

	if(showVisionInfo)
	{
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
			putText(guiFrame, imgSize, cv::Point(face.x, face.y+face.height+12), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

			if (vision->useFaceRecognition)
			{
				int textX = std::max(face.x - 10, 0);
				int textY = std::max(face.y - 10, 0);

				// show predicted face
				std::string textRecognition;
				if(vision->getRecognitionPrediction(i) > -1)
				{

					if(persons.size() != 0)
					{
						std::string dirPath = persons[vision->getRecognitionPrediction(i)]->getName();
						std::string name = RoboyFileSystem::getDirectoryName(dirPath);
						textRecognition = cv::format("%s: %1.2f", name.c_str(), vision->getRecognitionConfidence(i));
					}else{
						textRecognition = cv::format("%d: %1.2f", vision->getRecognitionPrediction(i), vision->getRecognitionConfidence(i));
					}

				}else{
					textRecognition = cv::format("unknown");
				}

				putText(guiFrame, textRecognition, cv::Point(textX, textY), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
			}
		}
	}

	if(showBehaviouralInfo)
	{
		// show number of faces detected
		std::string textBehaviour = cv::format(behaviour->getInfo().c_str());
		cv::putText(guiFrame, textBehaviour, cv::Point(50, 100), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(200,200,0), 1.5);
	}


	imshow(name.c_str(), guiFrame);

	char key = (char) cv::waitKey(10);

	if ( key == KEY_D )
		showDebugInfo = !showDebugInfo;

	if ( key == KEY_M )
		showDemoInfo = !showDemoInfo;

	if ( key == KEY_B ) 
		showBehaviouralInfo = !showBehaviouralInfo;

	if ( key == KEY_V ) 
		showVisionInfo = !showVisionInfo;

	if ( key == KEY_H )
		showHelp = !showHelp;

	if(key == KEY_R)
		vision->useFaceRecognition = !vision->useFaceRecognition;


	return key;

}


void
RoboyVisionInteractionGUI::setPersonList( std::vector<RoboyPerson*> persons)
{
	this->persons = persons;
}


}

