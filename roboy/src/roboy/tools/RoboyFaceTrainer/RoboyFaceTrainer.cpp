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


/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http://www.opensource.org/licenses/bsd-license>
 */

// #include "opencv2/opencv.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


#include <math.h>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

//#include <cv.h>
//#include <highgui.h>

#include "RoboyVision.h" 
#include "RoboyFileParser.h"
#include "RoboyTransforms.h"
#include "RoboyFileSystem.h"
#include "RoboyFileWriter.h"

//#include "RoboyTransforms.h"
//#include "RoboyGeometry.h"
//#include "RoboyFileSystem.h"
//#include "RoboyFileWriter.h"


int main(int argc, const char *argv[]) {
	
	// check for command line arguments
	if (argc != 3 && argc != 5) {
		std::cerr << "usage:" << argv[0] << " <device_id> <face_name> <configuration_file>. Example:"  << std::endl;
		std::cerr << "Example1:" << argv[0] << " ../../../../../roboy/database/configuration/RoboyConfigurationFile.conf Sara "  << std::endl;
		std::cerr << "Example2:" << argv[0] << " ../../../../../roboy/database/configuration/RoboyConfigurationFile.conf Sara 100 100"  << std::endl;
		std::cerr << "where the last two numbers represent the width and height of the recorded samples." << std::endl;
		exit(-1);
	}

	int imWidth;
	int imHeight;

	if(argc > 3)
	{
		imWidth = atoi(argv[3]);
		imHeight = atoi(argv[4]);
	}else{
		imWidth = 75;
		imHeight = 75;
	}

	std::string configurationFile = argv[1];
	std::cerr << "Configuration file: " << configurationFile << std::endl; 
	RoboyFileParser::parser = new RoboyFileParser(configurationFile);

	Roboy::RoboyVision* vision = new Roboy::RoboyVision();
	vision->trainFaceDetector();

	//string fn_haar = "/home/hgmarques/Applications/OpenCV-2.4.1/data/haarcascades/haarcascade_frontalface_alt2.xml";
	std::string databaseFolder = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH");
	
	std::cerr << "DATABASE: " << databaseFolder << std::endl;



	int NUMBER_OF_TRAINING_SAMPLES = RoboyFileParser::parser->get<int>("vision", "NUMBER_OF_TRAINING_RECOGNITION_SAMPLES");

	std::string personName = argv[2];

	// Holds the current frame from the Video device:
	cv::Mat frame;

	int width, height;

	int frameCounter = 0;

	bool personPhotosExist = false;
	
	std::string personFolder = databaseFolder + "/people/" + personName;
	std::string photoFolder = personFolder + "/photos";
	std::cerr << "PERSON: " << personFolder << "  PHOTO: " << photoFolder << std::endl;
	
	if(Roboy::RoboyFileSystem::directoryExists(personFolder))
	{
		char typedChar;
		std::cerr << " warning: the person you provided (" << personName << ") already exists. \nIf you continue you will add new photo samples to existing person. Do you want to continue? (y/n): ";
		std::cin >> typedChar;

		if (typedChar != 'y') 
			return 0;
	
		if(!Roboy::RoboyFileSystem::directoryExists(photoFolder))
			Roboy::RoboyFileSystem::createDirectory(photoFolder);
			
	}else{
		Roboy::RoboyFileSystem::createDirectory(personFolder);
		Roboy::RoboyFileSystem::createDirectory(photoFolder);
	}

	
	int currentSampleNr = 0;
	
	while(1)
	{
		vision->captureFrame();
		cv::Mat frame = vision->getGrayFrame();
		cv::Mat guiFrame = vision->getGUIFrame();
		//CV_GRAY2BGR

		
		width = (int)(frame.size().width);
		height = (int)(frame.size().height);

		// Find the faces in the frame:
		std::vector< cv::Rect_<int> > faces = vision->detectFaces();
		for(int i = 0; i < faces.size(); i++) 
		{
			cv::Rect_<int> face = vision->convertFromScaledFrameToOriginalFrameCoordinates(faces[i]);
			cv::rectangle(guiFrame, face, CV_RGB(0, 255,0), 1);

			std::string imgSize = cv::format("%d x %d", faces[i].width, faces[i].height);
			putText(guiFrame, imgSize, cv::Point(face.x, face.y+face.height+12), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
		}

		if ( frameCounter > 6 && faces.size() == 1 )
		{  
			 
			if(faces[0].width >= imWidth && faces[0].height >= imHeight)
			{
				cv::Rect_<int> faceOri = vision->convertFromScaledFrameToOriginalFrameCoordinates(faces[0]); 
							
				cv::rectangle(guiFrame, faceOri, CV_RGB(255, 0,0), 1);

				cv::Mat faceimg = frame(faces[0]);

				cv::Mat trimmed = Roboy::RoboyTransforms::trimImageToMaxSize(faceimg, cvSize(imWidth, imHeight));

//				std::string destFile = cv::format("%s%s%d.jpg", destFolder.c_str(), personName.c_str(), imgCounter); 

				bool fileExists = true;
				
				while (1) 
				{
					
					std::stringstream strNr;
					strNr << currentSampleNr;
					std::string sampleFile = photoFolder + "/" + personName + strNr.str() + ".jpg";

					if(!Roboy::RoboyFileSystem::fileExists(sampleFile))
					{
						std::cerr << "adding sample file: " << sampleFile << std::endl;
						cv::imwrite( sampleFile, trimmed );
						currentSampleNr++;
						break;
					}
					
					currentSampleNr++;
					
				}
				
				frameCounter = 0;
			}
		}

		frameCounter++;

//		cv::imshow("Roboy Face Trainer", frame);
		cv::imshow("Roboy Face Trainer", guiFrame);
		//   imshow("face_recognizer", gray);

		// And display it:
		char key = (char) cv::waitKey(20);

		// Exit this loop on escape:
		if(key == 27)
			break;

		if (currentSampleNr >= NUMBER_OF_TRAINING_SAMPLES)
			break;
	}

	
	return 0;
}



