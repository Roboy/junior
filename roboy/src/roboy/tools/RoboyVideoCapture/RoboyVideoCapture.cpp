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
		std::cerr << "Example1:" << argv[0] << " ../../../../../roboy/database/configuration/RoboyConfigurationFileRobotVisionInteraction.conf file "  << std::endl;
		std::cerr << "Example2:" << argv[0] << " ../../../../../roboy/database/configuration/RoboyConfigurationFileRobotVisionInteraction.conf file 100 100"  << std::endl;
		std::cerr << "where the last two numbers represent the width and height of the recorded samples." << std::endl;
		exit(-1);
	}

	int imWidth;
	int imHeight;

	std::string path = argv[2];

	if(!Roboy::RoboyFileSystem::directoryExists(path))
	{
		std::cerr << "[Error] unknown video directory path." << std::endl;
		exit(-1);
	}

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

	//string fn_haar = "/home/hgmarques/Applications/OpenCV-2.4.1/data/haarcascades/haarcascade_frontalface_alt2.xml";
	std::string databaseFolder = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH");

	std::cerr << "DATABASE: " << databaseFolder << std::endl;

	// Holds the current frame from the Video device:
	cv::Mat frame;

	int width, height;

	int frameCounter = 0;
	int currentSampleNr = 0;

	std::vector<int> quality;
	quality.push_back(CV_IMWRITE_JPEG_QUALITY);
	quality.push_back(90);

	while(1)
	{
		vision->captureFrame();
		cv::Mat frame = vision->getGrayFrame();
		cv::Mat guiframe = vision->getGUIFrame();
		//CV_GRAY2BGR

		width = (int)(frame.size().width);
		height = (int)(frame.size().height);

		cv::imshow("Roboy Video Capture", guiframe);


		std::stringstream ss;
		ss << path << "/img" << frameCounter << ".jpg";
		std::cout << "saving in: " << ss.str() << std::endl;

		cv::imwrite(ss.str(), guiframe, quality);

		// And display it:
		char key = (char) cv::waitKey(20);

		// Exit this loop on escape:
		if(key == 27)
			break;

		frameCounter++;
	}


	return 0;
}



