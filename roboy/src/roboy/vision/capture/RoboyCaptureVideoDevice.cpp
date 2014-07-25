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



#include <iostream>

#include "RoboyCaptureVideoDevice.h"

namespace Roboy
{

RoboyCaptureVideoDevice::RoboyCaptureVideoDevice(int deviceNr) : RoboyCapture(),
capture(deviceNr)
{

	this->deviceNr = deviceNr; 
	
	// check if capture device can be accessed to
	if(!capture.isOpened()) 
	{
		std::cerr << "Capture Device Nr " << deviceNr << " cannot be opened." << std::endl;
	}

	std::cerr << "Capturing from videodevice constructor" << std::endl;
	
}

RoboyCaptureVideoDevice::~RoboyCaptureVideoDevice() 
{

}

void
RoboyCaptureVideoDevice::start()
{

}



void
RoboyCaptureVideoDevice::init()
{

}

void 
RoboyCaptureVideoDevice::execute()
{

}



void
RoboyCaptureVideoDevice::terminate()
{

}


cv::Mat
RoboyCaptureVideoDevice::getFrame()
{
//	cv::Mat original;
	capture >> frame;	
//	cvtColor(original, frame, CV_BGR2GRAY);
	
	return frame;
}


std::string
RoboyCaptureVideoDevice::getInfo()
{
	return "video device";      
}

}
