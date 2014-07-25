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
#include <fstream>
#include <sstream>

#include "RoboyFileParser.h"
#include "RoboyVision.h"

namespace Roboy
{

// FIXME: the frame rate should move to the RoboyCapture

RoboyVision::RoboyVision( ) :
																frameRate(1)
{

	getParametersFromConfigurationFile();



	// FIXME: this should all be handled in the RoboyCapture infrastructure when the mvBlueFox is fully fixed

	capture->start();

	useFaceRecognition = false;
	std::cerr << " ROBOY VISION DELETE THIS"<< std::endl;

	if(!strcmp("mvBlueFox", device.c_str()))
	{
		usleep(8000000);
	}

	//	deviceID = "/dev/bus/usb/002/010"
	//	this->device = device;
	//	this->deviceID = atoi(deviceID.c_str());

	//atoi(deviceID.c_str())

	//	std::cerr << "OLE" << atoi("/dev/bus/usb/002/010") << " cannot be opened." << std::endl;

	//	// check if capture device can be accessed to
	//	if(!capture.isOpened()) {
	//		std::cerr << "Capture Device ID " << deviceID << " cannot be opened." << std::endl;
	//	}

}


RoboyVision::~RoboyVision()
{

}

cv::Mat
RoboyVision::captureFrame()
{
	originalFrame = capture->getFrame();

	applyRoboyFrameTransforms();

	updateFrameRateEstimate();

	return originalFrame;
}



std::vector< cv::Rect_<int> > 
RoboyVision::detectFaces()
{
	std::vector< cv::Rect_<int> > faces; 

	faceClassifier.detectMultiScale(grayFrame, faces);

	updateFaceDetectionEstimates( faces.size() );

	this->faces = faces;

  if(faces.size() == 0)
  {
    faceExists = false;  
  }else{
    faceExists = true;
    faceX = faces[0].x;
    faceY = faces[0].y;
    faceW = faces[0].width;
    faceH = faces[0].height;
  }

	return faces;
}

bool
RoboyVision::getFaceData(int& faceX, int& faceY, int& faceW, int& faceH)
{
  faceX = this->faceX;
  faceY = this->faceY;
  faceW = this->faceW;
  faceH = this->faceH;

  return faceExists;
} 

void 
RoboyVision::recognizeFaces()
{

	// Resizing the face is necessary for Eigenfaces and Fisherfaces. You can easily
	// verify this, by reading through the face recognition tutorial coming with OpenCV.
	// Resizing IS NOT NEEDED for Local Binary Patterns Histograms, so preparing the
	// input data really depends on the algorithm used.
	//
	// I strongly encourage you to play around with the algorithms. See which work best
	// in your scenario, LBPH should always be a contender for robust face recognition.
	//
	// Since I am showing the Fisherfaces algorithm here, I also show how to resize the
	// face you have just found:

	faceRecognitionPredictions.clear();
	faceRecognitionConfidences.clear();

	for(int i = 0; i < faces.size(); i++) {

		cv::Rect face_i = faces[i];

		// Crop the face from the image. So simple with OpenCV C++:
		cv::Mat face = grayFrame(face_i);

		cv::Mat face_resized;

		cv::resize(face, face_resized, cv::Size(RECOGNITION_FACE_WIDTH, RECOGNITION_FACE_HEIGHT), 1.0, 1.0, cv::INTER_CUBIC);

		int prediction = -1;
		double confidence = 0.0;

		recognitionModel->predict(face_resized, prediction, confidence);
		faceRecognitionPredictions.push_back(prediction);
		faceRecognitionConfidences.push_back(confidence);


	}

}

float
RoboyVision::getAverageNumberofFacesDetected()
{
	return avgNumberOfFacesDetected;
}

int
RoboyVision::getDeviceID()
{
	return deviceID;
}

std::string 
RoboyVision::getDevice()
{
	return device;
}

void 
RoboyVision::setDevice(std::string device)
{
	this->device = device;
}

cv::Mat 
RoboyVision::getOriginalFrame()
{
	return originalFrame;
}

cv::Mat 
RoboyVision::getScaledFrame()
{
	return scaledFrame;
}

cv::Mat 
RoboyVision::getGrayFrame()
{
	return grayFrame;
}

cv::Mat 
RoboyVision::getGUIFrame()
{
	return guiFrame;
}


void
RoboyVision::setFaceDetectionClassifier(std::string faceClassifierFile)
{
	std::cerr << "Face classifier selected: " << faceClassifierFile << std::endl;

	this->FACE_DETECTION_CLASSIFIER_FILE = FACE_DETECTION_CLASSIFIER_FILE;

	faceClassifier.load(FACE_DETECTION_CLASSIFIER_FILE);

}

void
RoboyVision::setFaceDetectionAverageWindowSize(int faceDetectionAvgWindowSize)
{
	this->FACE_DETECTION_AVG_WINDOW_SIZE = FACE_DETECTION_AVG_WINDOW_SIZE; 
}


float
RoboyVision::getFrameScale( )
{
	return FRAME_SCALE;
}

void
RoboyVision::setFrameScale(float frameScale)
{
	this->FRAME_SCALE = frameScale;
}

float 
RoboyVision::getFrameRate()
{
	return 1/(frameRate);
}


std::vector< cv::Rect_<int> >
RoboyVision::getFaces()
{
	return faces;
}

void
RoboyVision::trainFaceDetector()
{
	faceClassifier.load(FACE_DETECTION_CLASSIFIER_FILE);
}

void 
RoboyVision::trainFaceRecognizer()
{

	std::string filename = FACE_RECOGNITION_TRAINING_SET_FILE;
	std::vector<cv::Mat> images;
	std::vector<int> labels;

	// check if the filename is valid
	std::ifstream file(filename.c_str(), std::ifstream::in);
	if (!file) {
		std::string error_message = "No valid input file was given, please check the given filename.";
		CV_Error(CV_StsBadArg, error_message);
	}

	std::string line, path, classlabel;

	// read input file
	while (getline(file, line)) {
		std::stringstream liness(line);
		std::getline(liness, path, ';');
		std::getline(liness, classlabel);

		std::cerr << " path: " << path << ", " << classlabel  << std::endl;

		if(!path.empty() && !classlabel.empty()) {
			//			std::cout << path << std::endl;
			//			imshow("OLE", cv::imread(path, 0));
			images.push_back(cv::imread(path, 0));
			labels.push_back(atoi(classlabel.c_str()));
		}
	}

	//    recognitionModel = cv::createFisherFaceRecognizer();
	//    recognitionModel = cv::createLBPHFaceRecognizer(1, 8, 8, 8, DBL_MAX);

	//recognitionModel = cv::createLBPHFaceRecognizer(1, 8, 8, 8, 50);
	recognitionModel = cv::createEigenFaceRecognizer(80, DBL_MAX);
	recognitionModel->train(images, labels);

}


float
RoboyVision::getRecognitionConfidenceThreshold()
{
	return RECOGNITION_CONFIDENCE_THRESHOLD;
}

int 
RoboyVision::getRecognitionPrediction(int detectionFaceIndex)
{
	return faceRecognitionPredictions[detectionFaceIndex];
}

double
RoboyVision::getRecognitionConfidence(int detectionFaceIndex)
{
	return faceRecognitionConfidences[detectionFaceIndex];
}  


//std::vector<int>
//getFaceRecognitionVector();
//
//std::vector<float>
//getFaceRecognitionConfidenceValues();


void
RoboyVision::resetRecognitionRecord()
{
	for(int i=0; i<persons.size(); i++)
	{
		persons[i]->resetRecognitionRecord();
	}
}

void
RoboyVision::updateRecognitionRecord()
{

	int index;

	for(int i=0; i<persons.size(); i++)
	{
		index = getRecognitionIndex(i);

		if(index != -1)
		{
			persons[i]->updateRecognitionRecord(true, this->faceRecognitionConfidences[index]);
			std::cout << "RECOGNITION - person recognized: " << persons[i]->getName() << std::endl;
		} else {
			persons[i]->updateRecognitionRecord(false, 0);
		}


	}

}

int
RoboyVision::getRecognizedPerson()
{

	int index;

	for(int i=0; i<persons.size(); i++)
	{

		index = faceRecognitionPredictions[i];

		if(!persons[i]->hasBeenRecognized() && persons[i]->getRecognitionCount() > 5)
		{
			return i;
		}

	}

	return -1;
}


void
RoboyVision::setPeopleList(std::vector<RoboyPerson*> persons)
{
	this->persons = persons;
}



void
RoboyVision::applyRoboyFrameTransforms()
{

	scaledWidth = (int)(originalFrame.size().width * FRAME_SCALE);
	scaledHeight = (int)(originalFrame.size().height * FRAME_SCALE);

	resize(originalFrame, scaledFrame, cv::Size(scaledWidth, scaledHeight));

	// FIXME: this should ask whether the originalImage is black and white
	if(!strcmp("mvBlueFox", device.c_str()))
	{
		cvtColor(originalFrame, guiFrame, CV_GRAY2BGR);
		grayFrame = scaledFrame;
	} else {
		cvtColor(scaledFrame, grayFrame, CV_BGR2GRAY);
		guiFrame = originalFrame;
	}



}


void
RoboyVision::updateFaceDetectionEstimates(int nFaces){

	// Update number of faces estimation

	faceDetectionWindow.push_back(nFaces);

	if (faceDetectionWindow.size() > FACE_DETECTION_AVG_WINDOW_SIZE)
	{
		faceDetectionWindow.pop_front();

		std::list<int>::iterator i;
		int sum = 0;

		for(i=faceDetectionWindow.begin(); i != faceDetectionWindow.end(); ++i)
			sum += *i;

		avgNumberOfFacesDetected = (float) sum / (float) FACE_DETECTION_AVG_WINDOW_SIZE;

	}else{ 

		avgNumberOfFacesDetected = 0;
	}

	// Update time since last face estimation;
	if(avgNumberOfFacesDetected > FACE_DETECTION_THRESHOLD && noFaceTimer.isActive())
	{
		noFaceTimer.stop();
		noFaceTimer.reset();
	}

	if (avgNumberOfFacesDetected < NO_FACE_DETECTION_THRESHOLD && !noFaceTimer.isActive())
	{
		noFaceTimer.start();
	}

}

cv::Rect_<int>
RoboyVision::convertFromScaledFrameToOriginalFrameCoordinates(cv::Rect_<int> rect)
{
	int origX0 = originalFrame.size().width / 2; 
	int origY0 = originalFrame.size().height / 2; 	
	int scaledX0 = scaledFrame.size().width / 2; 
	int scaledY0 = scaledFrame.size().height / 2; 

	int x0 = origX0 - (scaledX0 - rect.tl().x) / FRAME_SCALE;
	int y0 = origY0 - (scaledY0 - rect.tl().y) / FRAME_SCALE;
	int w = rect.size().width / FRAME_SCALE;
	int h = rect.size().height / FRAME_SCALE;

	cv::Rect_<int> scaledRect ( x0, y0, w, h );
	return scaledRect;
}


double
RoboyVision::getTimeSinceLastFaceDetected()
{
	return noFaceTimer.getTime();
}


void
RoboyVision::updateFrameRateEstimate()
{
	if(frameRateTimer.isActive()){      
		frameRate = (frameRate * (FACE_DETECTION_AVG_WINDOW_SIZE - 1) + frameRateTimer.getTime()) / FACE_DETECTION_AVG_WINDOW_SIZE;
	}

	frameRateTimer.start();
}


int
RoboyVision::getRecognitionIndex(int person)
{

	for(int i=0; i<faceRecognitionPredictions.size(); i++)
		if(person == faceRecognitionPredictions[i])
			return i;

	return -1;
}



void
RoboyVision::getParametersFromConfigurationFile()
{
	std::string databasePath = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH");

//	std::string useVision = RoboyFileParser::parser->get<std::string>("vision", "USE_VISION");
//	USE_VISION = !useVision.compare("YES");

	FRAME_SCALE = RoboyFileParser::parser->get<float>("vision", "FRAME_SCALE");
	RECOGNITION_FACE_WIDTH = RoboyFileParser::parser->get<int>("vision", "RECOGNITION_FACE_WIDTH");
	RECOGNITION_FACE_HEIGHT = RoboyFileParser::parser->get<int>("vision", "RECOGNITION_FACE_HEIGHT");
	FACE_DETECTION_AVG_WINDOW_SIZE = RoboyFileParser::parser->get<int>("vision", "FACE_DETECTION_AVG_WINDOW_SIZE");
	RECOGNITION_CONFIDENCE_THRESHOLD = RoboyFileParser::parser->get<float>("vision", "RECOGNITION_CONFIDENCE_THRESHOLD");
	FACE_DETECTION_THRESHOLD = RoboyFileParser::parser->get<float>("vision", "FACE_DETECTION_THRESHOLD");
	NO_FACE_DETECTION_THRESHOLD = RoboyFileParser::parser->get<float>("vision", "NO_FACE_DETECTION_THRESHOLD");

	std::string classifierFile = RoboyFileParser::parser->get<std::string>("vision", "FACE_DETECTION_CLASSIFIER_FILE");
	FACE_DETECTION_CLASSIFIER_FILE = databasePath + "/" + classifierFile;

	FACE_RECOGNITION_TRAINING_SET_FILE =  RoboyFileParser::parser->get<std::string>("vision", "FACE_RECOGNITION_TRAINING_SET_FILE");

	device = RoboyFileParser::parser->get<std::string>("vision", "CAPTURE_HARDWARE");

	useVision = device.compare("novideo");

	std::cout << "Hardware video device: " << device << std::endl;


	if(!strcmp("mvBlueFox", device.c_str()))
	{

		int deviceNr = RoboyFileParser::parser->get<int>("vision", "CAPTURE_DEVICE");

		std::cout << "Hardware video device nr: " << deviceNr << std::endl;

		capture = new RoboyCaptureMVBlueFox(deviceNr);

	}else if (!strcmp("videodevice", device.c_str()))
	{

		std::string devicePath = RoboyFileParser::parser->get<std::string>("vision", "CAPTURE_DEVICE");

		std::cout << "Hardware video device path: " << devicePath << std::endl;

		capture = new RoboyCaptureVideoDevice(atoi(devicePath.c_str()));

	}else if (!strcmp("novideo", device.c_str())){

		NO_VISION_FILE_PATH = databasePath + "/" + RoboyFileParser::parser->get<std::string>("vision", "NO_VISION_FILE_PATH");

		std::cout << "Hardware video device path: NO VIDEO " << std::endl;

		capture = new RoboyCaptureNoVideo(NO_VISION_FILE_PATH);

	} else {
		std::cerr << "[Error] Video capture device not recognized." << std::endl;
	}

}







/*

  void 
  RoboyVision::readCSV(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, char separator) 
  {
    std::ifstream file(filename.c_str(), std::ifstream::in);

    if (!file)
      throw std::exception();
    std::string line, path, classlabel;

    while (getline(file, line)) {
      std::stringstream liness(line);
      std::getline(liness, path, separator);
      std::getline(liness, classlabel);
      images.push_back(cv::imread(path, 0));
      labels.push_back(atoi(classlabel.c_str()));
    }
  }



  cv::Ptr<cv::FaceRecognizer> 
  RoboyVision::trainFaceRecognizer(const std::string& filename, char separator) 
  {

    std::vector<cv::Mat> images;
    std::vector<int> labels;

    std::ifstream file(filename.c_str(), std::ifstream::in);
    if (!file) {
      std::string error_message = "No valid input file was given, please check the given filename.";
      CV_Error(CV_StsBadArg, error_message);
    }

    std::string line, path, classlabel;

    while (getline(file, line)) {
      std::stringstream liness(line);
      std::getline(liness, path, separator);
      std::getline(liness, classlabel);

      if(!path.empty() && !classlabel.empty()) {
        images.push_back(cv::imread(path, 0));
        labels.push_back(atoi(classlabel.c_str()));
      }
    }

    cv::Ptr<cv::FaceRecognizer> model = cv::createFisherFaceRecognizer();
    model->train(images, labels);

    return model;
  }

 */


}
