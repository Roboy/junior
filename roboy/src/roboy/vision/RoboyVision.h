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

#ifndef _ROBOY_VISION_H_
#define _ROBOY_VISION_H_


#include <string>
#include <list>

#include "RoboyPerson.h"
#include "opencv2/opencv.hpp"
#include "RoboyTimer.h"
#include "capture/RoboyCaptureDevices.h"

//#include <vector>

namespace Roboy {

class RoboyVision
{

public:

	RoboyVision( );

	virtual ~RoboyVision();

	/** This method also applies the necessary frame transforms for face detection and recognition */

	cv::Mat 
	captureFrame();

	std::vector< cv::Rect_<int> >
	detectFaces();

  bool
  getFaceData(int& faceX, int& faceY, int& faceW, int& faceH);

	void 
	recognizeFaces();

	float 
	getAverageNumberofFacesDetected();

	double
	getTimeSinceLastFaceDetected();

	int 
	getDeviceID();

	std::string 
	getDevice();

	void 
	setDevice(std::string deviceID);

	cv::Mat 
	getOriginalFrame();

	cv::Mat 
	getScaledFrame();

	cv::Mat 
	getGrayFrame();

	cv::Mat 
	getGUIFrame();

	void 
	setFaceDetectionClassifier(std::string faceClassifierFile);

	void
	setFaceDetectionAverageWindowSize(int faceDetectionAvgWindow);

	float
	getFrameScale();

	void 
	setFrameScale(float frameScale);

	float
	getFrameRate();

	std::vector< cv::Rect_<int> >
	getFaces();

	void
	trainFaceDetector();
	
	void
	trainFaceRecognizer();

	cv::Rect_<int>
	convertFromScaledFrameToOriginalFrameCoordinates(cv::Rect_<int> rect);

	float
	getRecognitionConfidenceThreshold();

	void
	setRecognitionConfidenceThreshold(float recognitionConfidenceThreshold);

	int 
	getRecognitionPrediction(int detectionFaceIndex);

	double 
	getRecognitionConfidence(int detectionFaceIndex);

	void
	resetRecognitionRecord();

	void
	updateRecognitionRecord();

	int
	getRecognizedPerson();

	void
	setPeopleList(std::vector<RoboyPerson*> personList);


private:

	void
	applyRoboyFrameTransforms();

	void
	updateFaceDetectionEstimates(int nFaces);

	void
	updateFrameRateEstimate();

	cv::Mat
	toGrayscale(cv::InputArray _src);

	int
	getRecognitionIndex(int person);

private:

	void
	getParametersFromConfigurationFile();

	/*
    void 
    readCSV(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, char separator = ';');

    cv::Ptr<cv::FaceRecognizer> 
    trainFaceRecognizer(const std::string& filename, char separator = ';'); 
	 */


private:

	float FRAME_SCALE;

	int RECOGNITION_FACE_WIDTH;

	int RECOGNITION_FACE_HEIGHT;

	int FACE_DETECTION_AVG_WINDOW_SIZE;

	float RECOGNITION_CONFIDENCE_THRESHOLD;

	float FACE_DETECTION_THRESHOLD;

	float NO_FACE_DETECTION_THRESHOLD;

	std::string FACE_DETECTION_CLASSIFIER_FILE;

	std::string FACE_RECOGNITION_TRAINING_SET_FILE;

	std::string NO_VISION_FILE_PATH;


	float frameRate;

	int scaledWidth, scaledHeight;

	float avgNumberOfFacesDetected;

	std::list<int> faceDetectionWindow;

	RoboyTimer noFaceTimer;

	RoboyTimer frameRateTimer;

	std::vector< cv::Rect_<int> > faces;


	int deviceID;

	RoboyCapture* capture;

	std::string device;

	// the guiFrame should be moved to RoboyGUI
	cv::Mat originalFrame, scaledFrame, grayFrame, guiFrame;

	cv::CascadeClassifier faceClassifier;

	cv::Ptr<cv::FaceRecognizer> recognitionModel;

	std::vector<int> faceRecognitionPredictions;

	std::vector<double> faceRecognitionConfidences;

	std::vector<RoboyPerson*> persons;

  int faceX, faceY, faceW, faceH;

  bool faceExists;

public:

	bool useVision;

	bool useFaceRecognition;

	const static int DEFAULT = 0;

	const static int FACE_DETECTED = 1;

	const static int NO_FACE_FOR_A_WHILE = 2;

	const static int FACE_RECOGNIZED = 3;

	const static int FACE_NOT_RECOGNIZED = 4;

};    

}

#endif  // _ROBOY_VISION_H_
