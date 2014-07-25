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



#include "RoboyFollowingFaceBehaviour.h"

namespace Roboy
{

RoboyFollowingFaceBehaviour::RoboyFollowingFaceBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, Robot *canBus_passed, RoboyVision* vision_passed): RoboyBehaviour(bodyActivities)
{
	std::cout << "Roboy FollowingFace Behaviour CONSTRUCTOR" << std::endl;
	currentStatus.str("FollowingFace Behaviour Constructed");
	canBus = canBus_passed;
	vision = vision_passed;
}

RoboyFollowingFaceBehaviour::~RoboyFollowingFaceBehaviour()
{

}

void
RoboyFollowingFaceBehaviour::init()
{
	// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
	// Therefore in the init() and execute() we have to say which bodyActivities should be played.
	std::cout << "RoboyFollowingFaceBehaviour INITIALIZATION." << std::endl;
	currentStatus.str("RoboyFollowingFaceBehaviour INITIALIZATION");

}

void
RoboyFollowingFaceBehaviour::execute()
{
	// A Behaviour does not contain its body activities, it only contains a Pointer to the BodyActivity Vector.
	// Therefore in the init() and execute() we have to say which bodyActivities should be played.
	std::cout << "RoboyFollowingFaceBehaviour EXECUTING" << std::endl;
	currentStatus.str("RoboyFollowingFaceBehaviour EXECUTING");
	neckMotionCommand nMC;
	int currentYaw, currentPitch, errorYaw, errorPitch, previousError;

	int pitch_detectedFace = 0; // pitch angles 0 (straight) to 15 (down)
	int yaw_detectedFace = 0; // yaw angles -15 (right) to 15 (left)

	nMC.neckRollAngle = 0;
	nMC.neckPitchAngle = 0;
	nMC.neckYawAngle = 0;
	nMC.rollVelocity = 3000;
	nMC.pitchVelocity = 3000;
	nMC.yawVelocity = 3000;
	nMC.rollAcceleration = 4000;
	nMC.pitchAcceleration = 4000;
	nMC.yawAcceleration = 4000;
	//	bodyMovement->moveHand(false, 500000, true);
	//	bodyMovement->moveHand(false, 0, true);
	bodyMovement->moveHead(nMC, true);

	std::cout << "ROBOT finished head init" << std::endl;
	previousError = 0;
  face->init("neutral");

	while (isActive()){ // actually isBehaviourActive is managed by start() and stop(), so it should always be true when running the thread and is automatically set to false after stopped (= interrupted). Still, if anything goes wrong there, we don't want to loop in case isActive is false.
    //vision->captureFrame();
		vision->detectFaces();


		int faceX, faceY, faceW, faceH;     

    bool faceExists = vision->getFaceData(faceX, faceY, faceW, faceH);
    if (faceExists)
		{
			// UPDATE FACE ANGLE HERE
			cv::Point faceLocation = getFaceLocation(faceX, faceY, faceW, faceH);

			std::cout << "face detected at: " << std::endl;
			std::cout << "face detected at: " << faceLocation.x << "," << faceLocation.y << std::endl;

			currentYaw = bodyMovement->canBus->getNeckYaw();
			currentPitch = bodyMovement->canBus->getNeckPitch();
  
      errorYaw = faceLocation.x/YAW_GAIN;
      errorPitch = faceLocation.y/PITCH_GAIN;

      if(faceLocation.x < DEAD_ZONE_PIXELS && faceLocation.x > -DEAD_ZONE_PIXELS) {
        errorYaw = 0;
      }

      if(faceLocation.y < DEAD_ZONE_PIXELS && faceLocation.y > -DEAD_ZONE_PIXELS) {
        errorPitch = 0;
      }

			nMC.neckPitchAngle = bound(currentPitch - errorPitch, 12, 0);
			nMC.neckYawAngle = bound(currentYaw - errorYaw, 12, -12);

      if(errorYaw == 0) {
        if(previousError != 0) face->init("neutral");
      } else {
        if(errorYaw*previousError <= 0) {
          if(errorYaw > 0) {
            face->init("eyesRight");
            std::cout << "Playing eyes right" << std::endl;
          } else {
            face->init("eyesLeft");
            std::cout << "Playing eyes left" << std::endl;
          }
        }
      }
  	  bodyMovement->moveHead(nMC, true);
      previousError = errorYaw;
		} else {
      if(previousError != 0) face->init("neutral");
      previousError = 0;
    }





		//			bodyMovement->moveHead(0, pitch_detectedFace, yaw_detectedFace, false); // (roll, pitch, yaw, blocking)

		// maybe add a surprise face when new face gets detected....
		//face->init("surprise");
		//textToSpeech->setText("oo");
		//textToSpeech->start();

		try{

			boost::this_thread::interruption_point();

		} catch(boost::thread_interrupted const&) {
			std::cout << "RoboyFollowingFaceBehaviour INTERRUPTED" << std::endl;
			break;
		}

	} // end while


	isBehaviourFinished = true;
}

void
RoboyFollowingFaceBehaviour::terminate()
{

	// LATER: All the running bodyActivities have to be stopped! Also headMovement, facialExpression, etc!

	std::cout << "RoboyFollowingFaceBehaviour STOP " << std::endl;
	bodyActivities[RoboyBodyActivity::BODY_MOVEMENT]->stop();
	// setting back the isFinished and isActive Values happens in the stop() function.


}


std::string
RoboyFollowingFaceBehaviour::getInfo()
{
	return "RoboyFollowingFaceBehaviour";
}

cv::Point
RoboyFollowingFaceBehaviour::getFaceLocation(int faceX, int faceY, int faceW, int faceH)
{
  cv::Rect_<int> face(faceX, faceY, faceW, faceH);
  cv::Rect_<int> faceScalled = vision->convertFromScaledFrameToOriginalFrameCoordinates(face);

	cv::Mat frame = vision->getGUIFrame();

	int x0 = faceScalled.x - frame.size().width/2 + faceScalled.width/2;
	int y0 = faceScalled.y - frame.size().height/2 + faceScalled.height/2;

	return cv::Point(x0, y0);



	//	std::vector< cv::Rect_<int> > faces = vision->getFaces();

//	if (faces.size() == 0)
//		return 0;

//	std::cout << " vision face convert " << std::endl;
	//    	std::cerr << " vision face converted " << std::endl;

//	return getFaceCentre(face);
}


/*
cv::Point*
RoboyFollowingFaceBehaviour::getFaceCentre(cv::Rect_<int> face)
{
	cv::Mat frame = vision->getGUIFrame();
	std::cerr << " get gui frame " << std::endl;

	int x0 = face.x - frame.size().width/2 + face.width/2;
	int y0 = face.y - frame.size().height/2 + face.height/2;

	std::cerr << " calculations done " << std::endl;
	return new cv::Point(x0, y0);
}
*/

}
