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

#ifndef _ROBOY_ROBOYBODYMOVEMENT_H_
#define _ROBOY_ROBOYBODYMOVEMENT_H_


#include "RoboyBodyActivity.h"

#pragma once

#include "Robot.h"
#include "genFunc.h"
#include <iostream>
#include <boost/thread.hpp>

#define MOTOR_POSITION_ERROR_ALLOWANCE 2000
#define POSITION_CHECKING_TIME_PERIOD_NS 250000000

#define DIGITAL_INPUT_CHECKING_TIME_PERIOD_US 100000

#define DEFAULT_NECK_VELOCITY 3000
#define DEFAULT_NECK_ACCELERATION 4000
#define DEFAULT_LEG_SWING_VELOCITY 2000
#define DEFAULT_LEG_SWING_ACCELERATION 3000

#define NO_LEG_SWING 0
#define DEFAULT_LEG_PHASE_SHIFT 90
#define LEG_SWING_CHECK_TIME_PERIOD_US 300000

#define LEFT_KNEE_REST_POSITION 336258
#define RIGHT_KNEE_REST_POSITION 165146

#define RIGHT_HIP_FRONT_REST_POSITION 432197
#define RIGHT_HIP_REAR_REST_POSITION 320091
#define LEFT_HIP_FRONT_REST_POSITION 556631
#define LEFT_HIP_REAR_REST_POSITION 369123


#define MAX_PHASE_SHIFT 360
#define MIN_PHASE_SHIFT 0

// BodyMovement should be applicable for a any Folder containing Motor Position Commands for the whole body (except head, under Arm and Hand?)


namespace Roboy
{

  class RoboyBodyMovement: public RoboyBodyActivity
  {
  
  public:

  RoboyBodyMovement(Robot *canBus_given);
    
	virtual ~RoboyBodyMovement();

  bool isLegSwinging;

    void 
		init(std::string filetoplay);
    
    void 
    execute();

    void
    terminate();
    

    virtual std::string
    getInfo();

		struct trajectoryFileInfo {
		int motorID, numWayPoints, totalTimeMS;
		};

		void
		playTrajectory();

		bool
		moveHand(bool isLeftHand, bool isClose, bool blocking = false);

		bool
		moveHand(bool isLeftHand, int value, bool blocking = false);

		bool
		moveForeArm(bool isLeftArm, int value, int maxVelocity, int maxAcceleration, bool blocking = false);

		bool
		isHandTouched(bool isLeftHand);

    // Blocking function, until sensor returns desired value
    void
		isHandTouched(bool isLeftHand, bool isTouched);

		bool
		moveHead(int roll, int pitch, int yaw, bool blocking = false);

    bool
		moveHead(neckMotionCommand nMC, bool blocking = false);

    void
		setLegSwingBehavior(int timePeriodUS, int velocity, int acceleration, int magnitude, int phaseShift, int swingSets);

		void
		setFolderToPlay(std::string folderName_passed);

		int *motorID;

		Robot *canBus;

		std::string folderName;

  private:
    // Phase shift in terms of degrees (90 degrees = left leg starts to swing at the peak position of right leg)
    // Left leg starts swinging when right leg returns if set to 180 and for values greater than 180, there will
    // be some time lag between the leg returning anf the left leg swinging
    // Phase shift is between 0 - 360. Set to 0 for negative values and 360 for values greater than 360.

    // Leg swing magnitude in terms of encoder counts
    // if leg swing period is less than the total time period required for both legs to complete a swing, the 
    // actual period will just be that the right leg will start swinging once the left leg completes
    // Legs do not swing if time period set to 0
    int legSwingTimePeriodUS, legSwingVelocity, legSwingAcceleration, legSwingMagnitude, legSwingPhaseShift, legSwingSets;
    void legSwingExecuteThread();
    // To prevent variables from changing during computation and activation
    void performLegSwing(int lSTP, int lSV, int lSA, int lSM, int lSPS, int lSS);
    
  protected:
	  boost::thread* legSwingThread;
    
  };

}

#endif  // _ROBOY_ROBOYBODYMOVEMENT_H_
