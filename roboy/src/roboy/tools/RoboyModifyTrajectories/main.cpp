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
#include "main.h"

int main(int argc, const char *argv[]) {
	Robot *canBus = new Robot("../../../../../roboy/", false);
	int i, numWayPoints;
	std::stringstream ss, ss2, folderPathStream;
	double *timeBetweenWaypoints;
	CubicSplineResult tmp;
	vector<CubicSplineResult> consolidated;
	
	if(argc < 3) {
		std::cerr << "usage: " << argv[0] << " <folder> <time Pt1-Pt2> <time Pt2-Pt3> ..."  << std::endl; 
		std::cerr << "Example for 4 waypoints:"  << std::endl;
		std::cerr << argv[0] << " GivingHand 2 2 2"  << std::endl;
		std::cerr << "Note that the time is in seconds and positive integers."  << std::endl;
		return 0;
	}
	
	folderPathStream.str("");	
	folderPathStream << canBus->pathToTrajectories.str().c_str() << argv[1] << "/";
	
	std::cerr << "Relative Folder Path: " << folderPathStream.str() << std::endl;
	std::cerr << "Time Array: [";
		
	timeBetweenWaypoints = (double *) malloc((argc-2)*sizeof(double));
	for(i = 2; i < argc; i++) {
		timeBetweenWaypoints[i-2] = atof(argv[i]);
		std::cerr << timeBetweenWaypoints[i-2];
		if(timeBetweenWaypoints[i-2] <= 0) {
			std::cerr << std::endl << "Error at Time Argument: " << (i-1) << std::endl;
			return 0;
		}
		if(i != argc - 1) std::cerr << " ";
	}
	std::cerr << "]" << std::endl;

	numWayPoints = argc - 1;
	consolidated.clear();
	
	for(i = 1; i <= TOTAL_MOTORS_IN_ROBOT; i++) {
		ss.str("");	
		ss2.str("");
		
		ss << folderPathStream.str().c_str();
		ss2 << folderPathStream.str().c_str();
		
		if(i < 10) {
			// Out.txt for output of Matlab
			ss << "Motor0" << i << "Waypoints.txt";
			ss2 << "Motor0" << i << "Out.txt";
		} else {
			ss << "Motor" << i << "Waypoints.txt";
			ss2 << "Motor" << i << "Out.txt";
		}
		tmp = canBus->allMotors[i].cubicSpline(ss.str().c_str(), ss2.str().c_str(), timeBetweenWaypoints, numWayPoints);
		if(tmp.success == false) {
			TRACE_RED_BOLD("Error generating trajectory file for Motor %d.", i); TRACE("\n");
		} else {
			tmp.motorID = i;
			consolidated.push_back(tmp);
		}
	}
	
	TRACE_PURP_BOLD("ID\tMax Vel\tMax Acc");TRACE("\n");
	for(i = 0; i < consolidated.size(); i++) {
		TRACE_PURP_BOLD("%d\t%d\t%d", consolidated[i].motorID, consolidated[i].maxVelocity, consolidated[i].maxAcceleration);TRACE("\n");
	
	}
	
	free(timeBetweenWaypoints);
	
	return 0;
}

