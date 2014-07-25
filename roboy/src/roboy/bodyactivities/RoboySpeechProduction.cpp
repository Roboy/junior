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

#include "RoboySpeechProduction.h"

namespace Roboy
{

RoboySpeechProduction::~RoboySpeechProduction() 
{

}

void
RoboySpeechProduction::init(std::string filetoplay)
{
	this->filetoplay = filetoplay;
	alutInit (NULL, NULL);
	sndBuffer = alutCreateBufferFromFile (filetoplay.c_str());

	alGenSources (1, &sndSource);
	alSourcei (sndSource, AL_BUFFER, sndBuffer);

	// TODO: should get .wav playing time
	std::cerr << " SPEECH PRODUCTION INIT: " << filetoplay << std::endl;            
}

void 
RoboySpeechProduction::execute()
{

	std::cerr << " SPEECH PRODUCTION EXECUTED " << std::endl;            

	alSourcePlay (sndSource);

	usleep(5000000);

	isBodyActivityFinished = true;
}

void
RoboySpeechProduction::terminate()
{
	std::cerr << " SPEECH PRODUCTION TERMINATE " << std::endl;
	alutExit ();
	filetoplay = "";
}


std::string
RoboySpeechProduction::getInfo()
{
	std::string info = "Speech production: ";
	info.append(filetoplay);
	return info;      
}

}
