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


#include "RoboyStarmind.h"

namespace Roboy
{

RoboyStarmind::RoboyStarmind(std::vector<RoboyBodyActivity*> bodyActivities, RoboyStarmindConnector* pStarmind, Robot *canBus_passed) : RoboyBehaviour(bodyActivities)
{
	starmind = pStarmind;
	isBehaviourActive = false;
	canBus = canBus_passed;
}

RoboyStarmind::~RoboyStarmind()
{
}

void
RoboyStarmind::init()
{
	std::cerr << " STARMIND INIT " << std::endl;
	isBehaviourActive = true;
}


void 
RoboyStarmind::execute()
{
	std::cerr << " STARMIND EXECUTE " << std::endl;

	while(isBehaviourActive) {
		std::string text = this->starmind->getNextText();
		if(!text.empty()){
			if(text.length() > 6 && text.substr(0,6).compare("voice_") == 0){
				unsigned pos = text.find("#");
				if(pos!=std::string::npos){
					std::string voice = text.substr(0, pos);
					text = text.substr(pos+1, text.length()-pos-1);
					textToSpeech->setVoice(voice);
				}
			}
			std::cout << "Starmind Text: " << text << std::endl;
			face->backToNormalFast();
			textToSpeech->setText(text);
			textToSpeech->start();
			face->init("speak");
			usleep(text.length()*80000);
			face->backToNormalFast();
		}
		std::string facial = this->starmind->getNextFacial();
		if(!facial.empty()){

			std::cout << "Starmind Facial: " << facial << std::endl;

			if(facial.compare("normal") == 0){
				face->backToNormalFast();
			}else{
				face->init(facial);
			}
		}
		int *head = this->starmind->getNextHead();
		if(head != 0){

			std::cout << "Starmind Head: roll: " << head[0] << " pitch: " << head[1] << " yaw: " << head[2] << std::endl;

			bodyMovement->moveHead(head[0], head[1], head[2], true);
		}
		std::string body = this->starmind->getNextBody();
		if(!body.empty()){

			std::cout << "Starmind Body: " << body << std::endl;

			bodyMovement->setFolderToPlay(body);
			bodyMovement->start();
		}
		usleep(10000);
	}

	isBehaviourFinished = true;
}

void
RoboyStarmind::terminate()
{
	std::cerr << " STARMIND TERMINATE " << std::endl;
	isBehaviourActive = false;
}


std::string
RoboyStarmind::getInfo()
{
	return "What?";
}

}
