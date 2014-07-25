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

#include "festival.h"
#include "RoboyTextToSpeech.h"
#include "RoboyFileParser.h"

namespace Roboy
{

RoboyTextToSpeech::RoboyTextToSpeech() : RoboyBodyActivity()
{

}


RoboyTextToSpeech::~RoboyTextToSpeech()
{

}

void
RoboyTextToSpeech::init(std::string voice)
{
	std::string v = RoboyFileParser::parser->get<std::string>("text2speech", "VOICE");
	std::cout << "VOICE from configuration file: " << v << std::endl;
	if(!voice.empty()){
		v = voice;
	}
	if(v.empty()){
		v = "voice_kal_diphone";
	}
	this->voice = v;
}

void 
RoboyTextToSpeech::execute()
{

	std::cout << " TEXT TO SPEECH EXECUTE" << std::endl;

	int heapSize = 210000;  // default scheme heap size
	int loadInitFiles = 1; // we want the festival init files loaded

	festival_initialize(loadInitFiles,heapSize);

	while(1)
	{

		while(!isBodyActivityActive){
			usleep(30000);
		}

		std::string v = "(set! current-voice ("+this->voice+"))";
		festival_eval_command(v.c_str());

		v = "(Parameter.set 'Duration_Stretch 1.1')";
		//festival_eval_command(v.c_str());

		std::cout << "(SayText "+this->text+"), using voice: " << this->voice << std::endl;

		festival_say_text(this->text.c_str());

        std::cout << "Done talking." << std::endl;

		festival_wait_for_spooler();

		isBodyActivityFinished = true;
		isBodyActivityActive = false;

	}

}

void
RoboyTextToSpeech::terminate()
{
//	std::cerr << " TEXT TO SPEECH TERMINATE " << std::endl;
	text = "";
}

void
RoboyTextToSpeech::setText(std::string text)
{
	this->text = text;
}

void
RoboyTextToSpeech::setVoice(std::string voice)
{
	this->voice = voice;
}

std::string
RoboyTextToSpeech::getVoice()
{
	return this->voice;
}

std::string
RoboyTextToSpeech::getInfo()
{
	std::string info = "Text to speech: ";
	info.append(this->text);
	info.append(" ("+this->voice+")");
	return info;      
}

}
