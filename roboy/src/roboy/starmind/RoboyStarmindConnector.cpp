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

#include "RoboyStarmindConnector.h"
#include "socket/ServerSocket.h"
#include "socket/SocketException.h"
#include "RoboyFileParser.h"

namespace Roboy
{

RoboyStarmindConnector::RoboyStarmindConnector(RoboyVision* vision)
{
	this->isStarmindActive = false;
	this->vision = vision;
	this->port = 30000;
}


RoboyStarmindConnector::~RoboyStarmindConnector()
{

}


void
RoboyStarmindConnector::start()
{
	std::string p = RoboyFileParser::parser->get<std::string>("starmind", "PORT");
	if(!p.empty()){
		int port = atoi(p.c_str());
		if(port > 1024){
			this->port = port;
		}
	}
	runConnector = true;
	starmindConnectorThread = new boost::thread( boost::bind(&RoboyStarmindConnector::executeConnector, this) );
	isConnectorActive = true;
}

void
RoboyStarmindConnector::stop()
{
	if(!isConnectorActive){
		return;
	}
	runConnector = false;

	starmindConnectorThread->interrupt();
	isConnectorActive = false;
}

void
RoboyStarmindConnector::executeConnector()
{
	try
	{
		// Create the socket
		ServerSocket server ( this->port );
		std::cout << "StarmindConnector started" << std::endl;
		while(runConnector){
			ServerSocket new_sock;
			server.accept ( new_sock );
			std::cout << "StarmindConnector: accepted new connection" << std::endl;
			try
			{
				while ( true )
				{
					//std::cout << "while" << std::endl;
					isStarmindActive = true;
					std::string request;
					new_sock >> request;
					//std::cout << request << std::endl;
					if(request.length() >= 3 && request.compare("pic") == 0){
						//std::cout << "do pic" << std::endl;
						std::vector<uchar> buff;

						//std::cout << "do pic1" << std::endl;

						cv::Mat frame = this->vision->getGUIFrame();

						//std::cout << "do pic2" << std::endl;

						std::vector<int> param = std::vector<int>(2);

						//std::cout << "do pic3" << std::endl;

						param[0]=CV_IMWRITE_JPEG_QUALITY;
						param[1]=95;//default(95) 0-100
						imencode(".jpg", frame, buff, param);

						//std::cout << "do pic4" << std::endl;

						new_sock.send_bytes(buff.data(), buff.size());

						//std::cout << "do pic5" << std::endl;

					}
					else if(request.length() >= 4 && request.substr(0,4).compare("text") == 0){

						std::string text = request.substr(5,request.length()-5);

						std::cout << "Text: " << text << std::endl;

						textes.push(text);

					}
					else if(request.length() >= 6 && request.compare("voices") == 0 ){
						std::string voices = execCmd("ls /usr/share/festival/voices/english/");
						new_sock << voices;
					}
					else if(request.length() >= 5 && request.substr(0,4).compare("mode") == 0){
						std::string k = request.substr(5, request.length()-5);

						std::cout << "Mode: " << k << std::endl;

						modes.push(k[0]);
					}
					else if(request.length() >= 5 && request.substr(0,4).compare("head") == 0){
						std::string head = request.substr(5, request.length()-5);
						int* iHead = new int[3];
						unsigned lastpos = 0;
						for(int i = 0; i < 3; i++){
							int end = head.find(":", lastpos);
							iHead[i] = atoi(head.substr(lastpos,end).c_str());
							lastpos = end+1;
						}

						std::cout << "Head: roll :" << iHead[0] << " pitch: " << iHead[1] << " yaw: " << iHead[2] << std::endl;

						heads.push(iHead);
					}
					else if(request.length() >= 7 && request.substr(0,6).compare("facial") == 0){
						std::string facial = request.substr(7,request.length()-7);

						std::cout << "Facial: " << facial << std::endl;

						facials.push(facial);
					}
					else if(request.length() >= 5 && request.substr(0,4).compare("body") == 0){
						std::string body = request.substr(5, request.length()-5);

						std::cout << "Body: " << body << std::endl;

						bodys.push(body);
					}
					else if(request.length() >= 4 && (request.compare("exit") == 0 || request.compare("quit"))){

					}

				}
			}
			catch ( SocketException& ) {}
			isStarmindActive = false;
		}
	}
	catch ( SocketException& e )
	{
		std::cout << "RoboyStarmindConnector: Exception was caught:" << e.description() << "\n";
	}
}

bool
RoboyStarmindConnector::isStarmindConnected()
{
	return this->isStarmindActive;
}

std::string
RoboyStarmindConnector::getNextText(){
	if(this->textes.empty()){
		return "";
	}
	std::string str = this->textes.front();
	this->textes.pop();
	return str;
}

char
RoboyStarmindConnector::getNextMode(){
	if(this->modes.empty()){
		return ' ';
	}
	char key = this->modes.front();
	this->modes.pop();
	return key;
}

int*
RoboyStarmindConnector::getNextHead(){
	if(this->heads.empty()){
		return 0;
	}
	int* head = this->heads.front();
	this->heads.pop();
	return head;
}

std::string
RoboyStarmindConnector::getNextFacial(){
	if(this->facials.empty()){
		return "";
	}
	std::string facial = this->facials.front();
	this->facials.pop();
	return facial;
}

std::string
RoboyStarmindConnector::getNextBody(){
	if(this->bodys.empty()){
		return "";
	}
	std::string body = this->bodys.front();
	this->bodys.pop();
	return body;
}

std::string
RoboyStarmindConnector::execCmd(char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		result += buffer;
    }
    pclose(pipe);
    return result;
}

/*
std::queue<std::string*>
RoboyStarmindConnector::getTextQueue()
{
	std::cout << "getText" << std::endl;
	return this->textes;
}*/


}





