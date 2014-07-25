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

#include <fstream>
#include <sstream>
#include <iostream>

#include "RoboyPerson.h"

namespace Roboy{

RoboyPerson::RoboyPerson(int index, std::string name, std::string audioFile)
{
	this->index = index;
	this->name = name;
	this->audioFile = audioFile;
}

RoboyPerson::~RoboyPerson()
{

}

std::vector<RoboyPerson*>
RoboyPerson::readPersonDatabase(std::string databaseFile)
{

	std::cout << "Reading Roboy person database... " << std::endl;

	std::vector<RoboyPerson*> db;

	std::ifstream infile(databaseFile.c_str());

	if (!infile)
	{
		std::cerr << "Roboy people database not found. File provided: " << databaseFile << std::endl;
		return db;
	}

	int index;
	std::string line, name, audioFile;
	int lineCount = 1;

	while (getline(infile, line))
	{
		if(infile >> index >> name >> audioFile)
		{
			db.push_back(new RoboyPerson(index, name, audioFile));
		} else {
			std::cerr << "Line " << lineCount << ": not well formatted " << std::endl;
			infile.clear();
		}

		lineCount++;
	}

	std::cout << "Reading Roboy person database... DONE " << std::endl;

	return db;
}


void
RoboyPerson::printPersonList(std::vector<RoboyPerson*> persons)
{
	for(int i=0; i<persons.size(); i++)
	{
		persons[i]->printData();
	}
}


void
RoboyPerson::printData()
{
	std::cout << index << " : " << name << " : " << audioFile << std::endl;
}


int
RoboyPerson::getIndex()
{
	return index;
}

void
RoboyPerson::setIndex(int index){
	this->index = index;
}

std::string
RoboyPerson::getName(){
	return name;
}

void
RoboyPerson::setName(std::string name)
{
	this->name = name;
}

std::string
RoboyPerson::getAudioFile()
{
	return audioFile;
}

void
RoboyPerson::setAudioFile(std::string audioFile)
{
	this->audioFile = audioFile;
}


bool
RoboyPerson::hasBeenRecognized()
{
	return recognized;
}

void
RoboyPerson::setRecognized()
{
	recognized = true;
}

void
RoboyPerson::setNotRecognized()
{
	recognized = false;
}


void
RoboyPerson::updateRecognitionRecord(bool recognized, int confidenceValue)
{
	if(recognized)
	{
		recognitionCount++;
		this->avgConfidenceValue = 0.8 * avgConfidenceValue + 0.2 * confidenceValue;
		return;
	}

	resetRecognitionRecord();

}


void
RoboyPerson::resetRecognitionRecord()
{
	recognitionCount = 0;
	this->avgConfidenceValue = 0;
}

int
RoboyPerson::getRecognitionCount()
{
	return recognitionCount;
}


float
RoboyPerson::getRecognitionAverageConfidence()
{
	return avgConfidenceValue;
}


}
