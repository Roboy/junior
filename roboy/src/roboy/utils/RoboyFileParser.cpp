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


#include "RoboyFileParser.h"

RoboyFileParser* RoboyFileParser::parser; 

RoboyFileParser::RoboyFileParser(std::string filename)
{
	std::cout << "Constructor" << std::endl; // FIXME

	std::ifstream f(filename.c_str());
	if(!f.is_open())
	{
		error("Failed to open configuration file: ", filename);
		return;
	}

	std::string line;
	std::map<std::string, std::string> *cSection = NULL;

	while(!f.eof())
	{
		std::getline(f, line);
		line.erase(line.begin(), std::find_if(line.begin(), line.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));

		if(line.empty() || line[0] == '#') continue;

		if(line[0] == '[')
		{
			if(line[line.size()-1] != ']')
			{
				error("syntax error at ", line);
				return;
			}

			cSection = new std::map<std::string, std::string>();
			sections.insert(std::pair<std::string, std::map<std::string,std::string> *>(line.substr(1, line.size()-2), cSection));

		} else {

			if(!cSection)
			{
				error("File does not start with section, but ", line);
				return;
			}
			else
			{
				size_t position = line.find(" =");
				if(position == std::string::npos)
				{
					error("syntax error at ", line);
					return;
				}

				cSection->insert(std::pair<std::string, std::string>(line.substr(0, position), line.substr(position+2, line.size())));
			}
		}
	}

	parser = this;

}


//template <class T> T 
//RoboyFileParser::get(std::string section, std::string key, std::string def)
//{
//	std::stringstream buffer;
//	T ret;
//
//	if(!sections.count(section) || !sections[section]->count(key))
//	{
//		buffer << def;
//		// FIXME: Maybe put a warning here
//	} else {
//		buffer << sections[section]->at(key);
//	}
//
//	buffer >> ret;
//	return ret;
//}



void 
RoboyFileParser::error(std::string msg, std::string arg)
{
	std::cerr << ERROR_PREFIX << msg << arg << std::endl;
}

//	config(config const&);
//	void operator=(config const&);

