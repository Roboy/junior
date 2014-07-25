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
#include <string>

#include "RoboyFileParser.h"

using namespace std;

int main()
{
	RoboyFileParser parser("/home/hgmarques/Documents/roboymediatest/test.conf");
	
	cout << "foo as string: " << "\033[1m" << parser.get<string>("main", "foo") << "\033[0m" << endl;
    cout << "foo as int: " << "\033[1m" << parser.get<int>("main", "foo") << "\033[0m" << endl;
    cout << "foo as float: " << "\033[1m" << parser.get<float>("main", "foo") << "\033[0m" << endl;
    cout << "bar as string: " << "\033[1m" << parser.get<string>("main", "bar") << "\033[0m" << endl;
    cout << "bar as int: " << "\033[1m" << parser.get<int>("main", "bar") << "\033[0m" << endl;
    cout << "bar as float: " << "\033[1m" << parser.get<float>("main", "bar") << "\033[0m" << endl;
    cout << "box as string: " << "\033[1m" << parser.get<string>("main", "box") << "\033[0m" << endl;
    cout << "box as int: " << "\033[1m" << parser.get<int>("main", "box") << "\033[0m" << endl;
    cout << "box as float: " << "\033[1m" << parser.get<float>("main", "box") << "\033[0m" << endl;
    cout << "extended: " << "\033[1m" << parser.get<string>("extended", "foo") << "\033[0m" << endl;
    cout << "robust: " << "\033[1m" << parser.get<string>("robust examples", "foo") << "\033[0m" << endl;
    cout << "unknown w/o default: " << "\033[1m" << parser.get<string>("main", "wut") << "\033[0m" << endl;
    cout << "unknown w/ default: " << "\033[1m" << parser.get<string>("main", "wut", "default") << "\033[0m" << endl;
}


