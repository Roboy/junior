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


#ifndef _ROBOY_ROBOYVISIONINTERACTIONGUI_H_
#define _ROBOY_ROBOYVISIONINTERACTIONGUI_H_


#include "opencv2/opencv.hpp"
#include "RoboyVision.h"
#include "RoboyPerson.h"
#include "RoboyBehaviour.h"


namespace Roboy {

  class RoboyVisionInteractionGUI
  {

  public:


	RoboyVisionInteractionGUI(std::string name, RoboyVision* vision);
    
    virtual ~RoboyVisionInteractionGUI();
    
    char
    show(RoboyBehaviour* behaviour);
    
    void
    setPersonList( std::vector<RoboyPerson*> persons);
    
  private:

    std::string name;
    
    RoboyVision* vision;
    
    std::vector<RoboyPerson*> persons;
    
    bool showDebugInfo;
    
    bool showDemoInfo;
    
    bool showBehaviouralInfo;
    
    bool showVisionInfo;

    bool showHelp;

  public:

    static const char KEY_NONE = 255;

    static const char KEY_0 = 48;

    static const char KEY_1 = 49;

    static const char KEY_2 = 50;

    static const char KEY_3 = 51;

    static const char KEY_4 = 52;

    static const char KEY_5 = 53;

    static const char KEY_6 = 54;

    static const char KEY_7 = 55;


    static const char KEY_B = 98;
        
    static const char KEY_D = 100;

    static const char KEY_H = 104;

    static const char KEY_M = 109;

    static const char KEY_R = 114;
    
    static const char KEY_V = 118;

    static const char KEY_ESC = 27;

  };    

}

#endif  // _ROBOY_ROBOYGUI_H_
