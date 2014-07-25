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

#ifndef _ROBOY_ROBOYSTATE_H_
#define _ROBOY_ROBOYSTATE_H_

#include <vector>

#include "RoboyBehaviouralRepertoire.h"

namespace Roboy
{

  
  class RoboyState
  {

  public:

    RoboyState();
    
    virtual ~RoboyState();
    
    void
    addBehaviour(RoboyBehaviour* behaviour);
    
    RoboyBehaviour*
    selectBehaviour();
    
  protected:
      
    bool isStateActive;
        
    std::vector<RoboyBehaviour*> behaviours;

  public:

    const static int DEFAULT = 0;

    const static int VISION = 1;

    const static int RESERVE_STATE = 2;
    
    const static int SHOW_EXPRESSIONS_STATE = 3;

    const static int MOUTH_FOLLOWNIG_SOUND_STATE = 4;

    const static int STARMIND = 5;

		const static int GAZING_STATE = 6;

		const static int GIVE_CLOSE_SHAKE_OPEN_DRAWBACK_STATE = 7;

    const static int INTRODUCTION_STATE = 8;

    const static int FOLLOWING_FACE_STATE = 9;

		const static int COME_TO_ME_STATE = 10;

		const static int SHY_STATE = 11;

		const static int HUG_ROLF_STATE = 12;

//    const static int FACE_LEFT = 10;

//    const static int NO_FACE_FOR_A_WHILE = 11;

  };

}

#endif  // _ROBOY_ROBOYSTATE_H_
