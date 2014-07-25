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
#include "RoboySpeakingRandomBehaviour.h"
#include "time.h"

namespace Roboy
{
  
	RoboySpeakingRandomBehaviour::RoboySpeakingRandomBehaviour(std::vector<RoboyBodyActivity*> bodyActivities, std::vector<std::string> sentences) : 
        RoboyBehaviour(bodyActivities), sentences(sentences)
	{
		
	}

    RoboySpeakingRandomBehaviour::~RoboySpeakingRandomBehaviour() 
    {
    
    }

    void
    RoboySpeakingRandomBehaviour::init()
    {
      std::cout << " SPEAKINGRANDOM INIT " << std::endl;            
    }

    void 
    RoboySpeakingRandomBehaviour::execute()
    {

      std::cout << " SPEAKINGRANDOM EXECUTE " << std::endl; 

			double last_time_LoudSound;           

      RoboyTextToSpeech *festival = ((RoboyTextToSpeech*)bodyActivities[RoboyBodyActivity::TEXT_TO_SPEECH]);
//      RoboyFacialExpression *face = ((RoboyFacialExpression*)bodyActivities[RoboyBodyActivity::FACIAL_EXPRESSION]);
      unsigned int i=0;
      while(1)
      {

/* // safe version, but with delays
				if(listenToSound->isSoundLoud()){
        // SPEAK
        //festival->setText(sentences[i++ % sentences.size()]);
        std::cout << " start speaking face " << std::endl;//face->init("speak");
        std::cout << " start speaking " << std::endl;//festival->start();

				last_time_LoudSound = (double) (clock() / CLOCKS_PER_SEC);
				while(1){
					
					if(listenToSound->isSoundLoud()) last_time_LoudSound = (double) (clock() / CLOCKS_PER_SEC);
					
					if((double) (clock() / CLOCKS_PER_SEC) - last_time_LoudSound > 0.1) break;
					
					usleep(1000); // wait
				} // end while(1)
        	
        std::cout << " stop speaking face\n"; //face->backToNormalFast();

				usleep(5000);
        // usleep(5000000); // repeat the whole thing
	
				} // end if(listenToSound->isSoundLoud())
*/

				if(listenToSound->isSoundLoud()){
		      // SPEAK
		      //festival->setText(sentences[i++ % sentences.size()]);
          //festival->start();
          //std::cout << " start speaking " << std::endl;
		      std::cout << " start speaking face " << std::endl;
          face->init("speak");

					while(listenToSound->isSoundLoud()){
					usleep(50);
					}
					
					std::cout << " stop speaking face\n";
          face->backToNormalFast();
					
				} // end if(listenToSound->isSoundLoud())

        boost::this_thread::interruption_point(); 
      }

		isBehaviourFinished = true; // never should get here

    }

    void
    RoboySpeakingRandomBehaviour::terminate()
    {
      std::cout << " SPEAKINGRANDOM TERMINATE " << std::endl;      
    }


    std::string
    RoboySpeakingRandomBehaviour::getInfo()
    {
      return "Speaking Random";
    }
}
