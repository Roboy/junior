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
#include <unistd.h>

#include "RoboyMediaPlayerInteraction.h"
#include "RoboyFileParser.h"

using namespace std;

int main(int argc, const char *argv[]) {
{
    if (argc != 2)
    {
        std::cerr << "[Error] configuration file required. " << std::endl;
        std::cerr << "Example: "<< argv[0] << " ../../../../../roboy/database/configuration/RoboyConfigurationFile.conf" << std::endl;
        return 0;
    }

    RoboyFileParser::parser = new RoboyFileParser(argv[1]);

	RoboyMediaPlayerInteraction mediaPlayer;

    std::string video_db = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH")+"/Videos/";
    Roboy::RoboyVideoPlaylist speak;
    speak.addVideo(video_db+"10_NeutralFaceToSpeaking_.mp4", false);
    speak.addVideo(video_db+"11_Speaking_.mp4", true);
    speak.addVideo(video_db+"12_SpeakingToNeutralFace_.mp4", false);

    mediaPlayer.init(video_db+"04_NeutralFace_.mp4");
    sleep(60);

    mediaPlayer.newPlaylist(speak);
    mediaPlayer.newPlaylist(speak);
    sleep(10);
    mediaPlayer.breakLoop();
    
    sleep(300);
    std::cout << "Done." << std::endl;
    mediaPlayer.shutdown();
}
	
  return 0;
}



