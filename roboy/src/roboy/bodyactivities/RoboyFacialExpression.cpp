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
#include "RoboyFacialExpression.h"
#include "debug.h"

namespace Roboy
{

typedef std::pair<std::string,RoboyVideoPlaylist> map_item;
RoboyFacialExpression::RoboyFacialExpression() : RoboyBodyActivity()
{
    std::string video_db = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH")+"/Videos/";
    RoboyVideoPlaylist speak;
    speak.addVideo(video_db+"10_NeutralFaceToSpeaking_.mp4", false);
    speak.addVideo(video_db+"11_Speaking_.mp4", true);
    speak.addVideo(video_db+"12_SpeakingToNeutralFace_.mp4", false);
    videos.insert(map_item("speak", speak));

    //RoboyVideoPlaylist sleep;TRACE_RED_BOLD
    //sleep.addVideo(video_db+"100_GoToSleep_.mp4", false);
    //sleep.addVideo(video_db+"02_Sleeping_.mp4", false);
    //videos.insert(map_item("sleep", sleep));

    RoboyVideoPlaylist sleep;
    sleep.addVideo(video_db+"100_GoToSleep_.mp4", false);
    sleep.addVideo(video_db+"02_Sleeping_.mp4", true);
    sleep.addVideo(video_db+"03_WakeUp_.mp4", false);
    videos.insert(map_item("sleep", sleep));

    //RoboyVideoPlaylist wakeup;
    //sleep.addVideo(video_db+"02_Sleeping_.mp4", false);
    //sleep.addVideo(video_db+"03_WakeUp_.mp4", false);
    //videos.insert(map_item("wakeup", wakeup));

    RoboyVideoPlaylist smile;
    smile.addVideo(video_db+"05_NeutralFaceToSmiling_.mp4", false);
    smile.addVideo(video_db+"06_Smiling_.mp4", true);
    smile.addVideo(video_db+"08_SmilingToNeutralFace_.mp4", false);
    videos.insert(map_item("smile", smile));

    RoboyVideoPlaylist smileblink;
    smileblink.addVideo(video_db+"05_NeutralFaceToSmiling_.mp4", false);
    smileblink.addVideo(video_db+"07_SmilingAndBlinking_.mp4", false);
    smileblink.addVideo(video_db+"08_SmilingToNeutralFace_.mp4", false);
    videos.insert(map_item("smileblink", smileblink));

    RoboyVideoPlaylist shy;
    shy.addVideo(video_db+"09_NeutralToShynessToNeutral_.mp4", false);
    videos.insert(map_item("shy", shy));

    RoboyVideoPlaylist surprise;
    surprise.addVideo(video_db+"13_Surprise_.mp4", false);
    videos.insert(map_item("surprise", surprise));

    RoboyVideoPlaylist kiss;
    kiss.addVideo(video_db+"16_Kiss_.mp4", false);
    videos.insert(map_item("kiss", kiss));
    
    RoboyVideoPlaylist neutral;
    neutral.addVideo(video_db+"04_NeutralFace_.mp4", false);
    videos.insert(map_item("neutral", neutral));
    
    RoboyVideoPlaylist angry;
    angry.addVideo(video_db+"40_NeutralFaceToAngry_.mp4", false);
    angry.addVideo(video_db+"41_Angry_.mp4", true);
    angry.addVideo(video_db+"42_AngryToNeutralFace_.mp4", false);
    videos.insert(map_item("angry", angry));

    RoboyVideoPlaylist sweat;
    sweat.addVideo(video_db+"18_Sweat_.mp4", true);
    videos.insert(map_item("sweat", sweat));

    RoboyVideoPlaylist eyesLeft;
    eyesLeft.addVideo(video_db+"20_NeutralFaceToEyesLeft_.mp4", false);
    eyesLeft.addVideo(video_db+"21_EyesLeft_.mp4", true);
    eyesLeft.addVideo(video_db+"22_EyesLeftToNeutralFace_.mp4", false);
    videos.insert(map_item("eyesLeft", eyesLeft));

    RoboyVideoPlaylist eyesDown;
    eyesLeft.addVideo(video_db+"26_NeutralFaceToEyesDown_.mp4", false);
    eyesLeft.addVideo(video_db+"27_EyesDown_.mp4", true);
    eyesLeft.addVideo(video_db+"28_EyesDownToNeutralFace_.mp4", false);
    videos.insert(map_item("eyesDown", eyesDown));

    RoboyVideoPlaylist eyesRight;
    eyesRight.addVideo(video_db+"23_NeutralFaceToEyesRight_.mp4", false);
    eyesRight.addVideo(video_db+"24_EyesRight_.mp4", true);
    eyesRight.addVideo(video_db+"25_EyesRightToNeutralFace_.mp4", false);
    videos.insert(map_item("eyesRight", eyesRight));

    mediaPlayer.init(video_db+"04_NeutralFace_.mp4");
}

RoboyFacialExpression::~RoboyFacialExpression() 
{
    mediaPlayer.shutdown();
}

void
RoboyFacialExpression::init(std::string facial_expression)
{
    // FIXME: Check if facial_expression exists
    if(videos.find(facial_expression) == videos.end()) {
      TRACE_RED_BOLD("Invalid facial expression: %s", facial_expression.c_str());TRACE("\n");
      return;
    }
    mediaPlayer.newPlaylist(videos[facial_expression]);
}

void RoboyFacialExpression::breakLoop()
{
    mediaPlayer.breakLoop();
}

void RoboyFacialExpression::backToNormalFast()
{
    mediaPlayer.backToNormalFast();
}

void 
RoboyFacialExpression::execute()
{

	std::cerr << " Facial expression EXECUTED " << std::endl;
    /*
    while(true)
    {
        while(!isBodyActivityFinished)
        {
            usleep(20000);
        }
    }
    */
	//isBodyActivityFinished = true;
}

void
RoboyFacialExpression::terminate()
{
	std::cerr << " Facial Expression TERMINATE " << std::endl;
	filetoplay = "";
}


std::string
RoboyFacialExpression::getInfo()
{
	std::string info = "Facial expression: ";
	info.append(filetoplay);
	return info;      
}

bool
RoboyFacialExpression::startedNewPlayList() {
  return mediaPlayer.startedNewPlayList();
}

}
