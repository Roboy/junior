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
#include <cstring>
#include <stdlib.h>

#include <cstdio>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <boost/lexical_cast.hpp>

#include "RoboyFileParser.h"
#include "RoboyMediaPlayerInteraction.h"

#define MPLAYER_BIN_FULL_SCREEN "mplayer -ao null -msglevel global=6 -fs -quiet -xineramascreen 1 -input nodefault-bindings -noconfig all -slave -idle -input file="
#define MPLAYER_BIN_NO_FULL_SCREEN "mplayer -ao null -msglevel global=6 -geometry 200x200 -quiet -xineramascreen 1 -input nodefault-bindings -noconfig all -slave -idle -input file="



RoboyMediaPlayerInteraction::RoboyMediaPlayerInteraction()
{
}

RoboyMediaPlayerInteraction::~RoboyMediaPlayerInteraction()
{
}


bool RoboyMediaPlayerInteraction::init(std::string normal)
{
    this->normal = normal;
    this->blink = RoboyFileParser::parser->get<std::string>("database", "DATABASE_PATH")+
                  "/Videos/"+
                  RoboyFileParser::parser->get<std::string>("faceexpressions", "BLINK_VIDEO", normal);
    srand(time(NULL));
    fifo_path = RoboyFileParser::parser->get<std::string>("faceexpressions", "MPLAYER_FIFO_PATH", "/tmp/roboy_mplayer_fifo_");
    fifo_path += boost::lexical_cast<std::string>(rand());
    std::cout << "Creating fifo file: " << fifo_path << std::endl; // FIXME: Logger info
    std::remove(fifo_path.c_str());
    mkfifo(fifo_path.c_str(), S_IWRITE|S_IREAD);
    fifo = open(fifo_path.c_str(), O_RDWR);
    if(fifo < 0)
    {
        std::cout << "Failed to open fifo " << fifo_path << " for writing." << std::endl; // FIXME: Logger fatal
        return false;
    }
    
	std::string fullScreenStr = RoboyFileParser::parser->get<std::string>("faceexpressions", "MPLAYER_USE_FULL_SCREEN");
	mplayerBin = (!fullScreenStr.compare("YES")) ? MPLAYER_BIN_FULL_SCREEN : MPLAYER_BIN_NO_FULL_SCREEN;
    mplayerBin += fifo_path;

	playerThread = new boost::thread( boost::bind( &RoboyMediaPlayerInteraction::runPlayer, this ) );
    sleep(1); // There could be a race condition here
    play(normal);


	std::cout << "MPLAYER?? mplayerBin???? " << mplayerBin<< std::endl;
	return true;
}



void 
RoboyMediaPlayerInteraction::shutdown()
{
	sendCommand("quit");
    close(fifo);
	
	playerThread->interrupt();
	playerThread->join();

    std::remove(fifo_path.c_str());

    std::cout << "mplayer shut down." << std::endl;
}

void RoboyMediaPlayerInteraction::setFullscreen(bool b)
{
	// Note: This only works after first video is displayed.
    // But the paramter -fs works from the beginning.
	if(b) sendCommand("vo_fullscreen 1");
	else  sendCommand("vo_fullscreen 0");
}

void RoboyMediaPlayerInteraction::newPlaylist(Roboy::RoboyVideoPlaylist videos)
{
    playlist_mutex.lock();
    playlists.push(videos);
    playlist_mutex.unlock();
    if(playlists.size() == 1)
        playNext();
}


void 
RoboyMediaPlayerInteraction::runPlayer()
{
    fclose(stderr); // Because mplayer outputs stuff that clutters the console
	FILE *fh = popen(mplayerBin.c_str(), "r");
	size_t bufSize = 511;
	char *buf = (char*) malloc(bufSize+1);
	while(1)
	{
		boost::this_thread::interruption_point(); 
		getline(&buf, &bufSize, fh);
        // std::cout << buf; FIXME: Logger: debug
        if(!std::strncmp(buf, "EOF code: 1", 11))
			playNext();
	}
}


void RoboyMediaPlayerInteraction::play(std::string file)
{
	sendCommand("loadfile " + file);
}

void 
RoboyMediaPlayerInteraction::sendCommand(std::string cmd)
{
	fifo_mutex.lock(); 
    dprintf(fifo, "%s\n", cmd.c_str()); // This is unbuffered
	fifo_mutex.unlock();
	//std::cout << "CALLING: " << cmd << std::endl;
}

void RoboyMediaPlayerInteraction::breakLoop()
{
    playlist_mutex.lock();
    if(!playlists.empty()) playlists.front().breakCurrentVideo();
    playlist_mutex.unlock();
}

void RoboyMediaPlayerInteraction::backToNormalFast()
{
  while(!playlists.empty()) {
    playlists.pop();
  }
  play(normal);
}

bool RoboyMediaPlayerInteraction::startedNewPlayList() {
  if(playlists.size() <= 1) return true;
  return false;
}

void RoboyMediaPlayerInteraction::playNext()
{
    playlist_mutex.lock();
    // To handle a second item in playlist
    if(playlists.size() > 1) {
      if(playlists.front().endOfPlaylist()) {
        playlists.pop();
        play(playlists.front().getNextVideo());
      } else {
        play(playlists.front().getNextVideo(true));
      }
      playlist_mutex.unlock();
      return;
    }
      
    if(!playlists.empty() && playlists.front().endOfPlaylist())
        playlists.pop();

    if(playlists.empty())
    {
        // if (rand() % 10) is 0, blink. (rand() % 10) is 0 in 1/10 of the cases,
        // that is 10%. 
        if(rand() % 10) play(normal);
        else play(blink);
    }
    else
    {
        play(playlists.front().getNextVideo());
    }
    playlist_mutex.unlock();
}
