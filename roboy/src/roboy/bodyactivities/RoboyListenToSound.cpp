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


#include "RoboyListenToSound.h"

#include <iostream>
#include <math.h>
#include <string.h>
#include <alsa/asoundlib.h>

#include "ALSAtestfunctions.h"

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API
#define FILTER_SIZE                9000
#define NUM_CHANNELS               1
#define NUM_MICROSECONDS_TO_RECORD 5000
#define DEBUG_OUTPUT               true


namespace Roboy
{

RoboyListenToSound::RoboyListenToSound() : RoboyBodyActivity()
{
	std::cout << "RoboyListenToSound CONSTRUCTOR" << std::endl;
	Loudness = 0;
	SoundIsLoud = false;
  listenToSoundThread = new boost::thread( boost::bind(&RoboyListenToSound::updateLoudness, this) );
}


RoboyListenToSound::~RoboyListenToSound() 
{

}

bool
RoboyListenToSound::isSoundLoud()
{
	return SoundIsLoud;
}

short
RoboyListenToSound::getLoudness()
{
	return Loudness;
}


void
RoboyListenToSound::init(std::string filetoplay)
{
	std::cout << "RoboyListenToSound INITIALIZATION" << std::endl;
	
}

void 
RoboyListenToSound::execute()
{          
	
	while (1) {

	} // end while (1)
}


void
RoboyListenToSound::terminate()
{
	std::cout << "RoboyListenToSound Terminating" << std::endl;
  listenToSoundThread->interrupt();

	// setting back the isFinished and isActive Values happens in the stop() function.
}

void
RoboyListenToSound::updateLoudness() {

	Loudness = 0;

/// inserted from C code
			int size;
			int direction;
			unsigned int val;
			snd_pcm_t *pHandle;
			snd_pcm_hw_params_t *pParams;
			snd_pcm_uframes_t frames_per_period = 32;
			char *pBuffer;

			// allocate buffers for last (FILTER_SIZE) values for first and second channel:
			double* pBuffer_lastvalues_first = (double *) malloc(sizeof(double)*FILTER_SIZE);
			double* pBuffer_lastvalues_second = (double *) malloc(sizeof(double)*FILTER_SIZE);

			// initialize buffer
			for (int i = 0; i < FILTER_SIZE; i++)
			{
				pBuffer_lastvalues_first[i] = 0;
				pBuffer_lastvalues_second[i] = 0;
			}

			int current_pointer_first = 0;
			int current_pointer_second = 0;

			short threshold_for_sounddetection = 16000;
			short amplitude_tolerance_to_count_as_same_sound = 1000;
			bool threshold_reached = false;
			bool sound_detected = false;
			// TODO: a function to read out sound_detected and sound_angle has to be written,
			// which can be called by other routines. When this function gets called and
			// sound_detected is true, sound_detected can be set back to false afterwards
			// and sound_angle can be set back to 0 again.
			double sound_angle = 0; // as a first approach: right is 1, left is -1

			double fake_loudness_first = 0;
			double fake_loudness_second = 0;
			double SUM_last_samples_squared_first = 0;
			double SUM_last_samples_squared_second = 0;
			double rms_last_samples_first = 0;
			double rms_last_samples_second = 0;

			double sample_first = 0;
			double sample_second = 0;

			double channel_difference = 0;

			int i;

			int errorCode = setupHardware(&pHandle, &pParams, NUM_CHANNELS, "default", &direction, &frames_per_period, &val, DEBUG_OUTPUT);
			if (errorCode)
			{
				printf("setting up hardware failed (%i)\n", errorCode);
				return;
			}
			else
			{
				printf("# set up hardware successful\n");
			}

		/* Use a buffer which holds one period (not the same as the internal ALSA buffer!) */
			snd_pcm_hw_params_get_period_size(pParams, &frames_per_period, &direction);

			size = frames_per_period * 2 * NUM_CHANNELS; /* 2 bytes/sample, X channels */
			pBuffer = (char *) malloc(size);

			if (pBuffer == NULL)
			{
				printf("Memory for buffer could not be allocated!\n");
				return;
			}

			snd_pcm_hw_params_get_period_time(pParams, &val, &direction);
			std::cout << "period length: " << val << "\n";


////



	while (1) {
	
		try {

//// inserted from C code

			// read interleaved frames
			errorCode = snd_pcm_readi(pHandle, pBuffer, frames_per_period);
			if (errorCode == -EPIPE)
			{
				// EPIPE means overrun
				fprintf(stderr, "overrun occurred\n");
				snd_pcm_prepare(pHandle);
			}
			else if (errorCode < 0)
			{
				fprintf(stderr,	"error from read: %s\n", snd_strerror(errorCode));
			}
			else if (errorCode != (int)frames_per_period)
			{
				fprintf(stderr, "short read, read %d frames_per_period\n", errorCode);
			}

			// loop over all samples (each frame consists of two two-byte samples, one for left, one for right)
			for (i = 0; i < (int)frames_per_period * (int)NUM_CHANNELS; i++)
			{
				//first channel (always first channel if only 1 channel):
				if (i%2 == 0 || NUM_CHANNELS == 1)
				{
					// to build the sum of the current and all the FILTER_SIZE-1 last squared samples,
					// first we subtract the last one in the queue
					// (which is exactly at position current_pointer):
					SUM_last_samples_squared_first = SUM_last_samples_squared_first - pBuffer_lastvalues_first[current_pointer_first]
						*pBuffer_lastvalues_first[current_pointer_first];

					// then we can overwrite the current position:
					sample_first = (double) *( (short*) &(pBuffer[i*2]) );
					pBuffer_lastvalues_first[current_pointer_first] = sample_first;

					fake_loudness_first += abs(sample_first);

					// then we add the squared current sample:
					SUM_last_samples_squared_first = SUM_last_samples_squared_first + sample_first*sample_first;

					// for the rms we have to take the average of the 100 squared values and then take the root:
					// (thom: i'd say this should exactly be the standard deviation)
					rms_last_samples_first = sqrt(SUM_last_samples_squared_first/FILTER_SIZE);

					//std::cout << rms_last_samples_first;
					//std::cout << sample_first;
					//std::cout << "\n";

					if (current_pointer_first == FILTER_SIZE-1)
					{
						current_pointer_first = 0;
					}
					else
					{
						current_pointer_first++;
					}

				}
				// repeat everything for second channel:
				else
				{
					// to build the sum of the current and all the FILTER_SIZE-1 last squared samples,
					// first we subtract the last one in the queue
					// (which is exactly at position current_pointer): 
					SUM_last_samples_squared_second = SUM_last_samples_squared_second - pBuffer_lastvalues_second[current_pointer_second]
						*pBuffer_lastvalues_second[current_pointer_second];

					// then we can overwrite the current position:
					sample_second = (double) *( (short*) &(pBuffer[i*2]) );
					pBuffer_lastvalues_second[current_pointer_second] = sample_second;

					fake_loudness_second += abs(sample_second);

					// then we add the squared current sample:
					SUM_last_samples_squared_second = SUM_last_samples_squared_second + sample_second*sample_second;

					// for the rms we have to take the average of the 100 squared values and then take the root:
					rms_last_samples_second = sqrt(SUM_last_samples_squared_second/FILTER_SIZE);

					//std::cout << rms_last_samples_second;
					//std::cout << sample_second;
					//std::cout << "\n";

					if (current_pointer_second == FILTER_SIZE-1)
					{
						current_pointer_second = 0;
					}
					else
					{
						current_pointer_second++;
					}

				}

				//// check if loudness treshold was reached ////
				// in case treshold was not reached in the last values,
				// check if a loud sound got detected in the right or left channel
				if (threshold_reached == false)
				{
					if (rms_last_samples_first > threshold_for_sounddetection || rms_last_samples_second > threshold_for_sounddetection)
					{
						threshold_reached = true;

						if (rms_last_samples_first > rms_last_samples_second)
						{
							sound_angle = 1;
						}
						else
						{
							sound_angle = -1;
						}
						sound_detected = true;
					}

					// in case treshold was already reached in the last values,
					// set it back to "not reached" in case both values get down
					// below (treshold minus tolerance)
				}
				else
				{
					if (rms_last_samples_first < (threshold_for_sounddetection - amplitude_tolerance_to_count_as_same_sound)
						&& rms_last_samples_second < (threshold_for_sounddetection - amplitude_tolerance_to_count_as_same_sound))
					{
						threshold_reached = false;
					}
				}

				// calculate the difference between the samples of the two channels
				// (just for debugging)
				channel_difference = sample_first - sample_second;
				//if (loops == 1) {std::cout << channel_difference << "\n";}
				//if (loops < 10) {std::cout << sample_first << "\n";}
				//if (loops == 1) {std::cout << sample_second << "\n";}

			} // end for-loop

			//std::cout << "threshold_reached = " << threshold_reached;

			//std::cout << "\n";
			//std::cout << "sound_detected = " << sound_detected;

			//std::cout << "\n";
			//std::cout << "sound_angle = " << sound_angle;

			//std::cout << "\n";

			fake_loudness_first = fake_loudness_first/((double)frames_per_period * 2);
			fake_loudness_second = fake_loudness_second/((double)frames_per_period * 2);
			//std::cout << fake_loudness;
			//std::cout << "\n";


////

			Loudness = rms_last_samples_first;
			//std::cout << "Loudness : " << Loudness << std::endl;

			SoundIsLoud = threshold_reached;
			//std::cout << "SoundIsLoud : " << SoundIsLoud << std::endl;

 			boost::this_thread::interruption_point();

		} catch(boost::thread_interrupted const&) {
				std::cout << "update Loudness INTERRUPTED" << std::endl;
				break;
		}

	} // end while
	
}



std::string
RoboyListenToSound::getInfo()
{
	std::string info = "Listen to sound";
	return info;      
}


} // end namespace Roboy
