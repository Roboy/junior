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
#include "ALSAtestfunctions.h"
#include <string.h>
#include <alsa/asoundlib.h>
#include <iostream>

using namespace std;


int setupHardware(snd_pcm_t **ppHandle, snd_pcm_hw_params_t **ppParams, unsigned int numChannels, char *pDeviceName, int *pDirection, snd_pcm_uframes_t *pFrames_per_period, unsigned int *pVal, bool outputDebug)
{
	int errorCode;

	// settings
	unsigned int sampleRate = 44100; // [Hz]

	/* Open PCM device for recording (capture). */
	errorCode = snd_pcm_open(ppHandle, pDeviceName, SND_PCM_STREAM_CAPTURE, 0);

	// check for error
	if (errorCode < 0)
	{
		//fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(errorCode));
		return errorCode;
	}
	else
	{
		//printf("# openend pcm device '%s'\n", pDeviceName);
	}

	/* Allocate a hardware parameters object. */
	snd_pcm_hw_params_alloca(ppParams);

	/* Fill it in with default values. */
	snd_pcm_hw_params_any(*ppHandle, *ppParams);

	/* Set the desired hardware parameters. */

	/* Interleaved mode */
	snd_pcm_hw_params_set_access(*ppHandle, *ppParams, SND_PCM_ACCESS_RW_INTERLEAVED);

	/* Signed 16-bit little-endian format */
	snd_pcm_hw_params_set_format(*ppHandle, *ppParams, SND_PCM_FORMAT_S16_LE);

	/* Set Channels */
	errorCode = snd_pcm_hw_params_set_channels(*ppHandle, *ppParams, numChannels);

	// check for error
	if (errorCode < 0)
	{
		//printf("Channels not available.\n");
		return errorCode;
	}

	/* set sample rate */
	snd_pcm_hw_params_set_rate_near(*ppHandle, *ppParams, &sampleRate, pDirection);

	/* Set period size */
	snd_pcm_hw_params_set_period_size_near(*ppHandle, *ppParams, pFrames_per_period, pDirection);

	/* Write the parameters to the driver */
	errorCode = snd_pcm_hw_params(*ppHandle, *ppParams);
	if (errorCode < 0)
	{
		//fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(errorCode));
		return errorCode;
	}

	if (outputDebug)
	{
		unsigned int val2;

		//printf("PCM handle name = '%s'\n", snd_pcm_name(*ppHandle));

		//printf("PCM state = %s\n", snd_pcm_state_name(snd_pcm_state(*ppHandle)));

		snd_pcm_hw_params_get_access(*ppParams, (snd_pcm_access_t *) pVal);
		//printf("access type = %s\n", snd_pcm_access_name((snd_pcm_access_t)*pVal));

		snd_pcm_hw_params_get_format(*ppParams, (snd_pcm_format_t *) pVal);
		//printf("format = '%s' (%s)\n", snd_pcm_format_name((snd_pcm_format_t)*pVal), snd_pcm_format_description((snd_pcm_format_t)*pVal));

		snd_pcm_hw_params_get_subformat(*ppParams, (snd_pcm_subformat_t *)pVal);
		//printf("subformat = '%s' (%s)\n", snd_pcm_subformat_name((snd_pcm_subformat_t)*pVal), snd_pcm_subformat_description((snd_pcm_subformat_t)*pVal));

		snd_pcm_hw_params_get_channels(*ppParams, pVal);
		//printf("channels = %d\n", *pVal);

		snd_pcm_hw_params_get_rate(*ppParams, pVal, pDirection);
		//printf("rate = %d bps\n", *pVal);

		snd_pcm_hw_params_get_period_time(*ppParams, pVal, pDirection);
		//printf("period time = %d us\n", *pVal);

		snd_pcm_hw_params_get_period_size(*ppParams, pFrames_per_period, pDirection);
		//printf("period size = %d frames\n", (int)*pFrames_per_period);

		snd_pcm_hw_params_get_buffer_time(*ppParams, pVal, pDirection);
		//printf("buffer time = %d us\n", *pVal);

		snd_pcm_hw_params_get_buffer_size(*ppParams, (snd_pcm_uframes_t *) pVal);
		//printf("buffer size = %d frames\n", *pVal);

		snd_pcm_hw_params_get_periods(*ppParams, pVal, pDirection);
		//printf("periods per buffer = %d \n", *pVal);

		snd_pcm_hw_params_get_rate_numden(*ppParams, pVal, &val2);
		//printf("exact rate = %d/%d bps\n", *pVal, val2);

		*pVal = snd_pcm_hw_params_get_sbits(*ppParams);
		//printf("significant bits = %d\n", *pVal);

		//snd_pcm_hw_params_get_tick_time(*ppParams, pVal, pDirection);
		////printf("tick time = %d us\n", *pVal);

		*pVal = snd_pcm_hw_params_is_batch(*ppParams);
		//printf("is batch = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_is_block_transfer(*ppParams);
		//printf("is block transfer = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_is_double(*ppParams);
		//printf("is double = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_is_half_duplex(*ppParams);
		//printf("is half duplex = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_is_joint_duplex(*ppParams);
		//printf("is joint duplex = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_can_overrange(*ppParams);
		//printf("can overrange = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_can_mmap_sample_resolution(*ppParams);
		//printf("can mmap = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_can_pause(*ppParams);
		//printf("can pause = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_can_resume(*ppParams);
		//printf("can resume = %d\n", *pVal);

		*pVal = snd_pcm_hw_params_can_sync_start(*ppParams);
		//printf("can sync start = %d\n", *pVal);
	}

	return 0;
}
