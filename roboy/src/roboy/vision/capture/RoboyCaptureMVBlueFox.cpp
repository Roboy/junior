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

#include "RoboyCaptureMVBlueFox.h"

namespace Roboy
{

RoboyCaptureMVBlueFox::RoboyCaptureMVBlueFox(int deviceNr) : RoboyCapture()
{
	this->deviceNr = deviceNr;

}

RoboyCaptureMVBlueFox::~RoboyCaptureMVBlueFox() 
{

}


void
RoboyCaptureMVBlueFox::init()
{


	std::cerr << "INIT "<< std::endl;

	// prefill the capture queue. There can be more then 1 queue for some device, but for this sample
	// we will work with the default capture queue. If a device supports more then one capture or result
	// queue, this will be stated in the manual. If nothing is set about it, the device supports one
	// queue only. Request as many images as possible. If there are no more free requests 'DEV_NO_FREE_REQUEST_AVAILABLE'
	// will be returned by the driver.


}

void 
RoboyCaptureMVBlueFox::execute()
{
	// FIXME: most of this code should go into the constructor and init() functions. For some reason if we do so it does not work

	DeviceManager devMgr;

	const unsigned int devCnt = devMgr.deviceCount();
	if( devCnt == 0 )
	{
		std::cout << "No MATRIX VISION device found! Unable to continue!" << std::endl;
		return;
	}

	device = devMgr[deviceNr];

	try	{

		device->open();

	} catch( const ImpactAcquireException& e )	{

		// this e.g. might happen if the same device is already opened in another process...
		std::cout << "An error occurred while opening the device " << device->serial.read() 
																		<< "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << ")). Terminating thread." << std::endl 
																		<< "Press [ENTER] to end the application..."
																		<< std::endl;

		return;
	}

	// establish access to the statistic properties
	Statistics statistics( device );

	// create an interface to the device found
	FunctionInterface interface( device );

	int result = DMR_NO_ERROR;

	SystemSettings ss( device );

	const int REQUEST_COUNT = ss.requestCount.read();

	for( int i=0; i<REQUEST_COUNT; i++ )
	{
		result = interface.imageRequestSingle();

		if( result != DMR_NO_ERROR )
		{
			std::cout << "Error while filling the request queue: " << ImpactAcquireException::getErrorCodeAsString( result ) << std::endl;
		}
	}

	// thread loop
	int w, h;

	unsigned int cnt = 0;

	//const Request* pRequest = 0;

	const unsigned int timeout_ms = 8000;   // USB 1.1 on an embedded system needs a large timeout for the first image

	int requestNr = INVALID_ID;
	// This next comment is valid once we have a display:
	// we always have to keep at least 2 images as the display module might want to repaint the image, thus we
	// can't free it unless we have a assigned the display to a new buffer.
	int lastRequestNr = INVALID_ID;

	while( 1 )
	{

			// wait for results from the default capture queue
			requestNr = interface.imageRequestWaitFor( timeout_ms );

			if( interface.isRequestNrValid( requestNr ) )
			{
				pRequest = interface.getRequest(requestNr);

				if( pRequest->isOK() )
				{
					++cnt;
					// here we can display some statistical information every 100th image
					if( cnt%100 == 0 )
					{
						frameRate = statistics.framesPerSecond.read();
						std::cout << " FR: " << frameRate << std::endl;
					}

				} else {
					std::cout << "Error: " << pRequest->requestResult.readS() << std::endl;
				}


				if( interface.isRequestNrValid( lastRequestNr ) )
				{
					// this image has been displayed thus the buffer is no longer needed...

					interface.imageRequestUnlock( lastRequestNr );
				}

				lastRequestNr = requestNr;

				// send a new image request into the capture queue
				interface.imageRequestSingle();


			} else {

				// If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide 
				// additional information under TDMR_ERROR in the interface reference (
				std::cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ", device " << device->serial.read() << ")"
						<< ", timeout value too small?" << std::endl;
			}

			int h = pRequest->imageHeight.read();
			int w = pRequest->imageWidth.read();
			mutex.lock();
			frame = cv::Mat(h, w, CV_8UC1, pRequest->imageData.read());
			mutex.unlock();
	}

	if( interface.isRequestNrValid( requestNr ) )
	{
		interface.imageRequestUnlock( requestNr );
	}
	// clear the request queue
	interface.imageRequestReset( 0, 0 );


}



void
RoboyCaptureMVBlueFox::terminate()
{

	// free the last potential locked request

}


cv::Mat
RoboyCaptureMVBlueFox::getFrame()
{
	cv::Mat buffer;
	mutex.lock();
	frame.copyTo(buffer);
	mutex.unlock();
	return buffer;
}


std::string
RoboyCaptureMVBlueFox::getInfo()
{
	return "mvBlueFox";      
}

}
