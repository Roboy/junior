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


#include "mvIMPACT_CPP/mvIMPACT_acquire.h"

#include "RoboyMVBlueFox.h"


using namespace mvIMPACT::acquire;
using namespace std;

RoboyMVBlueFox::RoboyMVBlueFox()
{

	unsigned int cnt = 0;

	DeviceManager devMgr;
	const unsigned int devCnt = devMgr.deviceCount();
	if( devCnt == 0 )
	{
		cout << "No MATRIX VISION device found! Unable to continue!" << endl;
		return;
	}

	cout << "Number of Devices " << devCnt << endl;

	Device* device = devMgr[0];

	try
	{
		device->open();
	}
	catch( const ImpactAcquireException& e )
	{
		// this e.g. might happen if the same device is already opened in another process...
		cout << "An error occurred while opening the device " << device->serial.read() 
				 << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << ")). Terminating thread." << endl 
				 << "Press [ENTER] to end the application..."
				 << endl;
		return;
	}

	// establish access to the statistic properties
	Statistics statistics( device );
	// create an interface to the device found
	FunctionInterface fi( device );

	// prefill the capture queue. There can be more then 1 queue for some device, but for this sample
	// we will work with the default capture queue. If a device supports more then one capture or result
	// queue, this will be stated in the manual. If nothing is set about it, the device supports one
	// queue only. Request as many images as possible. If there are no more free requests 'DEV_NO_FREE_REQUEST_AVAILABLE'
	// will be returned by the driver.
	int result = DMR_NO_ERROR;
	SystemSettings ss( device );
	const int REQUEST_COUNT = ss.requestCount.read();
	for( int i=0; i<REQUEST_COUNT; i++ )
	{
		result = fi.imageRequestSingle();
		if( result != DMR_NO_ERROR )
		{
			cout << "Error while filling the request queue: " << ImpactAcquireException::getErrorCodeAsString( result ) << endl;
		}
	}

	bool terminated = false;
	// run thread loop
	const Request* pRequest = 0;
	const unsigned int timeout_ms = 8000;   // USB 1.1 on an embedded system needs a large timeout for the first image
	int requestNr = INVALID_ID;
	// This next comment is valid once we have a display:
	// we always have to keep at least 2 images as the display module might want to repaint the image, thus we
	// can't free it unless we have a assigned the display to a new buffer.
	int lastRequestNr = INVALID_ID;
	while( !terminated )
	{
		// wait for results from the default capture queue
		requestNr = fi.imageRequestWaitFor( timeout_ms );
		if( fi.isRequestNrValid( requestNr ) )
		{
			pRequest = fi.getRequest(requestNr);
			if( pRequest->isOK() )
			{
				++cnt;
				// here we can display some statistical information every 100th image
				if( cnt%100 == 0 )
				{
					cout << "Info from " << device->serial.read() 
							<< ": " << statistics.framesPerSecond.name() << ": " << statistics.framesPerSecond.readS()
							<< ", " << statis§tics.errorCount.name() << ": " << statistics.errorCount.readS()
							<< ", " << statistics.captureTime_s.name() << ": " << statistics.captureTime_s.readS() << endl;
				}
			}
			else
			{
				cout << "Error: " << pRequest->requestResult.readS() << endl;
			}
			if( fi.isRequestNrValid( lastRequestNr ) )
			{
				// this image has been displayed thus the buffer is no longer needed...
				fi.imageRequestUnlock( lastRequestNr );
			}
			lastRequestNr = requestNr;
			// send a new image request into the capture queue
			fi.imageRequestSingle();
		}
		else
		{
			// If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide 
			// additional information under TDMR_ERROR in the interface reference (
			cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ", device " << device->serial.read() << ")"
					<< ", timeout value too small?" << endl;
		}

		int h = pRequest->imageHeight.read();
		int w = pRequest->imageWidth.read();
		int size = pRequest->imageSize.read();

		std::cerr << " size: " << w << " x " << h << " = " << size <<  std::endl;

		cv::Mat image(h, w, CV_8UC1, pRequest->imageData.read());

		imshow("Hello from mvBlueFOX", image);
		//	 		 imshow("Hello from mvBlueFOX", imagetmp);

		char key = (char) cv::waitKey(20);

		if ( key == 27 )
			terminated = true;

	}

	// free the last potential locked request
	if( fi.isRequestNrValid( requestNr ) )
	{
		fi.imageRequestUnlock( requestNr );
	}
	// clear the request queue
	fi.imageRequestReset( 0, 0 );

}

/*
RoboyMVBlueFox::RoboyMVBlueFox()
{

	cout << "Boooooo -3!" << endl;

	DeviceManager devMgr;


	cout << "Boooooo -2!" << endl;

	Device* pDev = devMgr[0];

	cout << "Boooooo -1!" << endl;

//	Device* pDev = getDeviceFromUserInput( devMgr );
	if( !pDev )
	{
		cout << "Cannot find camera device!" << endl;
		return;
	}

	cout << "Boooooo!" << endl;



	try
	{
		pDev->open();
	}
	catch( const ImpactAcquireException& e )
	{
		// this e.g. might happen if the same device is already opened in another process...
		cout << "An error occurred while opening the device(error code: " << e.getErrorCode() << "). Press [ENTER] to end the application..." << endl;
		cout << "Press [ENTER] to end the application" << endl;
		cin.get();
		return;
	}


	cout << "Boooooo2!" << endl;

	FunctionInterface* fi = new FunctionInterface( pDev );

	cout << "Boooooo3!" << endl;

	// send a request to the default request queue of the device and wait for the result.
	fi->imageRequestSingle();
	// Start the acquisition manually if this was requested(this is to prepare the driver for data capture and tell the device to start streaming data)
	if( pDev->acquisitionStartStopBehaviour.read() == assbUser )
	{
		TDMR_ERROR result = DMR_NO_ERROR;
		if( ( result = static_cast<TDMR_ERROR>(fi->acquisitionStart()) ) != DMR_NO_ERROR )
		{
			cout << "'FunctionInterface.acquisitionStart' returned with an unexpected result: " << result
				<< "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
		}
	}
	// Define the Image Result Timeout (The maximum time allowed for the Application 
	// to wait for a Result). Infinity value:-1
	const int iMaxWaitTime_ms = -1;   // USB 1.1 on an embedded system needs a large timeout for the first image.
	// wait for results from the default capture queue.
	int requestNr = fi->imageRequestWaitFor( iMaxWaitTime_ms );

	// check if the image has been captured without any problems.
	if( !fi->isRequestNrValid( requestNr ) )
	{
			// If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide 
			// additional information under TDMR_ERROR in the interface reference
			cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ")"
				 << ", timeout value too small?" << endl;
		return;
	}

	const Request* req = fi->getRequest( requestNr );
	if( !req->isOK() )
	{
		cout << "Error: " << req->requestResult.readS() << endl;
		// if the application wouldn't terminate at this point this buffer HAS TO be unlocked before
		// it can be used again as currently it is under control of the user. However terminating the application
		// will free the resources anyway thus the call
		// fi.imageRequestUnlock( requestNr );
		// can be omitted here.
		return;
	}


	 int h = req->imageHeight.read();
	 int w = req->imageWidth.read();
	 int size = req->imageSize.read();

	 std::cerr << " size: " << w << " x " << h << " = " << size <<  std::endl;

	 cv::Mat image(h, w, CV_8UC1, req->imageData.read());

	 std::cerr << " copied " << std::endl;

	// std::cerr << image.at<uchar>(10,10) << std::endl;

//	 cv::Mat imagetmp = cv::imread("/home/hgmarques/Pictures/a.png", CV_LOAD_IMAGE_COLOR);


	 while(1)
	 	 {

	 		int requestNr = fi->imageRequestWaitFor( 50 );

	 		// check if the image has been captured without any problems.
	 		if( !fi->isRequestNrValid( requestNr ) )
	 		{
	 				// If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide 
	 				// additional information under TDMR_ERROR in the interface reference
	 				cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ")"
	 					 << ", timeout value too small?" << endl;
	 			return;
	 		}

	 		 const Request* req = fi->getRequest( requestNr );
	 		 cv::Mat image(h, w, CV_8UC1, req->imageData.read());

	 		 imshow("Hello from mvBlueFOX", image);
//	 		 imshow("Hello from mvBlueFOX", imagetmp);

	 		 char key = (char) cv::waitKey(20);

	 		 if ( key == 27 )
	 			 break;

	 		 requestNr++;
	 	 }


//	 cv::Mat imagetmp = cv::imread("'/home/hgmarques/Pictures/a.png");


//	 
//	 	
//	 memcpy(image.ptr(), req->imageData.read(), size );

//    image.data.resize(req->imageSize.read());
//    image.height = req->imageHeight.read();
//    image.width = req->imageWidth.read();
//    image.step = req->imageLinePitch.read();
//    image.header.seq = req->infoFrameNr.read();
//
//    if(req->imageBayerMosaicParity.read() != mvIMPACT::acquire::bmpUndefined)
//    {
//      int bpp = req->imageBytesPerPixel.read()*8;
//      image.encoding = bayerString(req->imageBayerMosaicParity.read(), bpp);
////        ROS_INFO_STREAM_THROTTLE(1, "raw image, encoding: "<<image.encoding<<" bpp"<<bpp);
//    }
//    else
//    {
//      image.encoding = pixelFormat(req->imagePixelFormat.read());
////        ROS_INFO_STREAM_THROTTLE(1, "(processed) image, encoding: "<<image.encoding);
//    }
//





	// unlock the buffer to let the driver know that you no longer need this buffer.
	fi->imageRequestUnlock( requestNr );

}


 */

cv::Mat
RoboyMVBlueFox::captureFrame()
{
	return frame;
}


cv::Mat
RoboyMVBlueFox::getFrame()
{
	return frame;
}

