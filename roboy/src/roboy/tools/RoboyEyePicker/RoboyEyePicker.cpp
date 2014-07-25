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


#include "opencv2/opencv.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include <math.h>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include <cv.h>
#include <highgui.h>

#include "RoboyTransforms.h"
#include "RoboyGeometry.h"
#include "RoboyFileSystem.h"
#include "RoboyFileWriter.h"


using namespace cv;
using namespace std;


Mat cropFace(Mat img, Point leftEye, Point rightEye, Size size, float offset)
{

  // calculate offsets in original image
  int offset_h = floor(float(offset) * size.width);
  int offset_v = floor(float(offset) * size.height);
  
  // get the direction
  Point eyeDirection(rightEye.x - leftEye.x, rightEye.y - leftEye.y);

  // calc rotation angle in radians
  float rotation = atan2(float(eyeDirection.y),float(eyeDirection.x));
  float dist = Roboy::RoboyGeometry::pointDistance(leftEye, rightEye);

  // calculate the reference eye-width
  float reference = size.width- 2.0 * offset_h;
  
  // scale factor
  float scale = float(dist)/float(reference);

  // rotate original around the left eye
  Mat image = Roboy::RoboyTransforms::ScaleRotateTranslate(img, rotation, &leftEye);

  // crop the rotated image
  Point crop_xy((int) (leftEye.x - scale * offset_h), (int) (leftEye.y - scale * offset_v));
  Size crop_size((int) (size.width * scale), (int) (size.height * scale));
  
  cv::Rect roi(crop_xy.x, crop_xy.y, crop_size.width, crop_size.height);
 
  Mat image2;
  image = image(roi);

  // resize it
  Mat resized;
  resize(image, resized, size); //, Image.ANTIALIAS);

  return resized;
}

int currentEye;
int leftEye[2], rightEye[2];

void mouseEvent(int event, int x, int y, int flags, void* params)
{

	switch( event )
  {

		case CV_EVENT_LBUTTONUP:

      if( currentEye == 1 ){
				rightEye[0] = x;
        rightEye[1] = y;
        currentEye++;
			}

			if( currentEye == 0 ){
				leftEye[0] = x;
        leftEye[1] = y;
        currentEye++;
			}

			break;
	}
}


int main(int argc, const char *argv[]) {

  // check for command line arguments
  if (argc != 1) {
    cout << "usage: " << argv[0] << " <source_directory> <destination_directory>" << endl;
    exit(1);
  }

  const char* name = "photos";
	namedWindow( "photos" , CV_WINDOW_AUTOSIZE);

	// Set up the mouse callback
	  cvSetMouseCallback( "photos", mouseEvent, (void*) 0);

  int maxWidth = 640, maxHeight = 480;
  int eyeDisplayRadius = 10;

  string srcRoot = "/home/hgmarques/Documents/academia/code/Roboy/photos/raw";
  string destRoot = "/home/hgmarques/Documents/academia/code/Roboy/photos/processed";
  string extFile = "/home/hgmarques/Documents/academia/code/Roboy/album.csv";
  vector<string> filelist;

  // get file list
  Roboy::RoboyFileSystem::getSubdirectoryFileList(srcRoot, filelist) ;

  mkdir(destRoot.c_str(), 0777);

  string fullDirectory, currentDirectory = "", directory, filename, destFile, destDirectory;
  string row;

  // create training database file
  Roboy::RoboyFileWriter::createFile(extFile);

  int directoryCounter = -1;
  
  // go through the file list
  for(int i=0; i<filelist.size(); i++)
  {
    Roboy::RoboyFileSystem::getDirectory(filelist[i], fullDirectory);
    Roboy::RoboyFileSystem::getFilename(filelist[i], filename);
    Roboy::RoboyFileSystem::getFilename(fullDirectory, directory);

    destDirectory = destRoot + "/" + directory;
    destFile = destDirectory + "/" + filename;   

    // update database
    if (strcmp(fullDirectory.c_str(), currentDirectory.c_str()))
    {
      currentDirectory = fullDirectory;
      directoryCounter++;
    }   

    char num[10];
    sprintf(num, "%d", directoryCounter);
    row = destFile + ":" + num + ":" + directory;
    Roboy::RoboyFileWriter::writeLine(extFile, row);


    if(Roboy::RoboyFileSystem::fileExists(destFile)) continue;

    // create new face sample
    mkdir(destDirectory.c_str(), 0777);

    Mat photo = imread(filelist[i], CV_LOAD_IMAGE_COLOR);
    photo = Roboy::RoboyTransforms::trimImageToMaxSize(photo, Size(maxWidth, maxHeight));
  
    while (currentEye != 2)
    {    
      imshow("photos", photo);
      waitKey(20);
    }

    Mat face = cropFace(photo, Point(leftEye[0], leftEye[1]), Point(rightEye[0], rightEye[1]), Size(70, 70), 0.2);
    imshow("face", face);
    imwrite( destFile, face );
    currentEye = 0;
  }
  
  char key = (char) waitKey(0); 
  return 0;
}


