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
#include <fstream>
#include <sstream>

#include <math.h>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "RoboyTransforms.h"

using namespace Roboy;

namespace Roboy
{

cv::Mat
RoboyTransforms::trimImageToMaxSize(cv::Mat img, cv::Size maxSize)
{

  cv::Size imgSize = img.size();

  float scale;
  float whRatio = imgSize.width / imgSize.height;

  if( whRatio > 1 )
  {

    if(imgSize.width > maxSize.width)
      scale = (float) maxSize.width / (float) imgSize.width;
    else scale = 1;

  } else {
       
    if(imgSize.height > maxSize.height)
      scale = (float) maxSize.height / (float) imgSize.height;
    else scale = 1;

  }

  cv::Mat resized;
  
  if(scale != 1){
    resize(img, resized, cv::Size((int) (imgSize.width * scale), (int) (imgSize.height * scale)));
    return resized;
  } else return img;
  
}

  cv::Mat 
  RoboyTransforms::rotateImage(cv::Mat img, double angle)
  {
    cv::Point2f imgCenter(img.cols/2.0F, img.rows/2.0F);
    cv::Mat rMatrix = cv::getRotationMatrix2D(imgCenter, angle, 1.0); // rotation matrix
    cv::Mat dest;
    cv::warpAffine(img, dest, rMatrix, img.size());
    return dest;
}


  cv::Mat 
  RoboyTransforms::ScaleRotateTranslate(cv::Mat img, float angle, cv::Point* center, cv::Point* newCenter, float scale)
  {
    if (scale == 0 && center == 0)
      return rotateImage(img, angle);
    
    int nx = center->x, ny = center->y, x = center->x, y = center->y;
    float sx = 1.0, sy = 1.0;

    if (newCenter != 0)
    {
      nx = newCenter->x;
      ny = newCenter->y;  
    }

    if (scale != 0)
    {
      sx = scale;
      sy = scale;
    }
    
    cv::Mat dest, t(2,3,CV_64F);
    
    float cosine = cos(angle);
    float sine = sin(angle);
    t.at<double>(0,0) = cosine / sx;
    t.at<double>(0,1) = sine / sx;
    t.at<double>(0,2) = x - nx * t.at<double>(0,0) - ny * t.at<double>(0,1);
    t.at<double>(1,0) = -sine / sy;
    t.at<double>(1,1) = cosine / sy;
    t.at<double>(1,2) =  y - nx * t.at<double>(1,0) - ny * t.at<double>(1,1);

    
    cv::warpAffine(img, dest, t, img.size());
    
    return dest;

  }



}
