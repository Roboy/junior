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
#ifndef _GENERAL_FUNCTIONS_
#define _GENERAL_FUNCTIONS_

#include "GlobalParams.h"
#include "debug.h"
#include <vector>
#include <sys/stat.h>
#include <sstream>
#include <string>

#define MAX_ERROR 5000
#define INITIAL_SEARCH_TOLERANCE 0
#define MAX_MATCHING_COORD_ORDER 5

#define SECONDS_TO_MICRO_SECONDS 1000000

using std::vector;

int atoh(char input[]);

// 2D Matrix defined in 1D array is in row major form
// 0  1  2  3
// 4  5  6  7
// 8  9  10 11
// 12 13 14 15

float* zeros(int row, int col);
int* zerosINT(int row, int col);

struct matchingCoord {
	int index, value;
};

struct matchingCoordHighOrder {
	int index; 
	int value[MAX_MATCHING_COORD_ORDER];
};

struct bestFitResult {
	bool validResult;
	int matchedValue;
};

struct dataArrayWrapper {
	float *dataArray;
	int col, row;
};

std::string createFolder(const char *prefix);

// Remember to delete/free the data array within the wrapper after using or
// before using the same struct to get another array
// it will lead to memory leaks if not handled properly
dataArrayWrapper loadDataFileArray(const char* fileName, int expectedCols);

// Returns the best x-axis value for the first point on the second
// graph to match the first graph (y-axis value is in even indices and x-axis value is in odd indices)
// Currently using a simple method by shifting the toMatchGraph
// along the x-axis by finding all the matching coordinates for the
// first point. The one with the minimum error for all the other points will be 
// considered as the correct point. Not using a minimum distance/least squared
// approach as that will take too much time to compute
bestFitResult bestFitGraph2D(dataArrayWrapper inputGraph, dataArrayWrapper toMatchGraph);

// A first order interpolation
vector<matchingCoord> findMatchingCoords(dataArrayWrapper inputGraph, int coord, bool isXCoord, float tolerance = 0);

bool isBetween(int firstPoint, int secondPoint, int checkPoint, float tolerance = 0);
int interpolate(int ref1, int output1, int ref2, int output2, int check);

vector<matchingCoordHighOrder> findMatchingCoords(dataArrayWrapper inputGraph, int coord, int checkCol);

int bound(int input, int max);
int bound(int input, int max, int min);

int max(int a, int b);
float max(float a, float b);
double max(double a, double b);

int norm(int a);
float norm(float a);
double norm(double a);

#endif
