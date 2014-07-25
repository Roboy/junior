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
#include "genFunc.h"

int atoh(char input[]) {
	int output = 0;
	int size, i, sign;
	
	size = sizeof(input)/sizeof(char);
	
	if(size < 1) return 0;
	if(input[0] == 45) {
		sign = -1;
		i = 1;
	} else {
		sign = 1;
		i = 0;
	}
	
	for(;i < size; i++) {
		if(input[i] >= '0' && input[i] <= '9') {
			output = output*16 + ((int) input[i]) - 48;
		} else if(input[i] >= 'A' && input[i] <= 'F') {
			output = output*16 + ((int) input[i]) - 55;
		} else if(input[i] >= 'a' && input[i] <= 'f') {
			output = output*16 + ((int) input[i]) - 87;
		} else {
			break;
		}
	}
	
	return output*sign;
}

// 2D Matrix defined in 1D array is in row major form
// 0  1  2  3
// 4  5  6  7
// 8  9  10 11
// 12 13 14 15

float* zeros(int row, int col) {
  float *mat;
  int i;
  mat = (float *)malloc(row*col*sizeof(float));
  for(i = 0; i < col*row;i++) {
    mat[i] = 0;
  }
  return mat;
}

int* zerosINT(int row, int col) {
  int *mat;
  int i;
  mat = (int *)malloc(row*col*sizeof(int));
  for(i = 0; i < col*row;i++) {
    mat[i] = 0;
  }
  return mat;
}


int norm(int a) {
	if(a < 0) 
		return -a; 
	else 
		return a;
}

float norm(float a) {
	if(a < 0) 
		return -a; 
	else 
		return a;
}

double norm(double a) {
	if(a < 0) 
		return -a; 
	else 
		return a;
}

std::string createFolder(const char *prefix) {

	// Create folder
	std::stringstream ss;
	struct tm *ttime;
	time_t tm = time(0);
	char time_string[15];

	ttime = localtime(&tm);
	strftime(time_string,15,"%Y%m%d%H%M%S",ttime); 		
		
	ss << prefix << time_string;
	mkdir(ss.str().c_str(), S_IRWXU|S_IRGRP|S_IXGRP);
	
	return ss.str();
}



// Remember to delete/free the data array after using or
// before using the same pointer to get another array
// will lead to memory leaks if not handled properly
dataArrayWrapper loadDataFileArray(const char* fileName, int expectedCols) {
	dataArrayWrapper dataArray;
	FILE *ptr_data;
  int dataCount, row;
  float data;
	
  dataArray.dataArray = zeros(expectedCols, 1);
  dataArray.row = 0;
  dataArray.col = 0;
  ptr_data = fopen(fileName,"r");
  dataCount = 0;

 	if(ptr_data == NULL) {
		TRACE_RED_BOLD("Unable to open file: %s for reading.", fileName);TRACE("\n");
		return dataArray;
	} 

  while(fscanf(ptr_data,"%f",&data) != EOF) {
    dataCount++;
  }
  if(dataCount%expectedCols != 0) {
		TRACE_RED_BOLD("Data number is incorrect, not loading. Total data: %d, Expected columns: %d", dataCount, expectedCols);
		TRACE("\n");
    return dataArray;
  }
	row = dataCount/expectedCols;
  
  free(dataArray.dataArray);
  rewind(ptr_data);
  dataArray.dataArray = zeros(row, expectedCols);
	dataArray.row = row;
	dataArray.col = expectedCols;
	
  dataCount = 0;

  while(fscanf (ptr_data,"%f",&data) != EOF) {
    dataArray.dataArray[dataCount] = data;
    dataCount++;
  }
	fclose(ptr_data);
  return dataArray;
}

// Returns the best x-axis value for the first point on the second
// graph to match the first graph
// Currently using a simple method by shifting the toMatchGraph
// along the x-axis by finding all the matching coordinates for the
// first point. The one with the minimum error for all the other points will be 
// considered as the correct point. Not using a minimum distance/least squared
// approach as that will take too much time to compute
bestFitResult bestFitGraph2D(dataArrayWrapper inputGraph, dataArrayWrapper toMatchGraph) {
	vector<matchingCoord> setOfMatchingCoords;
	vector<matchingCoord> forErrorSearch;
	bestFitResult outputResult;
	unsigned int i;
	int j;
	int bestMatchIndex, minError, error;
	
	outputResult.validResult = false;

	if(inputGraph.col != 2 || toMatchGraph.col != 2) return outputResult;
	
	setOfMatchingCoords = findMatchingCoords(inputGraph, toMatchGraph.dataArray[0], false, INITIAL_SEARCH_TOLERANCE);
	
	TRACE_CYAN_BOLD("Number of matching coordinates: %d", setOfMatchingCoords.size());TRACE("\n");
	// No solution returns 0
	if(setOfMatchingCoords.size() <= 0) return outputResult;
	
	outputResult.validResult = true;
	// Unique solution
	if(setOfMatchingCoords.size() == 1) {
		outputResult.matchedValue = setOfMatchingCoords[0].value;
		return outputResult;
	}
	
	for(i = 0; i < setOfMatchingCoords.size(); i++) {
	
		TRACE_CYAN_BOLD("  %d: Index: %d\t Value: %d", i, setOfMatchingCoords[i].index, setOfMatchingCoords[i].value);TRACE("\n");
	
	}
	
	//BREAK_POINT("1");
	bestMatchIndex = 0;	
	
	for(i = 0; i < setOfMatchingCoords.size(); i++) {
		error = 0;
		for(j = 1; j < toMatchGraph.row; j++) {
			forErrorSearch = findMatchingCoords(inputGraph, toMatchGraph.dataArray[toMatchGraph.col*j + 1] - toMatchGraph.dataArray[1] + setOfMatchingCoords[i].value, true);
			if(forErrorSearch.size() <= 0) {
				TRACE_RED_BOLD("CANNOT FIND MATCHING X COORDINATE");TRACE("\n");
				error = error + MAX_ERROR;
			} else {
				// Should only have 1 intersection
				TRACE_CYAN_BOLD("Error of point %d: %.1f", j, norm(forErrorSearch[0].value - toMatchGraph.dataArray[toMatchGraph.col*j]));
				TRACE_CYAN_BOLD(" Search %d: To Match: %.1f", forErrorSearch[0].value, toMatchGraph.dataArray[toMatchGraph.col*j]);
				TRACE("\n");
				error = error + norm(forErrorSearch[0].value - toMatchGraph.dataArray[toMatchGraph.col*j]);
			}
		}
		TRACE_PURP_BOLD("Total Error for %d: %d", setOfMatchingCoords[i].value, error);
		TRACE("\n");
						
		if(i == 0) {
			minError = error;
		} else {
			if(minError > error) {
				minError = error;
				bestMatchIndex = i;
			}
		}
	}
	outputResult.matchedValue = setOfMatchingCoords[bestMatchIndex].value;
	return outputResult;
}

bool isBetween(int firstPoint, int secondPoint, int checkPoint, float tolerance) {

	if(firstPoint > secondPoint) {
		if(checkPoint >= (secondPoint + tolerance*(secondPoint - firstPoint)) && checkPoint <= (firstPoint + tolerance*(firstPoint - secondPoint)))
			return true;
	} else {
		if(checkPoint <= (secondPoint + tolerance*(secondPoint - firstPoint)) && checkPoint >= (firstPoint + tolerance*(firstPoint - secondPoint)))
			return true;
	}

	return false;
}

int interpolate(int ref1, int output1, int ref2, int output2, int check) {
	if(ref1 == ref2) return ((output2 + output1)/2);
	return ((int) (check - ref1)*(((float)(output1 - output2))/((float)(ref1 - ref2))) + output1);
}


vector<matchingCoord> findMatchingCoords(dataArrayWrapper inputGraph, int coord, bool isXCoord, float tolerance) {
	vector<matchingCoord> setOfMatchingCoords;
	matchingCoord dummy;
	int i, offset1, offset2;
	
	if(inputGraph.col != 2) return setOfMatchingCoords;
	
	if(isXCoord) {
		offset1 = 1;
		offset2 = 0;		
	} else {
		offset1 = 0;
		offset2 = 1;			
	}
	
	for(i = 0; i < inputGraph.row - 1; i++) {
		if(isBetween((int)inputGraph.dataArray[inputGraph.col*i + offset1], (int) inputGraph.dataArray[inputGraph.col*(i+1) + offset1], coord, tolerance)) {
			dummy.value = interpolate((int) inputGraph.dataArray[inputGraph.col*i + offset1], (int) inputGraph.dataArray[inputGraph.col*i + offset2], (int) inputGraph.dataArray[inputGraph.col*(i+1) + offset1], (int) inputGraph.dataArray[inputGraph.col*(i+1) + offset2], coord);
			dummy.index = i; // between point i and point i + 1;
			setOfMatchingCoords.push_back(dummy);
			TRACE_PURP_BOLD("Check Point: %d, First Point: %d, Second Point: %d\t", coord, (int) inputGraph.dataArray[inputGraph.col*i + offset1], (int) inputGraph.dataArray[inputGraph.col*(i+1) + offset1]);
			TRACE_PURP_BOLD("Added coordinate index: %d, Value: %d", dummy.index, dummy.value);TRACE("\n");
		}
	}
	return setOfMatchingCoords;
}

int bound(int input, int max) {
	return bound(input, max, -max);
}


int bound(int input, int max, int min) {
	if(input < min) return min;
	if(input > max) return max;
	return input;
}

int max(int a, int b) {
	if(a > b){ return a;} else {return b;}
}

float max(float a, float b) {
	if(a > b){ return a;} else {return b;}
}

double max(double a, double b) {
	if(a > b){ return a;} else {return b;}
}

vector<matchingCoordHighOrder> findMatchingCoords(dataArrayWrapper inputGraph, int coord, int checkCol) {
	vector<matchingCoordHighOrder> result;
	matchingCoordHighOrder tmp;
	int i, j;

	if(inputGraph.col < 2 || checkCol >= inputGraph.col || inputGraph.col > MAX_MATCHING_COORD_ORDER) return result;
	
	for(i = 0; i < inputGraph.row - 1; i++) {
		if(isBetween((int)inputGraph.dataArray[inputGraph.col*i + checkCol], (int) inputGraph.dataArray[inputGraph.col*(i+1) + checkCol], coord, 0)) {
			tmp.index = i;
			for(j = 0; j < inputGraph.col; j++) {
				if(j != checkCol) {
					tmp.value[j] = interpolate((int) inputGraph.dataArray[inputGraph.col*i + checkCol], (int) inputGraph.dataArray[inputGraph.col*i + j], (int) inputGraph.dataArray[inputGraph.col*(i+1) + checkCol], (int) inputGraph.dataArray[inputGraph.col*(i+1) + j], coord);
				TRACE_PURP_BOLD("Check Point: %d, First Point: %d, Second Point: %d\t", coord, (int) inputGraph.dataArray[inputGraph.col*i + checkCol], (int) inputGraph.dataArray[inputGraph.col*(i+1) + checkCol]);				
				TRACE_PURP_BOLD("Added coordinate index: %d, Value: %d", tmp.index, tmp.value[j]);TRACE("\n");
				} else {
					tmp.value[j] = coord;
				}
			}	
			result.push_back(tmp);
		}
	}
	return result;
}



