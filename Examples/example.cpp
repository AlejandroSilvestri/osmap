/*
Serialization example of ficticious MapPoints.
*/

#include <iostream>
#include <fstream>
//#include <string>
#include "../osmap.h"
#include <cstdlib>

using namespace std;
using namespace cv;

#define N_MAPPOINTS 100	// Number of fake MapPoints in map
#define N_KEYFRAMES 11	// Number of fake KeyFrames in map
#define N_FEATURES 19	// Number of fake features in each KeyFrame

/*
 * main arguments:
 *
 * 1º argument: file name.  Defaults to exampleDummyMap.
 * 2º argument: options, a number.  Defaults to 0.
 */
int main(int argc, char **argv){

  // Arguments
  string filename;
  int options = 0;
  if(argc == 1)
	  filename = "exampleDummyMap";
  else{
	  filename = argv[1];
	  if(argc>2)
		options = stoi(argv[2]);
  }

  Map map;

  // Generate MapPoints
  MapPoint *pMP;
  int dataInt[] = {0,1,2,3,4,5,6,7};
  Mat descriptorModelo = Mat(1, 8, CV_32S, dataInt);
  for(int i=0; i<N_MAPPOINTS; i++){
    pMP = new MapPoint();
    pMP->mnId = i;
    pMP->mnVisible = 200 + i;
    pMP->mnFound = 100 + i;

    pMP->mWorldPos = Mat(3, 1, CV_32F);
    float *data = (float*)pMP->mWorldPos.data;
    data[0] = 300+i;
    data[1] = 0;
    data[2] = -500+i;

    pMP->mDescriptor = descriptorModelo;
    map.mspMapPoints.insert(pMP);
  }

  // Generate KeyFrames
  KeyFrame *pKF;
  Mat K = Mat::eye(3,3,CV_32F);
  auto itMP = map.mspMapPoints.begin();
  for(int i=0; i<N_KEYFRAMES; i++){
	  pKF = new KeyFrame();
	  pKF->N = N_FEATURES;
	  pKF->mK = K;
	  pKF->mTcw = Mat::eye(4,4,CV_32F);
	  pKF->mnId = i;

	  // Generate Features
	  pKF->mvKeysUn.resize(N_FEATURES);
	  pKF->mvpMapPoints.resize(N_FEATURES);
	  for(int j=0; j<N_FEATURES; j++){
		  pKF->mvKeysUn.push_back(KeyPoint(i, j, 0, 1, 0, 0));
		  pKF->mDescriptors.push_back(descriptorModelo);
		  pKF->mvpMapPoints.push_back(*itMP);
		  if(++itMP == map.mspMapPoints.end())
			  itMP = map.mspMapPoints.begin();
	  }
	  map.mspKeyFrames.insert(pKF);
  }
  KeyFrame::nNextId = N_KEYFRAMES;

  // Save map
  Osmap osmap(map);	// Create singleton
  osmap.options = options;	// Set saving options
  osmap.mapSave(filename);	// Save the map to files

  // Clear and load map
  map.mspMapPoints.clear();
  map.mspKeyFrames.clear();
  osmap.mapLoad(filename);

  // Show loaded MapPoints
  cout << "MapPoints retrieved: " << endl;
  for(auto pMP : map.mspMapPoints)
    cout << "mnId: " << pMP->mnId << ", mnVisible: " << pMP->mnVisible << ", mnFound: " << pMP->mnFound << ", mDescriptor: " << pMP->mDescriptor << ", mWorldPos: " << pMP->mWorldPos << endl;

  // Show loaded KeyFrames
  cout << "KeyFrames retrieved: " << endl;
  for(auto pKF : map.mspKeyFrames)
    cout << "mnId: " << pKF->mnId << ", N: " << pKF->N << "\nmTcw:\n" << pKF->mTcw << "\n\nmDescriptors:\n" << pKF->mDescriptors << endl << endl;

  cout << endl << "Closing example.cpp" << endl;
  return 0;
}
