/*
Serialization example of ficticious MapPoints.
*/

#include <Osmap.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

#define N_MAPPOINTS 100	// Number of fake MapPoints in map
#define N_KEYFRAMES 11	// Number of fake KeyFrames in map
#define N_FEATURES 19	// Number of fake features in each KeyFrame

unsigned int MapPoint::nNextId = 0;
unsigned int KeyFrame::nNextId = 0;


void generateDummyMap(System& system){
	  Map *pMap = new Map;

	  // Generate MapPoints
	  MapPoint *pMP;
	  int dataInt[] = {0,1,2,3,4,5,6,7};
	  Mat descriptorModelo = Mat(1, 32, CV_8UC1, dataInt);
	  for(int i=0; i<N_MAPPOINTS; i++){
	    pMP = new MapPoint(NULL);
	    pMP->mnId = i;
	    pMP->mnVisible = 200 + i;
	    pMP->mnFound = 100 + i;

	    pMP->mWorldPos = Mat(3, 1, CV_32F);
	    float *data = (float*)pMP->mWorldPos.data;
	    data[0] = 300+i;
	    data[1] = 0;
	    data[2] = -500+i;

	    pMP->mDescriptor = descriptorModelo;
	    pMap->mspMapPoints.insert(pMP);
	  }

	  // Generate KeyFrames
	  KeyFrame *pKF, *pKF2;
	  auto itMP = pMap->mspMapPoints.begin();
	  for(int i=0; i<N_KEYFRAMES; i++){
		  pKF = new KeyFrame(NULL);
		  if(!i) pKF2 = pKF;
		  pKF->N = N_FEATURES;
		  pKF->mK  = Mat::eye(3,3,CV_32F);
		  pKF->Tcw = Mat::eye(4,4,CV_32F);
		  pKF->mnId = i;

		  // Generate Features
		  pKF->mvKeysUn.resize(N_FEATURES);
		  pKF->mvpMapPoints.resize(N_FEATURES);
		  for(int j=0; j<N_FEATURES; j++){
			  pKF->mvKeysUn.push_back(KeyPoint(i, j, 0, 1, 0, 0));
			  pKF->mDescriptors.push_back(descriptorModelo);
			  pKF->mvpMapPoints.push_back(*itMP);
			  if(++itMP == pMap->mspMapPoints.end())
				  itMP = pMap->mspMapPoints.begin();
		  }
		  pMap->mspKeyFrames.insert(pKF);
	  }

	  // Add a loop closure between 1º and last keframes
	  pKF ->mspLoopEdges.insert(pKF2);
	  pKF2->mspLoopEdges.insert(pKF);


	  KeyFrame::nNextId = N_KEYFRAMES;
	  pMap->mnMaxKFid = N_KEYFRAMES - 1;

	  system.mpMap = pMap;
	  system.mpKeyFrameDatabase = new KeyFrameDatabase;
}

/*
 * main arguments:
 *
 * 1º argument: file name.  Defaults to exampleDummyMap.
 * 2º argument: options, a number.  Defaults to 0.
 */
int main(int argc, char **argv){


  // Creates a fake map with system, mappoints, keyframes and everything to serialize and deserialize it
  System system;
  generateDummyMap(system);

  // Save map
  Osmap osmap(system);

  // Arguments
  string filename;
  if(argc == 1)
	  filename = "exampleDummyMap";
  else
	  filename = argv[1];

  if(argc>2)
	  osmap.options = stoi(argv[2]);
  else{
	  osmap.options.set(Osmap::NO_SET_BAD);	// This dummy map doesn't build graphs and connections, this options skips this anomally detection after loading.
	  osmap.options.set(Osmap::NO_DEPURATION);	// Avoids map depuration before save.  This dummy map doesn't need it.
  }

  // Save the map to files
  osmap.mapSave(filename);

  // Turn verbose on before loading to see a lot of debugging data in your console
  //osmap.verbose = true;

  // Load map.  mapLoad clears previous map.  Sets NO_SET_BAD flag, needed on this dummy map.
  osmap.mapLoad(filename + ".yaml", true);

  // Show loaded MapPoints
  cout << "MapPoints retrieved: " << endl;
  for(auto pMP : system.mpMap->mspMapPoints)
    cout << "mnId: " << pMP->mnId << ", mnVisible: " << pMP->mnVisible << ", mnFound: " << pMP->mnFound << ", mDescriptor: " << pMP->mDescriptor << ", mWorldPos: " << pMP->mWorldPos << endl;

  // Show loaded KeyFrames
  cout << "KeyFrames retrieved: " << endl;
  for(auto pKF : system.mpMap->mspKeyFrames){
    cout << "mnId: " << pKF->mnId << ", N: " << pKF->N << "\nmTcw:\n" << pKF->Tcw << "\n\nmDescriptors:\n" << pKF->mDescriptors << endl;
    if(pKF->mspLoopEdges.size())
    	cout << "Loop edge with keyframe " << (*pKF->mspLoopEdges.begin())->mnId << endl;
    cout << endl;
  }

  cout << endl << "Closing example.cpp" << endl;
  return 0;
}
