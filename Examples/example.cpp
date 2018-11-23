/*
Serialization example of ficticious MapPoints.
*/

#include <iostream>
#include <fstream>
#include "../osmap.h"

using namespace std;
using namespace cv;

unsigned int KeyFrame::nNextId = 0;	// Compiler requires it, this example doesn't use it.

int main(){
  Map map;

  // Generate MapPoints
  MapPoint *pMP;
  int dataInt[] = {0,1,2,3,4,5,6,7};
  for(int i=0; i<100; i++){
    pMP = new MapPoint();
    pMP->mnId = i;
    pMP->mnVisible = 200 + i;
    pMP->mnFound = 100 + i;

    pMP->mWorldPos = Mat(3, 1, CV_32F);
    float *data = (float*)pMP->mWorldPos.data;
    data[0] = 300+i;
    data[1] = 0;
    data[2] = -500+i;

    pMP->mDescriptor = Mat(1, 8, CV_32S, dataInt);
    map.mspMapPoints.insert(pMP);
  }


  // Initial
  Osmap osmap(map);

  // Save MapPoints

  // Copied code from mapSave()
  // Other files
  ofstream fileOutput;
  string filename;

  // MapPoints
  filename = "mappointExample.mappoints";
  fileOutput.open(filename, ofstream::binary);
  SerializedMappointArray serializedMappointArray;
  cout << "Options: " << osmap.options << endl
	<< "Mappoints serialized: " << osmap.serialize(map.mspMapPoints, serializedMappointArray) << endl;
  if (!serializedMappointArray.SerializeToOstream(&fileOutput)) {/*error*/};
  fileOutput.close();

  cout << "Serialization done!" << endl << endl;

  // Load MapPoints
  map.mspMapPoints.clear();
  ifstream fileInput;
  // MapPoints
  filename = "mappointExample.mappoints";
  fileInput.open(filename, ofstream::binary);
  serializedMappointArray.ParseFromIstream(&fileInput);
  cout << "Mappoints deserialized: "
    << osmap.deserialize(serializedMappointArray, map.mspMapPoints) << endl << endl;
  fileInput.close();


  // Imprimir vector
  cout << "MapPoints retrieved: " << endl;
  for(auto pMP : map.mspMapPoints)
    cout << "mnId: " << pMP->mnId << ", mnVisible: " << pMP->mnVisible << ", mnFound: " << pMP->mnFound << ", mDescriptor: " << pMP->mDescriptor << ", mWorldPos: " << pMP->mWorldPos << endl;

  return 0;
}
