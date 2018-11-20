/*
Serialization example of ficticious MapPoints.
*/

#include <iostream>
#include <fstream>
#include "../osmap.h"

using namespace std;
using namespace cv;


int main(){
  Map map;

  // Generate MapPoints
  MapPoint *pMP;
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

    int dataInt[] = {0,1,2,3,4,5,6,7};
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
  //unsigned int nMappoints;
  SerializedMappointArray serializedMappointArray;
  cout << "Mappoints serialized: "
  	  << "MapPoints count: " << osmap.serialize(map.mspMapPoints, serializedMappointArray) << endl;
  if (!serializedMappointArray.SerializeToOstream(&fileOutput)) {/*error*/};
  fileOutput.close();

  cout << "Serialization done!" << endl;

  // Load MapPoints
  map.mspMapPoints.clear();
  ifstream fileInput;
  // MapPoints
  filename = "mappointExample.mappoints";
  fileInput.open(filename, ofstream::binary);
  //SerializedMappointArray serializedMappointArray;
  serializedMappointArray.ParseFromIstream(&fileInput);
  cout << "Mappoints deserialized: "
    << osmap.deserialize(serializedMappointArray, map.mspMapPoints);
  fileInput.close();




  // Imprimir vector
  cout << "MapPoints retrieved:" << endl;
  for(auto pMP : map.mspMapPoints)
    cout << "mnId: " << pMP->mnId << ", mWorldPos: " << pMP->mWorldPos << ", mnVisible: " << pMP->mnVisible << ", mnFound: " << pMP->mnFound << ", mDescriptor: " << pMP->mDescriptor << endl;


  return 0;
}
//}}} // namespaces Osmap & std & cv
