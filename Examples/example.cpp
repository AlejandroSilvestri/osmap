/*
Serialization example of ficticious MapPoints.
*/

#include <iostream>
#include <fstream>
#include <opencv>
#include "osmap.h"

class KeyPoint{
  struct {float x, y;} pt;
  float angle, octave;
}

class MapPoint{
  unsigned int mnId;
  Mat mWorldPos;
  float mnVisible;
  float mnFound;
  Mat mDescriptor;
  void print(){
    cout << "mnId: " << mnId << ", mWorldPos: " << mWorldPos << ", mnVisible: " << mnVisible << ", mnFound: " << mnFound << ", mDescriptor: " << mDescriptor << endl;
  }
}


class KeyFrame{
  unsigned int mnId;
  Mat mTcw;
  Mat mDescriptors;
  unsigned int N;
  vector<KeyPoint> mvKeysUn;
  vector<MapPoint*> mvpMapPoints;
  void print(){
    cout << "mnId: " << mnId << ", pose: " << mTcw << ", N: " << N << endl;
  }
}

class Map{
  set<MapPoint*> mspMapPoints;
  set<KeyFrame*> mspKeyFrames;
} map;

using namespace std{ using Omap{ using cv{
void main(){
  // Generate MapPoints
  MapPoint *pMP;
  for(int i=0; i<100; i++){
    pMP = new MapPoint();
    pMP->mnId = i;
    pMP->mnVisible = 200 + i;
    pMP->mnFound = 100 + i;

    pMP->mWorldPos = Mat(3, 1, CV_32F);
    float *data = pMP->mWorldPos.data;
    data[0] = 300+i;
    data[1] = 0;
    data[2] = -500+i;

    int dataInt[] = {0,1,2,3,4,5,6,7};
    pMP->mDescriptor = Mat(1, 8, CV_32S, dataInt);
    mspMapPoints.insert(pMP);
  }


  // Initial
  Osmap osmap(&map);


  // Save MapPoints

  // Copied code from mapSave()
  // Other files
  ofstream file;
  string filename;

  // MapPoints
  filename = "mappointExample.mappoints";
  file.open(filename, ofstream::binary);
  unsigned int nMappoints = SerializedMapPointArray serializedMapPointArray;
  headerFile << "mappointsFile" << filename;
  headerFile << "nMappoints"
    << serialize(map.mspMapPoints.begin(), map.mspMapPoints.end(), serializedMapPointArray);
  if (!serializedMapPointArray.SerializeToOstream(&file)) {/*error*/}
  file.close();

  cout << "Serialization done!" << endl;

  // Load MapPoints
  map.mspMapPoints.clear();

  // MapPoints
  filename = "mappointExample.mappoints";
  file.open(filename, ofstream::binary);
  SerializedMapPointArray serializedMapPointArray;
  serializedMapPointArray.ParseFromIstream(file);
  cout << "Mappoints deserialized: "
    << deserialize(serializedMapPointArray, std::inserter(map.mspMapPoints, map.mspMapPoints.end()));
  if (!serializedMapPointArray.SerializeToOstream(&file)) {/*error*/}
  file.close();




  // Imprimir vector
  cout << "MapPoints retrieved:" << endl;
  for(int i=0; i<vpMP.size(); i++){
    vpMP[i]->print();
  }
}
}}} // namespaces Osmap & std & cv
