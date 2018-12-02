/*
 * dummymap.h
 *
 *  Created on: 17 nov. 2018
 *      Author: alejandro
 */

#ifndef DUMMYMAP_H_
#define DUMMYMAP_H_

#include <set>
#include <map>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

class KeyFrame;

class MapPoint{
public:
  static unsigned int nNextId;
  unsigned int mnId;
  bool mbBad = false;
  Mat mWorldPos;
  float mnVisible;
  float mnFound;
  Mat mDescriptor;

  KeyFrame *mpRefKF;
  map<KeyFrame*,size_t> MapPoint::mObservations;
  void AddObservation(KeyFrame*, int){}
  void SetBadFlag(){}
  void UpdateNormalAndDepth(){}
};

class KeyFrame{
public:
  static unsigned int nNextId;
  unsigned int mnId;
  bool mbBad = false;
  double mTimeStamp = 0;
  Mat mTcw;
  Mat mK;
  Mat mDescriptors;
  unsigned int N;
  vector<KeyPoint> mvKeysUn;
  vector<MapPoint*> mvpMapPoints;
  set<KeyFrame*> mspLoopEdges;

  bool mbNotErase;
  const int mnMinX, mnMinY, mnGridCols, mnGridRows;
  const float mfGridElementWidthInv, mfGridElementHeightInv;
  KeyFrame *mpParent = NULL;
  vector<vector<vector<size_t> > > mGrid;
  map<KeyFrame*, int> mConnectedKeyFrameWeights;
  vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
  void ComputeBoW(){}
  void SetPose(Mat);
  void UpdateConnections(){}
  void SetBadFlag(){}
  void ChangeParent(KeyFrame *pKF){}
};
#define FRAME_GRID_COLS 10
#define FRAME_GRID_ROWS 10


class Map{
public:
  set<MapPoint*> mspMapPoints;
  set<KeyFrame*> mspKeyFrames;
  vector<KeyFrame*> mvpKeyFrameOrigins;
  long unsigned int mnMaxKFid;
};

class KeyFrameDatabase{
public:
	void add(KeyFrame *pKF){}
	void clear(){}
};


class System{
public:
	Map *mpMap;
	KeyFrameDatabase *mpKeyFrameDatabase;
};
#endif /* DUMMYMAP_H_ */
