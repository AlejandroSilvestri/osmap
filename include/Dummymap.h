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

namespace ORB_SLAM2{

class KeyFrame;
class Map;
class Osmap;

class MapPoint{
public:
  static unsigned int nNextId;
  unsigned int mnId=0;
  bool mbBad = false;
  cv::Mat mWorldPos;
  int mnVisible=0;
  int mnFound=0;
  cv::Mat mDescriptor;
  KeyFrame *mpRefKF = NULL;
  std::map<KeyFrame*,size_t> mObservations;
  Map *mpMap = NULL;

  MapPoint(Osmap*){};
  void AddObservation(KeyFrame*, size_t){}
  void SetBadFlag(){}
  void UpdateNormalAndDepth(){}
};

class KeyFrame{
public:
  static unsigned int nNextId;
  unsigned int mnId=0;
  bool mbBad = false;
  double mTimeStamp = 0;
  cv::Mat Tcw;
  cv::Mat mK;
  cv::Mat mDescriptors;
  int N=0;
  std::vector<cv::KeyPoint> mvKeysUn;
  std::vector<MapPoint*> mvpMapPoints;
  std::set<KeyFrame*> mspLoopEdges;

  bool mbNotErase=false;
  const int mnMinX=0, mnMinY=0, mnMaxX=2, mnMaxY=2, mnGridCols=1, mnGridRows=1;
  const float mfGridElementWidthInv=.5, mfGridElementHeightInv=.5;
  KeyFrame *mpParent = NULL;
  std::vector<std::vector<std::vector<size_t> > > mGrid;
  std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
  std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;

  KeyFrame(Osmap*){};
  void ComputeBoW(){}
  void SetPose(cv::Mat){}
  void UpdateConnections(){}
  void SetBadFlag(){}
  void ChangeParent(KeyFrame *pKF){}
};


class Map{
public:
  std::set<MapPoint*> mspMapPoints;
  std::set<KeyFrame*> mspKeyFrames;
  std::vector<KeyFrame*> mvpKeyFrameOrigins;
  long unsigned int mnMaxKFid;
};

class KeyFrameDatabase{
public:
	void add(KeyFrame *pKF){}
	void clear(){}
};

class Frame{};

class Tracker{
public:
	Frame mCurrentFrame;
};

class System{
public:
	Map *mpMap;
	KeyFrameDatabase *mpKeyFrameDatabase;
	Tracker *mpTracker;
};

}	// namespace ORB_SLAM2

#endif /* DUMMYMAP_H_ */
