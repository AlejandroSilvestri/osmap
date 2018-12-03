/*
 * default constructors.h
 *
 *  Created on: 2 dic. 2018
 *      Author: alejandro
 */

/**
 * Default constructors of MapPoint y KeyFrame, needed by Osmap for map serialization.
 *
 * To attach Osmap to ORB-SLAM2, these changes are needed in these ORB-SLAM2 files:
 *
 * MapPoint.h
 *
 * 1) add this code after the last #include
 *
  	#include "osmap.h"
 *
 *
 *
 * 2) add this code right after "class MapPoint{" :
 *
  	MapPoint();
	friend class Osmap;
 *
 *
 *
 *
 * KeyFrame.h
 *
 * 1) add this code after the last #include
 *
  	#include "osmap.h"
 *
 *
 *
 * 2) add this code in the next line right after "class KeyFrame{" :
 *
  	KeyFrame();
	friend class Osmap;
 *
 *
 * Map.h
 *
 * 1) add this code after the last #include
 *
  	#include "osmap.h"
 *
 *
 *
 * 2) add this code in the next line right after "class Map{" :
 *
	friend class Osmap;
 *
 */

#ifndef DEFAULT_CONSTRUCTORS_H_
#define DEFAULT_CONSTRUCTORS_H_

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "osmap.h"

MapPoint::MapPoint():
mnFirstKFid(0), nObs(0), mnTrackReferenceForFrame(0),
mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(NULL), mnVisible(1), mnFound(1), mbBad(false),
mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0)), mpMap(*System->mpMap))
{}

KeyFrame::KeyFrame():
	// Public
	mnFrameId(0),  mTimeStamp(0.0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
	mfGridElementWidthInv(Frame::mfGridElementWidthInv),
	mfGridElementHeightInv(Frame::mfGridElementHeightInv),

	mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
	mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),

	fx(Frame::fx), fy(Frame::fy), cx(Frame::cx), cy(Frame::cy), invfx(Frame::invfx), invfy(Frame::invfy),
	/*mbf(0.0), mb(0.0), mThDepth(0.0),*/	// Valores no usados en monocular, que pasan por varios constructores.
	N(0), mnScaleLevels(Osmap::system->mpTracker->mCurrentFrame.mnScaleLevels),
	mfScaleFactor(Osmap::system->mpTracker->mCurrentFrame.mfScaleFactor),
	mfLogScaleFactor(Osmap::system->mpTracker->mCurrentFrame.mfLogScaleFactor),
	mvScaleFactors(Osmap::system->mpTracker->mCurrentFrame.mvScaleFactors),
	mvLevelSigma2(Osmap::system->mpTracker->mCurrentFrame.mvLevelSigma2),
	mvInvLevelSigma2(Osmap::system->mpTracker->mCurrentFrame.mvInvLevelSigma2),
	mnMinX(Frame::mnMinX), mnMinY(Frame::mnMinY), mnMaxX(Frame::mnMaxX), mnMaxY(Frame::mnMaxY),
	mK(Osmap::system->mpTracker->mCurrentFrame.mK),

	// Protected
	mpKeyFrameDB(Osmap::system->mpKeyFrameDatabase),
	mpORBvocabulary(Osmap::system->mpVocabulary),
	mbFirstConnection(false),
	mpParent(NULL),
	mbBad(false),
	mpMap(Osmap::system->mpMap)
{}





#endif /* DEFAULT_CONSTRUCTORS_H_ */
