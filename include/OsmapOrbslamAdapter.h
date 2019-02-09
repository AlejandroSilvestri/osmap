/*
 * OsmapOrbslamAdapter.h
 *
 * This header is intended to be used when merging osmap with orbslam2, instead of dummymap.h.
 * This header grabs orbslam2 class headers (map, mappoint keyframe, etc.).
 * dummymap.h is used for map serialization without orbslam2.  Let say you want to open a map and count its mappoints.
 */

#ifndef INCLUDE_OSMAPORBSLAMADAPTER_H_
#define INCLUDE_OSMAPORBSLAMADAPTER_H_

#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "System.h"
#include "Frame.h"
#include "System.h"
#include "Tracking.h"

namespace ORB_SLAM2{

/**
 * Default constructors with const properties initialized
 */

MapPoint::MapPoint(Osmap *osmap):
	mnFirstKFid(0), nObs(0), mnTrackReferenceForFrame(0),
	mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
	mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(NULL), mnVisible(1), mnFound(1), mbBad(false),
	mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0),
	mpMap(&osmap.map)
{};

KeyFrame::KeyFrame(Osmap *osmap):
	// Públicas
    mnFrameId(0),  mTimeStamp(0.0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(Frame::mfGridElementWidthInv),
    mfGridElementHeightInv(Frame::mfGridElementHeightInv),

    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),

    fx(Frame::fx), fy(Frame::fy), cx(Frame::cx), cy(Frame::cy), invfx(Frame::invfx), invfy(Frame::invfy),
    N(0), mnScaleLevels(osmap.currentFrame.mnScaleLevels),
    mfScaleFactor(osmap.currentFrame.mfScaleFactor),
    mfLogScaleFactor(osmap.currentFrame.mfLogScaleFactor),
    mvScaleFactors(osmap.currentFrame.mvScaleFactors),
    mvLevelSigma2(osmap.currentFrame.mvLevelSigma2),
    mvInvLevelSigma2(osmap.currentFrame.mvInvLevelSigma2),
    mnMinX(Frame::mnMinX), mnMinY(Frame::mnMinY), mnMaxX(Frame::mnMaxX), mnMaxY(Frame::mnMaxY),

	// Protegidas:
    mpKeyFrameDB(&osmap.keyFrameDatabase),
    mpORBvocabulary(osmap.system.mpVocabulary),
    mbFirstConnection(false),
	mpParent(NULL),
	mbBad(false),
	mpMap(&osmap.map)
{};
}	// namespace ORB_SLAM2



#endif /* INCLUDE_OSMAPORBSLAMADAPTER_H_ */
