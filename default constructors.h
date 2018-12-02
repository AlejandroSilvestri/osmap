/*
 * default constructors.h
 *
 *  Created on: 2 dic. 2018
 *      Author: alejandro
 */

#ifndef DEFAULT_CONSTRUCTORS_H_
#define DEFAULT_CONSTRUCTORS_H_

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

MapPoint::MapPoint():
mnFirstKFid(0), nObs(0), mnTrackReferenceForFrame(0),
mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(NULL), mnVisible(1), mnFound(1), mbBad(false),
mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0)))
{
	//mpMap(Sistema->mpMap)
}

KeyFrame::KeyFrame():
	// Public
    mnFrameId(0),  mTimeStamp(0.0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(Frame::mfGridElementWidthInv),
    mfGridElementHeightInv(Frame::mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(Frame::fx), fy(Frame::fy), cx(Frame::cx), cy(Frame::cy), invfx(Frame::invfx), invfy(Frame::invfy),
    N(0), mnScaleLevels(Sistema->mpTracker->mCurrentFrame.mnScaleLevels),
    mfScaleFactor(Sistema->mpTracker->mCurrentFrame.mfScaleFactor),
    mfLogScaleFactor(Sistema->mpTracker->mCurrentFrame.mfLogScaleFactor),
    mvScaleFactors(Sistema->mpTracker->mCurrentFrame.mvScaleFactors),
    mvLevelSigma2(Sistema->mpTracker->mCurrentFrame.mvLevelSigma2),
    mvInvLevelSigma2(Sistema->mpTracker->mCurrentFrame.mvInvLevelSigma2),
    mnMinX(Frame::mnMinX), mnMinY(Frame::mnMinY), mnMaxX(Frame::mnMaxX), mnMaxY(Frame::mnMaxY),
    mK(Sistema->mpTracker->mCurrentFrame.mK),
    // Protected
    mpKeyFrameDB(Sistema->mpKeyFrameDatabase),
    mpORBvocabulary(Sistema->mpVocabulary),
	mpMap(Sistema->mpMap),
    mbFirstConnection(false),
	mpParent(NULL),
	mbBad(false)
{

}





#endif /* DEFAULT_CONSTRUCTORS_H_ */
