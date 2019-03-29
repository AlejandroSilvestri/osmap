/**
* This file is part of OSMAP.
*
* Copyright (C) 2018-2019 Alejandro Silvestri <alejandrosilvestri at gmail>
* For more information see <https://github.com/AlejandroSilvestri/osmap>
*
* OSMAP is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OSMAP is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OSMAP. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fstream>
#include <iostream>
#include <assert.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "Osmap.h"

// Option check macro
#define OPTION(OP) if(options[OP]) headerFile << #OP;

using namespace std;
using namespace cv;

namespace ORB_SLAM2{

Osmap::Osmap(System &_system):
	map(static_cast<OsmapMap&>(*_system.mpMap)),
	keyFrameDatabase(*_system.mpKeyFrameDatabase),
	system(_system),
	currentFrame(_system.mpTracker->mCurrentFrame)
{
#ifndef OSMAP_DUMMY_MAP

	/* Every new MapPoint require a dummy pRefKF in its constructor, copying the following parameters:
	 *
	 * - mnFirstKFid(pRefKF->mnId)
	 * - mnFirstFrame(pRefKF->mnFrameId)
	 * - mpRefKF(pRefKF)
	 *
	 * A fake keyframe construction requires a Frame with
	 *
	 *  - mTcw (must be provided, error if not)
	 *  - Grid (already exists)
	 */
	Frame dummyFrame;
	dummyFrame.mTcw = Mat::eye(4, 4, CV_32F);
	pRefKF = new KeyFrame(dummyFrame, &map, &keyFrameDatabase);

#endif
};


void Osmap::mapSave(const string givenFilename, bool pauseThreads){
	// Stop threads
	if(pauseThreads){
		system.mpLocalMapper->RequestStop();
		while(!system.mpLocalMapper->isStopped()) usleep(1000);
	}

	// Strip out .yaml if present
	string baseFilename, filename, pathDirectory;
	parsePath(givenFilename, &filename, &pathDirectory);
	if(pathDirectory != "")
		chdir(pathDirectory.c_str());

	int length = filename.length();
	if(length>5 && filename.substr(length-5) == ".yaml")
	  baseFilename = filename.substr(length-5);
	else
	  baseFilename = filename;

	// Map depuration
	if(!options[NO_DEPURATION])
		depurate();


	// Actual saving
	filename = baseFilename + ".yaml";

	// Open YAML file for write, it will be the last file to close.
	// FileStorage https://docs.opencv.org/3.1.0/da/d56/classcv_1_1FileStorage.html
	FileStorage headerFile(filename, FileStorage::WRITE);
	if(!headerFile.isOpened()){
	// Is this necessary?
	 cerr << "Couldn't create file " << baseFilename << ".yaml, map not saved." << endl;
	 return;
	}

	// MapPoints
	if(!options[NO_MAPPOINTS_FILE]){
	  // Order mappoints by mnId
	  getMapPointsFromMap();

	  // New file
	  filename = baseFilename + ".mappoints";

	  // Serialize
	  cout << "Saving " << filename << endl;
	  headerFile << "mappointsFile" << filename;
	  headerFile << "nMappoints" << MapPointsSave(filename);
	}

	// K: grab camera calibration matrices.  Will be saved to yaml file later.
	if(!options[K_IN_KEYFRAME]) getVectorKFromKeyframes();

	// KeyFrames
	if(!options[NO_KEYFRAMES_FILE]){
	  getKeyFramesFromMap();

	  // New file
	  filename = baseFilename + ".keyframes";

	  // Serialize
	  cout << "Saving " << filename << endl;
	  headerFile << "keyframesFile" << filename;
	  headerFile << "nKeyframes" << KeyFramesSave(filename);
	}

	// Features
	if(!options[NO_FEATURES_FILE]){
	  filename = baseFilename + ".features";
	  cout << "Saving " << filename << endl;
	  headerFile << "featuresFile" << filename;
	  headerFile << "nFeatures" << featuresSave(filename);
	}


	// Save options, as an int
	headerFile << "Options" << (int) options.to_ulong();
	// Options
	if(options.any()){
	headerFile << "Options descriptions" << "[:";
	OPTION(NO_LOOPS)
	OPTION(NO_FEATURES_DESCRIPTORS)
	OPTION(K_IN_KEYFRAME)
	OPTION(ONLY_MAPPOINTS_FEATURES)
	OPTION(FEATURES_FILE_DELIMITED)
	OPTION(FEATURES_FILE_NOT_DELIMITED)
	OPTION(NO_MAPPOINTS_FILE)
	OPTION(NO_KEYFRAMES_FILE)
	OPTION(NO_FEATURES_FILE)
	headerFile << "]";
	}


	// K: camera calibration matrices, save to yaml at the end of file.
	if(!options[K_IN_KEYFRAME]){
	// Save K matrices in header file yaml
	headerFile << "cameraMatrices" << "[";
	for(auto pK:vectorK)
	   headerFile << "{:"  << "fx" << pK->at<float>(0,0) << "fy" << pK->at<float>(1,1) << "cx" << pK->at<float>(0,2) << "cy" << pK->at<float>(1,2) << "}";
	headerFile << "]";
	}

	// Save yaml file
	headerFile.release();

	// Clear temporary vectors
	clearVectors();

	if(pauseThreads)
	  system.mpViewer->Release();
}

void Osmap::mapLoad(string yamlFilename, bool pauseThreads){
	if(pauseThreads){
		system.mpLocalMapper->Release();

		// Limpia el mapa de todos los singletons
		system.mpTracker->Reset();
		// En este punto el sistema está reseteado.

		// Espera a que se detenga LocalMapping y  Viewer
		system.mpLocalMapper->RequestStop();
		system.mpViewer	    ->RequestStop();

		while(!system.mpLocalMapper->isStopped()) usleep(1000);
		while(!system.mpViewer     ->isStopped()) usleep(1000);
	}

	string filename;
	int intOptions;

	// Open YAML
	cv::FileStorage headerFile(yamlFilename, cv::FileStorage::READ);

	// Options
	headerFile["Options"] >> intOptions;
	options = intOptions;

	// K
	if(!options[K_IN_KEYFRAME]){
		vectorK.clear();
		FileNode cameraMatrices = headerFile["cameraMatrices"];
		FileNodeIterator it = cameraMatrices.begin(), it_end = cameraMatrices.end();
		for( ; it != it_end; ++it){
			Mat *k = new Mat();
			*k = Mat::eye(3,3,CV_32F);
			k->at<float>(0,0) = (*it)["fx"];
			k->at<float>(1,1) = (*it)["fy"];
			k->at<float>(0,2) = (*it)["cx"];
			k->at<float>(1,2) = (*it)["cy"];
			vectorK.push_back(k);
		}
	}


	// Change directory
	string pathDirectory;
	parsePath(yamlFilename, NULL, &pathDirectory);
	if(pathDirectory != "")
		chdir(pathDirectory.c_str());


	// MapPoints
	vectorMapPoints.clear();
	if(!options[NO_MAPPOINTS_FILE]){
		headerFile["mappointsFile"] >> filename;
		MapPointsLoad(filename);
	}


	// KeyFrames
	vectorKeyFrames.clear();
	if(!options[NO_KEYFRAMES_FILE]){
		headerFile["keyframesFile"] >> filename;
		KeyFramesLoad(filename);
	}

	// Features
	if(!options[NO_FEATURES_FILE]){
		headerFile["featuresFile"] >> filename;
		cout << "Loading features from " << filename << " ..." << endl;
		featuresLoad(filename);
	}

	// Close yaml file
	headerFile.release();

	// Rebuild
	rebuild();

	// Copy to map
	setMapPointsToMap();
	setKeyFramesToMap();

	// Release temporary vectors
	clearVectors();

#ifndef OSMAP_DUMMY_MAP
// Lost state, the system must relocalize itself in the just loaded map.
	system.mpTracker->mState = ORB_SLAM2::Tracking::LOST;
#endif

	if(pauseThreads){
		// Resume threads

		// Reactivate viewer.  Do not reactivate localMapper because the system resumes in "only tracking" mode immediatly after loading.
		system.mpViewer->Release();

		// Tracking do this when going to LOST state.
		// Involed after viewer.Release() because of mutex.
		system.mpFrameDrawer->Update(system.mpTracker);
	}
}

int Osmap::MapPointsSave(string filename){
	ofstream file;
	file.open(filename, std::ofstream::binary);

	// Serialize
	SerializedMappointArray serializedMappointArray;
	int nMP = serialize(vectorMapPoints, serializedMappointArray);

	// Closing
	if (!serializedMappointArray.SerializeToOstream(&file))
		// Signals the error
		nMP = -1;
	file.close();

	return nMP;
}

int Osmap::MapPointsLoad(string filename){
	ifstream file;
	file.open(filename, ifstream::binary);

	SerializedMappointArray serializedMappointArray;
	serializedMappointArray.ParseFromIstream(&file);
	int nMP = deserialize(serializedMappointArray, vectorMapPoints);
	cout << "Mappoints loaded: " << nMP << endl;

	file.close();
	return nMP;
}

int Osmap::KeyFramesSave(string filename){
	ofstream file;
	file.open(filename, std::ofstream::binary);

	// Serialize
	SerializedKeyframeArray serializedKeyFrameArray;
	int nKF = serialize(vectorKeyFrames, serializedKeyFrameArray);

	// Closing
	if (!serializedKeyFrameArray.SerializeToOstream(&file))
		// Signals the error
		nKF = -1;
	file.close();

	return nKF;
}

int Osmap::KeyFramesLoad(string filename){
	ifstream file;
	file.open(filename, ifstream::binary);
#ifndef OSMAP_DUMMY_MAP
	if(!currentFrame.mTcw.dims)	// if map is no initialized, currentFrame has no pose, a pose is needed to create keyframes.
		currentFrame.mTcw = Mat::eye(4, 4, CV_32F);
#endif
	SerializedKeyframeArray serializedKeyFrameArray;
	serializedKeyFrameArray.ParseFromIstream(&file);
	int nKF = deserialize(serializedKeyFrameArray, vectorKeyFrames);
	cout << "Keyframes loaded: "
		<< nKF << endl;
	file.close();
	return nKF;
}

int Osmap::featuresSave(string filename){
	int nFeatures = 0;
	ofstream file;

	file.open(filename, ofstream::binary);
	if(
		options[FEATURES_FILE_DELIMITED] ||
		(!options[FEATURES_FILE_NOT_DELIMITED] && countFeatures() > FEATURES_MESSAGE_LIMIT)
	){
		// Saving with delimited ad hoc file format
		// Loop serializing blocks of no more than FEATURES_MESSAGE_LIMIT features, using Kendon Varda's function

		options.set(FEATURES_FILE_DELIMITED);

		// This Protocol Buffers stream must be deleted before closing file.  It happens automatically at }.
		::google::protobuf::io::OstreamOutputStream protocolbuffersStream(&file);
		vector<OsmapKeyFrame*> vectorBlock;
		vectorBlock.reserve(FEATURES_MESSAGE_LIMIT/30);

		auto it = vectorKeyFrames.begin();
		while(it != vectorKeyFrames.end()){
			unsigned int n = (*it)->N;
			vectorBlock.clear();
			do{
				vectorBlock.push_back(*it);
				++it;
				if(it == vectorKeyFrames.end()) break;
				KeyFrame *KF = *it;
				n += KF->N;
			} while(n <= FEATURES_MESSAGE_LIMIT);

			SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
			nFeatures += serialize(vectorBlock, serializedKeyframeFeaturesArray);
			writeDelimitedTo(serializedKeyframeFeaturesArray, &protocolbuffersStream);
		}
	}else{
		options.set(FEATURES_FILE_NOT_DELIMITED);
		SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
		nFeatures = serialize(vectorKeyFrames, serializedKeyframeFeaturesArray);
		if (!serializedKeyframeFeaturesArray.SerializeToOstream(&file)){
			cerr << "Error while serializing features file without delimitation." << endl;
			nFeatures = -1;
		}
	}
	file.close();

	return nFeatures;
}

int Osmap::featuresLoad(string filename){
	int nFeatures = 0;
	ifstream file;
	file.open(filename, ifstream::binary);
	auto *googleStream = new ::google::protobuf::io::IstreamInputStream(&file);
	SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
	if(options[FEATURES_FILE_DELIMITED]){
		while(true)
			if(readDelimitedFrom(googleStream, &serializedKeyframeFeaturesArray)){
				nFeatures += deserialize(serializedKeyframeFeaturesArray);
				cout << "Features deserialized in loop: "
					 << nFeatures << endl;
			}
			else
				break;
	} else {
		// Not delimited, pure Protocol Buffers
		serializedKeyframeFeaturesArray.ParseFromIstream(&file);
		nFeatures = deserialize(serializedKeyframeFeaturesArray);
	  }
	cout << "Features loaded: " << nFeatures << endl;
	file.close();
	return nFeatures;
}

void Osmap::getMapPointsFromMap(){
	  vectorMapPoints.clear();
	  vectorMapPoints.reserve(map.mspMapPoints.size());
	  std::transform(map.mspMapPoints.begin(), map.mspMapPoints.end(), std::back_inserter(vectorMapPoints), [](MapPoint *pMP)->OsmapMapPoint*{return static_cast<OsmapMapPoint*>(pMP);});
	  sort(vectorMapPoints.begin(), vectorMapPoints.end(), [](const MapPoint* a, const MapPoint* b){return a->mnId < b->mnId;});
}

void Osmap::setMapPointsToMap(){
	map.mspMapPoints.clear();
	copy(vectorMapPoints.begin(), vectorMapPoints.end(), inserter(map.mspMapPoints, map.mspMapPoints.end()));
}

void Osmap::getKeyFramesFromMap(){
	// Order keyframes by mnId
	vectorKeyFrames.clear();
	vectorKeyFrames.reserve(map.mspKeyFrames.size());
	std::transform(map.mspKeyFrames.begin(), map.mspKeyFrames.end(), std::back_inserter(vectorKeyFrames), [](KeyFrame *pKF)->OsmapKeyFrame*{return static_cast<OsmapKeyFrame*>(pKF);});
	sort(vectorKeyFrames.begin(), vectorKeyFrames.end(), [](const KeyFrame *a, const KeyFrame *b){return a->mnId < b->mnId;});
}

void Osmap::setKeyFramesToMap(){
	map.mspKeyFrames.clear();
	copy(vectorKeyFrames.begin(), vectorKeyFrames.end(), inserter(map.mspKeyFrames, map.mspKeyFrames.end()));
}



void Osmap::clearVectors(){
	keyframeid2vectorkIdx.clear();
	vectorKeyFrames.clear();
	vectorMapPoints.clear();
	vectorK.clear();
}

void Osmap::parsePath(const string &path, string *filename, string *pathDirectory){
	size_t pos = path.find_last_of("\\/");
	if(std::string::npos == pos)
		// No directory separator, file is assumed.
		pos = 0;
	else
		// Last directory separator (/) will be in pathDirectory, not in filename.
		pos++;

	if(pathDirectory)
		*pathDirectory = path.substr(0, pos);
	if(filename)
		*filename = path.substr(pos);
	return;
}


void Osmap::depurate(){
	// First erase MapPoint from KeyFrames, and then erase KeyFrames from MapPoints.

	// NULL out bad MapPoints in KeyFrame::mvpMapPoints
	for(auto pKF: map.mspKeyFrames){
		// NULL out bad MapPoints and warns if not in map.  Usually doesn't find anything.
		auto pOKF = static_cast<OsmapKeyFrame*>(pKF);
		auto &pMPs = pOKF->mvpMapPoints;
		for(int i=pMPs.size(); --i>=0;){
			auto pOMP = static_cast<OsmapMapPoint *>(pMPs[i]);

			if(!pOMP) continue;	// Ignore if NULL

			if(pOMP->mbBad){
				// If MapPoint is bad, NULL it in keyframe's observations.
				cerr << "depurate(): Nullifying bad MapPoint " << pOMP->mnId << " in KeyFrame " << pOKF->mnId << endl;
				pMPs[i] = NULL;
			} else if(!map.mspMapPoints.count(pOMP) && !options[NO_APPEND_FOUND_MAPPOINTS]){
				// If MapPoint is not in map, append it to the map
				map.mspMapPoints.insert(pOMP);
				cout << "depurate(): APPEND_FOUND_MAPPOINTS: MapPoint " << pOMP->mnId << " added to map. ";
			}
		}
	}
}

void Osmap::rebuild(){
	/*
	 * On every KeyFrame:
	 * - Builds the map database
	 * - UpdateConnections to rebuild covisibility graph
	 * - MapPoint::AddObservation on each point to rebuild MapPoint:mObservations y MapPoint:mObs
	 */
	cout << "Rebuilding map:" << endl;
	keyFrameDatabase.clear();

	for(auto *pKF : vectorKeyFrames){
		pKF->mbNotErase = !pKF->mspLoopEdges.empty();

		// Build BoW vectors
		pKF->ComputeBoW();

		// Build many pose matrices
		pKF->SetPose(pKF->Tcw);

		/*
		 * Rebuilding grid.
		 * Code from Frame::AssignFeaturesToGrid()
		 */
		std::vector<std::size_t> grid[pKF->mnGridCols][pKF->mnGridRows];
		int nReserve = 0.5f*pKF->N/(pKF->mnGridCols*pKF->mnGridRows);
		for(int i=0; i<pKF->mnGridCols;i++)
			for (int j=0; j<pKF->mnGridRows;j++)
				grid[i][j].reserve(nReserve);

		for(int i=0;i<pKF->N;i++){
			const cv::KeyPoint &kp = pKF->mvKeysUn[i];
			int posX = round((kp.pt.x-pKF->mnMinX)*pKF->mfGridElementWidthInv);
			int posY = round((kp.pt.y-pKF->mnMinY)*pKF->mfGridElementHeightInv);

			//Keypoint's coordinates are undistorted, which could cause to go out of the image
			if(!(posX<0 || posX>=pKF->mnGridCols || posY<0 || posY>=pKF->mnGridRows))
				grid[posX][posY].push_back(i);
		}

		pKF->mGrid.resize(pKF->mnGridCols);
		for(int i=0; i < pKF->mnGridCols;i++){
			pKF->mGrid[i].resize(pKF->mnGridRows);
			for(int j=0; j < pKF->mnGridRows; j++)
				pKF->mGrid[i][j] = grid[i][j];
		}

		// Append keyframe to the database
		keyFrameDatabase.add(pKF);

		// Calling UpdateConnections in mnId order rebuilds the covisibility graph and the spanning tree.
		pKF->UpdateConnections();

		if(!options[NO_SET_BAD])
			// If this keyframe is isolated (and it isn't keyframe zero), erase it.
			if(pKF->mConnectedKeyFrameWeights.empty() && pKF->mnId){
				cerr << "Isolated keyframe " << pKF->mnId << " set bad." << endl;
				pKF->SetBadFlag();
			}

		// Rebuilds MapPoints obvervations
		size_t n = pKF->mvpMapPoints.size();
		for(size_t i=0; i<n; i++){
			MapPoint *pMP = pKF->mvpMapPoints[i];
			if (pMP)
				pMP->AddObservation(pKF, i);
		}
	}

	// Last KeyFrame's id
	map.mnMaxKFid = vectorKeyFrames.back()->mnId;

	// Next KeyFrame id
	KeyFrame::nNextId = map.mnMaxKFid + 1;

	/*
	 * Check and fix the spanning tree created with UpdateConnections.
	 * Rebuilds the spanning tree asigning a mpParent to every orphan KeyFrame without, except that with id 0.
 	 * It ends when every KeyFrame has a parent.
	 */

	// mvpKeyFrameOrigins should be empty at this point, and must contain only one element, the first keyframe.
	map.mvpKeyFrameOrigins.clear();
	map.mvpKeyFrameOrigins.push_back(*vectorKeyFrames.begin());

	// Number of parents assigned in each iteration and in total.  Usually 0.
	int nParents = -1, nParentsTotal = 0;
	while(nParents){
		nParents = 0;
		for(auto pKF: vectorKeyFrames)
			if(!pKF->mpParent && pKF->mnId)	// Process all keyframes without parent, exccept id 0
				for(auto *pConnectedKF : pKF->mvpOrderedConnectedKeyFrames){
					auto poConnectedKF = static_cast<OsmapKeyFrame*>(pConnectedKF);
					if(poConnectedKF->mpParent || poConnectedKF->mnId == 0){	// Parent found: not orphan or id 0
						nParents++;
						pKF->ChangeParent(pConnectedKF);
						break;
					}
				}
		nParentsTotal += nParents;
		cout << "Parents assigned in this loop: " << nParents << endl;
	}
	cout << "Parents assigned in total: " << nParentsTotal << endl;

	/*
	 * On every MapPoint:
	 * - Rebuilds mpRefKF as the first observation, which should be the KeyFrame with the lowest id
	 * - Rebuilds many properties with UpdateNormalAndDepth()
	 */
	for(OsmapMapPoint *pMP : vectorMapPoints){
		// Rebuilds mpRefKF.  Requires mObservations.
		if(!options[NO_SET_BAD] && pMP->mnId && pMP->mObservations.empty()){
			cerr << "MP " << pMP->mnId << " without observations." << "  Set bad." << endl;
			pMP->SetBadFlag();
			continue;
		}

		// Asumes the first observation in mappoint has the lowest mnId.  Processed keyframes in mnId order ensures this.
		auto pair = (*pMP->mObservations.begin());
		pMP->mpRefKF = pair.first;

		/* UpdateNormalAndDepth() requires prior rebuilding of mpRefKF, and rebuilds:
		 * - mNormalVector
		 * - mfMinDistance
		 * - mfMaxDistance
		 */
		pMP->UpdateNormalAndDepth();
	}
	MapPoint::nNextId = vectorMapPoints.back()->mnId + 1;
}

void Osmap::getVectorKFromKeyframes(){
  vectorK.clear();
  keyframeid2vectorkIdx.resize(KeyFrame::nNextId);	// Assume map is not ill formed so nNextId is ok, thus no keyframe's id is bigger than this.
  fill(keyframeid2vectorkIdx.begin(), keyframeid2vectorkIdx.end(), 0);	// Fill with index 0 to prevent segfault from unknown bugs.

  if(vectorKeyFrames.empty())
	  getKeyFramesFromMap();

  //for(auto &pKF:map.mspKeyFrames){
  for(auto pKF: vectorKeyFrames){
    // Test if K can be found in vectorK.  If new, add it to the end of vectorK.
    //Mat &K = const_cast<cv::Mat &> (pKF->mK);
    const Mat &K = pKF->mK;

    // Will be the index of K in vectorK
    unsigned int i;
    for(i=0; i<vectorK.size(); i++){
      const Mat &vK = *vectorK[i];

      // Tests: break if found

      // Quick test
      if(K.data == vK.data) break;

      // Slow test, compare each element
/*
      if(
        K.at<float>(0,0) == vK.at<float>(0,0) &&
        K.at<float>(1,1) == vK.at<float>(1,1) &&
        K.at<float>(0,2) == vK.at<float>(0,2) &&
        K.at<float>(1,2) == vK.at<float>(1,2)
      ) break;
*/
#define DELTA 0.1
      if(
        abs(K.at<float>(0,0) - vK.at<float>(0,0)) < DELTA &&
        abs(K.at<float>(1,1) - vK.at<float>(1,1)) < DELTA &&
        abs(K.at<float>(0,2) - vK.at<float>(0,2)) < DELTA &&
        abs(K.at<float>(1,2) - vK.at<float>(1,2)) < DELTA
      ) break;
    }

    // if not found, push
    if(i>=vectorK.size()){
      // add new K
      vectorK.push_back(&K);
    }

    // i is the vectorK index for this keyframe
    keyframeid2vectorkIdx[ pKF->mnId ] = i;
  }
}

int Osmap::countFeatures(){
	int n=0;
	for(auto pKP : vectorKeyFrames)
		n += pKP->N;

	return n;
}


// Utilities
MapPoint *Osmap::getMapPoint(unsigned int id){
  //for(auto pMP : map.mspMapPoints)
  for(auto pMP : vectorMapPoints)
    if(pMP->mnId == id)
    	return pMP;

  // Not found
  return NULL;
}

OsmapKeyFrame *Osmap::getKeyFrame(unsigned int id){
  //for(auto it = map.mspKeyFrames.begin(); it != map.mspKeyFrames.end(); ++it)
  for(auto pKF : vectorKeyFrames)
	if(pKF->mnId == id)
	  return pKF;

  // If not found
  return NULL;
}




// K matrix ================================================================================================
void Osmap::serialize(const Mat &k, SerializedK *serializedK){
  serializedK->set_fx(k.at<float>(0,0));
  serializedK->set_fy(k.at<float>(1,1));
  serializedK->set_cx(k.at<float>(0,2));
  serializedK->set_cy(k.at<float>(1,2));
}

void Osmap::deserialize(const SerializedK &serializedK, Mat &m){
  m = Mat::eye(3,3,CV_32F);
  m.at<float>(0,0) = serializedK.fx();
  m.at<float>(1,1) = serializedK.fy();
  m.at<float>(0,2) = serializedK.cx();
  m.at<float>(1,2) = serializedK.cy();
}

void Osmap::serialize(const vector<Mat*> &vK, SerializedKArray &serializedKArray){

}

void Osmap::deserialize(const SerializedKArray &serializedKArray, vector<Mat*> &vK){

}



// Descriptor ================================================================================================
void Osmap::serialize(const Mat &m, SerializedDescriptor *serializedDescriptor){
  assert(m.rows == 1 && m.cols == 32);
  for(unsigned int i = 0; i<8; i++)
	serializedDescriptor->add_block(((unsigned int*)m.data)[i]);
}

void Osmap::deserialize(const SerializedDescriptor &serializedDescriptor, Mat &m){
  assert(serializedDescriptor.block_size() == 8);
  m = Mat(1, 32, CV_8UC1);
  for(unsigned int i = 0; i<8; i++)
	((unsigned int*)m.data)[i] = serializedDescriptor.block(i);
}

// Pose ================================================================================================
void Osmap::serialize(const Mat &m, SerializedPose *serializedPose){
  float *pElement = (float*) m.data;
  for(unsigned int i = 0; i<12; i++)
    serializedPose->add_element(pElement[i]);
}

void Osmap::deserialize(const SerializedPose &serializedPose, Mat &m){
  assert(serializedPose.element_size() == 12);
  m = Mat::eye(4,4,CV_32F);
  float *pElement = (float*) m.data;
  for(unsigned int i = 0; i<12; i++)
	pElement[i] = serializedPose.element(i);
}


// Position ================================================================================================
void Osmap::serialize(const Mat &m, SerializedPosition *serializedPosition){
  serializedPosition->set_x(m.at<float>(0,0));
  serializedPosition->set_y(m.at<float>(1,0));
  serializedPosition->set_z(m.at<float>(2,0));
}

void Osmap::deserialize(const SerializedPosition &serializedPosition, Mat &m){
  m = Mat(3,1,CV_32F);
  m.at<float>(0,0) = serializedPosition.x();
  m.at<float>(1,0) = serializedPosition.y();
  m.at<float>(2,0) = serializedPosition.z();
}

// KeyPoint ================================================================================================
void Osmap::serialize(const KeyPoint &kp, SerializedKeypoint *serializedKeypoint){
  serializedKeypoint->set_ptx(kp.pt.x);
  serializedKeypoint->set_pty(kp.pt.y);
  serializedKeypoint->set_octave(kp.octave);
  serializedKeypoint->set_angle(kp.angle);
}

void Osmap::deserialize(const SerializedKeypoint &serializedKeypoint, KeyPoint &kp){
  kp.pt.x   = serializedKeypoint.ptx();
  kp.pt.y   = serializedKeypoint.pty();
  kp.octave = serializedKeypoint.octave();
  kp.angle  = serializedKeypoint.angle();
}



// MapPoint ================================================================================================
void Osmap::serialize(const OsmapMapPoint &mappoint, SerializedMappoint *serializedMappoint){
  serializedMappoint->set_id(mappoint.mnId);
  serialize(mappoint.mWorldPos, serializedMappoint->mutable_position());
  serializedMappoint->set_visible(mappoint.mnVisible);
  serializedMappoint->set_found(mappoint.mnFound);
  //if(options[NO_FEATURES_DESCRIPTORS])	// This is the only descriptor to serialize	** This line is disable to force mappoint descriptor serialization, while it's not being reconstructed in rebuild. **
    serialize(mappoint.mDescriptor, serializedMappoint->mutable_briefdescriptor());
}

OsmapMapPoint *Osmap::deserialize(const SerializedMappoint &serializedMappoint){
  OsmapMapPoint *pMappoint = new OsmapMapPoint(this);
  //pMappoint->mpMap = &map;

  pMappoint->mnId        = serializedMappoint.id();
  pMappoint->mnVisible   = serializedMappoint.visible();
  pMappoint->mnFound     = serializedMappoint.found();
  if(serializedMappoint.has_briefdescriptor()) deserialize(serializedMappoint.briefdescriptor(), pMappoint->mDescriptor);
  if(serializedMappoint.has_position())        deserialize(serializedMappoint.position(),        pMappoint->mWorldPos  );

  return pMappoint;
}

int Osmap::serialize(const vector<OsmapMapPoint*>& vectorMP, SerializedMappointArray &serializedMappointArray){
  for(auto pMP : vectorMP)
    serialize(*pMP, serializedMappointArray.add_mappoint());

  return vectorMP.size();
}


int Osmap::deserialize(const SerializedMappointArray &serializedMappointArray, vector<OsmapMapPoint*>& vectorMapPoints){
  int i, n = serializedMappointArray.mappoint_size();
  for(i=0; i<n; i++)
	vectorMapPoints.push_back(deserialize(serializedMappointArray.mappoint(i)));

  return i;
}


// KeyFrame ================================================================================================
void Osmap::serialize(const OsmapKeyFrame &keyframe, SerializedKeyframe *serializedKeyframe){
  serializedKeyframe->set_id(keyframe.mnId);
  serialize(keyframe.Tcw, serializedKeyframe->mutable_pose());
  serializedKeyframe->set_timestamp(keyframe.mTimeStamp);
  if(options[K_IN_KEYFRAME])
	serialize(keyframe.mK, serializedKeyframe->mutable_kmatrix());
  else
	serializedKeyframe->set_kindex(keyframeid2vectorkIdx[keyframe.mnId]);
  if(!keyframe.mspLoopEdges.empty())
	for(auto loopKF : keyframe.mspLoopEdges)
		// Only serialize id of keyframes already serialized, to easy deserialization.
		if(keyframe.mnId > loopKF->mnId)
			serializedKeyframe->add_loopedgesids(loopKF->mnId);
}

OsmapKeyFrame *Osmap::deserialize(const SerializedKeyframe &serializedKeyframe){
	OsmapKeyFrame *pKeyframe = new OsmapKeyFrame(this);

  pKeyframe->mnId = serializedKeyframe.id();
  const_cast<double&>(pKeyframe->mTimeStamp) = serializedKeyframe.timestamp();

  if(serializedKeyframe.has_pose())
	  deserialize(serializedKeyframe.pose(), pKeyframe->Tcw);

  if(serializedKeyframe.has_kmatrix())
	  // serialized with K_IN_KEYFRAME option, doesn't use K list in yaml
	  deserialize(serializedKeyframe.kmatrix(), const_cast<cv::Mat&>(pKeyframe->mK));
  else
	  // serialized with default no K_IN_KEYFRAME option, K list in yaml
	  const_cast<cv::Mat&>(pKeyframe->mK) = *vectorK[serializedKeyframe.kindex()];

  if(serializedKeyframe.loopedgesids_size()){
	// Only ids of keyframes already deserialized and present on vectorKeyFrames
	for(int i=0; i<serializedKeyframe.loopedgesids_size(); i++){
	  unsigned int loopEdgeId = serializedKeyframe.loopedgesids(i);
	  OsmapKeyFrame *loopEdgeKF = getKeyFrame(loopEdgeId);
	  loopEdgeKF->mspLoopEdges.insert(pKeyframe);
	  pKeyframe->mspLoopEdges.insert(loopEdgeKF);
	}
  }

  return pKeyframe;
}

int Osmap::serialize(const vector<OsmapKeyFrame*>& vectorKF, SerializedKeyframeArray &serializedKeyframeArray){
  for(auto pKF: vectorKF)
    serialize(*pKF, serializedKeyframeArray.add_keyframe());

  return vectorKF.size();
}



int Osmap::deserialize(const SerializedKeyframeArray &serializedKeyframeArray, vector<OsmapKeyFrame*>& vectorKeyFrames){
  int i, n = serializedKeyframeArray.keyframe_size();
  for(i=0; i<n; i++)
	  vectorKeyFrames.push_back(deserialize(serializedKeyframeArray.keyframe(i)));

  return i;
}


// Feature ================================================================================================
void Osmap::serialize(const OsmapKeyFrame &keyframe, SerializedKeyframeFeatures *serializedKeyframeFeatures){
  serializedKeyframeFeatures->set_keyframe_id(keyframe.mnId);
  for(int i=0; i<keyframe.N; i++){
	if(!options[ONLY_MAPPOINTS_FEATURES] || keyframe.mvpMapPoints[i]){	// If chosen to only save mappoints features, check if there is a mappoint.
		SerializedFeature &serializedFeature = *serializedKeyframeFeatures->add_feature();

		// KeyPoint
		serialize(keyframe.mvKeysUn[i], serializedFeature.mutable_keypoint());

		// If there is a MapPoint, serialize it
		if(keyframe.mvpMapPoints[i])
		  serializedFeature.set_mappoint_id(keyframe.mvpMapPoints[i]->mnId);

		// Serialize descriptor but skip if chosen to not do so.
		if(!options[NO_FEATURES_DESCRIPTORS])	//
		  serialize(keyframe.mDescriptors.row(i), serializedFeature.mutable_briefdescriptor());
	}
  }
}

OsmapKeyFrame *Osmap::deserialize(const SerializedKeyframeFeatures &serializedKeyframeFeatures){
  unsigned int KFid = serializedKeyframeFeatures.keyframe_id();
  OsmapKeyFrame *pKF = getKeyFrame(KFid);
  if(pKF){
	  int n = serializedKeyframeFeatures.feature_size();
	  const_cast<int&>(pKF->N) = n;
	  const_cast<std::vector<cv::KeyPoint>&>(pKF->mvKeysUn).resize(n);
	  pKF->mvpMapPoints.resize(n);
	  const_cast<cv::Mat&>(pKF->mDescriptors) = Mat(n, 32, CV_8UC1);	// n descriptors

// ORB-SLAM2 needs to have set mvuRight and mvDepth even though they are not used in monocular.  DUMMY_MAP and OS1 don't have these properties.
#if !defined OSMAP_DUMMY_MAP && !defined OS1
	  const_cast<std::vector<float>&>(pKF->mvuRight) = vector<float>(n,-1.0f);
	  const_cast<std::vector<float>&>(pKF->mvDepth) = vector<float>(n,-1.0f);
#endif
	  for(int i=0; i<n; i++){
		const SerializedFeature &feature = serializedKeyframeFeatures.feature(i);
		if(feature.mappoint_id())		  pKF->mvpMapPoints[i] = getMapPoint(feature.mappoint_id());
		if(feature.has_keypoint())    	  deserialize(feature.keypoint(), const_cast<cv::KeyPoint&>(pKF->mvKeysUn[i]));
		if(feature.has_briefdescriptor()){
			Mat descriptor;
			deserialize(feature.briefdescriptor(), descriptor);
			descriptor.copyTo(pKF->mDescriptors.row(i));
		}
	  }
  } else {
	  cerr << "KeyFrame id "<< KFid << "not found while deserializing features: skipped.  Inconsistence between keyframes and features serialization files." << endl;
  }
  return pKF;
}


int Osmap::serialize(const vector<OsmapKeyFrame*> &vectorKF, SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray){
  unsigned int nFeatures = 0;
  for(auto pKF:vectorKF){
    serialize(*pKF, serializedKeyframeFeaturesArray.add_feature());
    nFeatures += pKF->N;
  }

  return nFeatures;
}


int Osmap::deserialize(const SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray){
  int nFeatures = 0, i, n = serializedKeyframeFeaturesArray.feature_size();
  for(i=0; i<n; i++){
    KeyFrame *pKF=deserialize(serializedKeyframeFeaturesArray.feature(i));
	if(pKF)
		nFeatures += pKF->N;
  }

  return nFeatures;
}



// Kendon Varda's code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja
// Returns false if error, true if ok.
bool Osmap::writeDelimitedTo(
    const google::protobuf::MessageLite& message,
    google::protobuf::io::ZeroCopyOutputStream* rawOutput
){
  // We create a new coded stream for each message.  Don't worry, this is fast.
  google::protobuf::io::CodedOutputStream output(rawOutput);

  // Write the size.
  const int size = message.ByteSize();
  output.WriteVarint32(size);
  uint8_t* buffer = output.GetDirectBufferForNBytesAndAdvance(size);
  if (buffer != NULL) {
    // Optimization:  The message fits in one buffer, so use the faster
    // direct-to-array serialization path.
    message.SerializeWithCachedSizesToArray(buffer);
  } else {
    // Slightly-slower path when the message is multiple buffers.
    message.SerializeWithCachedSizes(&output);
    if (output.HadError()){
      cerr << "Error in writeDelimitedTo." << endl;
      return false;
    }
  }
  return true;
}

bool Osmap::readDelimitedFrom(
    google::protobuf::io::ZeroCopyInputStream* rawInput,
    google::protobuf::MessageLite* message
){
  // We create a new coded stream for each message.  Don't worry, this is fast,
  // and it makes sure the 64MB total size limit is imposed per-message rather
  // than on the whole stream.  (See the CodedInputStream interface for more
  // info on this limit.)
  google::protobuf::io::CodedInputStream input(rawInput);

  // Read the size.
  uint32_t size;
  if (!input.ReadVarint32(&size)) return false;

  // Tell the stream not to read beyond that size.
  google::protobuf::io::CodedInputStream::Limit limit =
      input.PushLimit(size);

  // Parse the message.
  if (!message->MergeFromCodedStream(&input)) return false;
  if (!input.ConsumedEntireMessage()) return false;

  // Release the limit.
  input.PopLimit(limit);

  return true;
};


/*
 * Orbslam adapter.  Class wrappers.
 */
#ifndef OSMAP_DUMMY_MAP

OsmapMapPoint::OsmapMapPoint(Osmap *osmap):
	MapPoint(Mat(), osmap->pRefKF, &osmap->map)
{};

OsmapKeyFrame::OsmapKeyFrame(Osmap *osmap):
	KeyFrame(osmap->currentFrame, &osmap->map, &osmap->keyFrameDatabase)
{};

#else

OsmapMapPoint::OsmapMapPoint(Osmap *osmap):
	MapPoint(osmap)
{};

OsmapKeyFrame::OsmapKeyFrame(Osmap *osmap):
	KeyFrame(osmap)
{};

#endif


}	// namespace ORB_SLAM2
