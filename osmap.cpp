#include "osmap.h"
#include <fstream>
#include <iostream>
#include <assert.h>
#include <opencv2/core.hpp>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#define OPTION(OP) if(options[OP]) headerFile << #OP;


using namespace std;
using namespace cv;

unsigned int KeyFrame::nNextId = 0;

void Osmap::mapSave(string baseFilename){
  // Map depuration
  if(!options[NO_DEPURATION])
	depurate();


  // "Savings"

  // Open YAML file for write, it will be the last file to close.
  // FileStorage https://docs.opencv.org/3.1.0/da/d56/classcv_1_1FileStorage.html
  cv::FileStorage headerFile(baseFilename + ".yaml", cv::FileStorage::WRITE);
  if(!headerFile.isOpened()){
    // Is this necessary?
     cerr << "Couldn't create file " << baseFilename << ".yaml" << endl;
     return;
  }

  // Other files
  ofstream file;
  string filename;

  // MapPoints
  if(!options[NO_MAPPOINTS_FILE]){
	  // Order keyframes by mnId
	  vectorMapPoints.clear();
	  vectorMapPoints.reserve(map.mspMapPoints.size());
	  vectorMapPoints.assign(map.mspMapPoints.begin(), map.mspMapPoints.end());
	  sort(vectorMapPoints.begin(), vectorMapPoints.end(), [](const MapPoint* a, const MapPoint* b){return a->mnId < b->mnId;});

	  // New file
	  filename = baseFilename + ".mappoints";
	  file.open(filename, std::ofstream::binary);

	  // Serialize
	  SerializedMappointArray serializedMappointArray;
	  headerFile << "mappointsFile" << filename;
	  headerFile << "nMappoints" << serialize(vectorMapPoints, serializedMappointArray);
	  if (!serializedMappointArray.SerializeToOstream(&file)) {/*error*/}

	  file.close();
  }

  // K: grab camera calibration matrices.  Will be saved to yaml file later.
  if(!options[K_IN_KEYFRAME]) getVectorKFromKeyframes();

  // KeyFrames
  if(!options[NO_KEYFRAMES_FILE]){
	  // Order keyframes by mnId
	  vectorKeyFrames.clear();
	  vectorKeyFrames.reserve(map.mspKeyFrames.size());
	  vectorKeyFrames.assign(map.mspKeyFrames.begin(), map.mspKeyFrames.end());
	  //copy(map.mspKeyFrames.begin(), map.mspKeyFrames.end(), vectorKeyFrames.begin());//back_inserter(vectorKeyFrames));
	  sort(vectorKeyFrames.begin(), vectorKeyFrames.end(), [](const KeyFrame *a, const KeyFrame *b){return a->mnId < b->mnId;});

	  // New file
	  filename = baseFilename + ".keyframes";
	  file.open(filename, ofstream::binary);

	  // Serialize
	  SerializedKeyframeArray serializedKeyFrameArray;
	  headerFile << "keyframesFile" << filename;
	  headerFile << "nKeyframes" << serialize(vectorKeyFrames, serializedKeyFrameArray);
	  if (!serializedKeyFrameArray.SerializeToOstream(&file)) {/*error*/}

	  file.close();
  }

  // Features
  if(!options[NO_FEATURES_FILE]){
	  filename = baseFilename + ".features";
	  file.open(filename, ofstream::binary);
	  headerFile << "featuresFile" << filename;
	  if(
		options[FEATURES_FILE_DELIMITED] ||
		(!options[FEATURES_FILE_NOT_DELIMITED] && countFeatures() > FEATURES_MESSAGE_LIMIT)
	  ){
		  options.set(FEATURES_FILE_DELIMITED);

		  // Loop serializing blocks of no more than FEATURES_MESSAGE_LIMIT features, using Kendon Varda's function
		  int nFeatures = 0;
		  vector<KeyFrame*> vectorBlock;
		  vectorBlock.reserve(FEATURES_MESSAGE_LIMIT/30);
		  auto it = vectorKeyFrames.begin();
		  auto *googleStream = new ::google::protobuf::io::OstreamOutputStream(&file);
		  while(it != vectorKeyFrames.end()){
			  unsigned int n = (*it)->N;
			  vectorBlock.clear();
			  do{
				  vectorBlock.push_back(*it);
				  ++it;
				  if(it == vectorKeyFrames.end()) break;
				  KeyFrame* KF = *it;
				  n += KF->N;
				  //n += (*it)->N;
			  } while(n <= FEATURES_MESSAGE_LIMIT);

			  SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
			  nFeatures += serialize(vectorBlock, serializedKeyframeFeaturesArray);
			  writeDelimitedTo(serializedKeyframeFeaturesArray, googleStream);
		  }
		  headerFile << "nFeatures" << nFeatures;
	  }else{
		  options.set(FEATURES_FILE_NOT_DELIMITED);
		  SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
		  headerFile << "nFeatures" << serialize(vectorKeyFrames, serializedKeyframeFeaturesArray);
		  if (!serializedKeyframeFeaturesArray.SerializeToOstream(&file)) {/*error*/}
	  }
	  file.close();
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
}

void Osmap::mapLoad(string baseFilename){
  // Open YAML
  cv::FileStorage headerFile(baseFilename + ".yaml", cv::FileStorage::READ);
  ifstream file;
  string filename;
  int intOptions;

  // Options
  headerFile["Options"] >> intOptions;
  options = intOptions;

  // K
  if(!options[K_IN_KEYFRAME]){
	  FileNode cameraMatrices = headerFile["cameraMatrices"];
	  FileNodeIterator it = cameraMatrices.begin(), it_end = cameraMatrices.end();
	  for( ; it != it_end; ++it){
		  // TODO
		  Mat *k = new Mat();
		  *k = Mat::eye(3,3,CV_32F);
		  k->at<float>(0,0) = (*it)["fx"];
		  k->at<float>(1,1) = (*it)["fy"];
		  k->at<float>(2,0) = (*it)["cx"];
		  k->at<float>(2,1) = (*it)["cy"];
		  vectorK.push_back(k);
	  }
  }


  // MapPoints
  if(!options[NO_MAPPOINTS_FILE]){
	  headerFile["mappointsFile"] >> filename;
	  file.open(filename, ifstream::binary);
	  SerializedMappointArray serializedMappointArray;
	  serializedMappointArray.ParseFromIstream(&file);
	  cout << "Mappoints deserialized: "
		<< deserialize(serializedMappointArray, map.mspMapPoints) << endl;
	  file.close();
  }

  if(!options[NO_KEYFRAMES_FILE]){
	  // KeyFrames
	  headerFile["keyframesFile"] >> filename;
	  file.open(filename, ifstream::binary);
	  SerializedKeyframeArray serializedKeyFrameArray;
	  serializedKeyFrameArray.ParseFromIstream(&file);
	  cout << "Keyframes deserialized: "
		<< deserialize(serializedKeyFrameArray, map.mspKeyFrames) << endl;
	  file.close();
  }

  // Features
  if(!options[NO_FEATURES_FILE]){
	  headerFile["featuresFile"] >> filename;
	  file.open(filename, ifstream::binary);
	  auto *googleStream = new ::google::protobuf::io::IstreamInputStream(&file);
	  SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
	  if(options[FEATURES_FILE_DELIMITED]){
		  bool dataRemaining;
		  do{
			  dataRemaining = readDelimitedFrom(googleStream, &serializedKeyframeFeaturesArray);
			  cout << "Features deserialized in loop: "
				<< deserialize(serializedKeyframeFeaturesArray, map.mspKeyFrames) << endl;
		  } while(dataRemaining);
	  } else {
		  // Not delimited, pure Protocol Buffers
		  serializedKeyframeFeaturesArray.ParseFromIstream(&file);
		  cout << "Features deserialized: " << deserialize(serializedKeyframeFeaturesArray, map.mspKeyFrames) << endl;
	  }
	  file.close();
  }

  // Close yaml file
  headerFile.release();
}


void Osmap::depurate(){
	// First erase MapPoint from KeyFrames, and then erase KeyFrames from MapPoints.

	// NULL out bad MapPoints in KeyFrame::mvpMapPoints
	for(auto &pKF: map.mspKeyFrames){
		// NULL out bad MapPoints and warns if not in map.  Usually doesn't find anything.
		auto &pMPs = pKF->mvpMapPoints;
		for(int i=pMPs.size(); --i>=0;){
			auto pMP = pMPs[i];

			if(!pMP) continue;	// Ignore if NULL

			if(pMP->mbBad && !options[NO_ERASE_BAD_MAPPOINTS]){	// If MapPoint is bad, NULL it
				cout << "ERASE_BAD_MAPPOINTS: Nullifying bad MapPoint " << pMP->mnId << " in KeyFrame " << pKF->mnId << endl;
				pMPs[i] = NULL;
			} else if(!map.mspMapPoints.count(pMP) && !options[NO_APPEND_FOUND_MAPPOINTS]){	// If MapPoint is not in map, append it to the map
				map.mspMapPoints.insert(pMP);
				cout << "APPEND_FOUND_MAPPOINTS: MapPoint " << pMP->mnId << " added to map. ";
			}
		}

		// NULL out bad loop edges.  Loop edges are KeyFrames.
		if(!options[NO_ERASE_ORPHAN_KEYFRAME_IN_LOOP])
		  for(auto &pKFLoop: pKF->mspLoopEdges)
			if(pKFLoop->mbBad || !map.mspKeyFrames.count(pKFLoop)){
			  cout << "ERASE_ORPHAN_KEYFRAME_IN_LOOP: Nullifying loop edge " << pKFLoop->mnId << " from keyframe " << pKF->mnId << endl;
			  pKF->mspLoopEdges.erase(pKFLoop);
			}
	}
}
//if(!options[])

void Osmap::rebuild(){

}

void Osmap::getVectorKFromKeyframes(){
  vectorK.clear();
  keyframeid2vectork.resize(KeyFrame::nNextId);
  for(auto &pKF:map.mspKeyFrames){
    // Test if K is new
    Mat &K = pKF->mK;
    unsigned int i=0;
    for(; i<vectorK.size(); i++){
      Mat &vK = *vectorK[i];
      // Quick test
      if(K.data == vK.data) break;

      // Slow test, compare each element
      if(
        K.at<float>(0,0) != vK.at<float>(0,0) ||
        K.at<float>(1,1) != vK.at<float>(1,1) ||
        K.at<float>(0,2) != vK.at<float>(0,2) ||
        K.at<float>(1,2) != vK.at<float>(1,2)
      ) break;

    }
    if(i>=vectorK.size()){
      // add new K
      vectorK.push_back(&K);
    }
    // i is the vectorK index for this keyframe
    keyframeid2vectork[ pKF->mnId ] = i;
  }
}

int Osmap::countFeatures(){
	int n=0;
	for(auto pKP : vectorKeyFrames)
		n += pKP->N;

	return n;
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
  assert(m.rows == 1 && m.cols == 8);
  for(unsigned int i = 0; i<8; i++){
	serializedDescriptor->add_block(m.at<unsigned int>(0,i));
  }
}

void Osmap::deserialize(const SerializedDescriptor &serializedDescriptor, Mat &m){
  assert(serializedDescriptor.block_size() == 8);
  m = Mat(1,8,CV_32S);
  for(unsigned int i = 0; i<8; i++)
    m.at<unsigned int>(0,i) = serializedDescriptor.block(i);
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
void Osmap::serialize(const MapPoint &mappoint, SerializedMappoint *serializedMappoint){
  serializedMappoint->set_id(mappoint.mnId);
  serialize(mappoint.mWorldPos, serializedMappoint->mutable_position());
  serializedMappoint->set_visible(mappoint.mnVisible);
  serializedMappoint->set_found(mappoint.mnFound);
  if(!options[NO_FEATURES_DESCRIPTORS])
    serialize(mappoint.mDescriptor, serializedMappoint->mutable_briefdescriptor());
}

// TODO: handle NO_ID, autonumerate MapPoints
MapPoint *Osmap::deserialize(const SerializedMappoint &serializedMappoint){
  MapPoint *pMappoint = new MapPoint();

  pMappoint->mnId        = serializedMappoint.id();
  pMappoint->mnVisible   = serializedMappoint.visible();
  pMappoint->mnFound     = serializedMappoint.found();
  if(serializedMappoint.has_briefdescriptor()) deserialize(serializedMappoint.briefdescriptor(), pMappoint->mDescriptor);
  if(serializedMappoint.has_position())        deserialize(serializedMappoint.position(),        pMappoint->mWorldPos  );

  return pMappoint;
}

/*
int Osmap::serialize(const set<MapPoint*>& setMapPoints, SerializedMappointArray &serializedMappointArray){
  //int n = 0;
  //for(auto it = setMapPoints.begin(); it != setMapPoints.end(); it++, n++;)
  for(auto pMP : setMapPoints)
    serialize(*pMP, serializedMappointArray.add_mappoint());
    //n++;

  return setMapPoints.size();
}
*/

int Osmap::serialize(const vector<MapPoint*>& vectorMP, SerializedMappointArray &serializedMappointArray){
  for(auto pMP : vectorMP)
    serialize(*pMP, serializedMappointArray.add_mappoint());

  return vectorMP.size();
}




int Osmap::deserialize(const SerializedMappointArray &serializedMappointArray, set<MapPoint*>& setMapPoints){
  int i, n = serializedMappointArray.mappoint_size();
  for(i=0; i<n; i++)
	setMapPoints.insert(deserialize(serializedMappointArray.mappoint(i)));

  return i;
}


// KeyFrame ================================================================================================
void Osmap::serialize(const KeyFrame &keyframe, SerializedKeyframe *serializedKeyframe){
  serializedKeyframe->set_id(keyframe.mnId);
  serialize(keyframe.mTcw, serializedKeyframe->mutable_pose());
  serializedKeyframe->set_timestamp(keyframe.mTimeStamp);
  if(options[K_IN_KEYFRAME])
	serialize(keyframe.mK, serializedKeyframe->mutable_kmatrix());
  else
	serializedKeyframe->set_kindex(keyframeid2vectork[keyframe.mnId]);
}

KeyFrame *Osmap::deserialize(const SerializedKeyframe &serializedKeyframe){
  KeyFrame *pKeyframe = new KeyFrame();

  pKeyframe->mnId = serializedKeyframe.id();
  pKeyframe->mTimeStamp = serializedKeyframe.timestamp();

  if(serializedKeyframe.has_pose())
	  deserialize(serializedKeyframe.pose(), pKeyframe->mTcw);

  if(serializedKeyframe.has_kmatrix())
	  // serialized with K_IN_KEYFRAME option, doesn't use K list in yaml
	  deserialize(serializedKeyframe.kmatrix(), pKeyframe->mK);
  else
	  // serialized with default no K_IN_KEYFRAME option, K list in yaml
	  pKeyframe->mK = *vectorK[serializedKeyframe.kindex()];

  return pKeyframe;
}

int Osmap::serialize(const vector<KeyFrame*>& vectorKF, SerializedKeyframeArray &serializedKeyframeArray){
  for(auto pKF: vectorKF)
    serialize(*pKF, serializedKeyframeArray.add_keyframe());

  return vectorKF.size();
}


/*
int Osmap::serialize(const set<KeyFrame*>& setKeyFrame, SerializedKeyframeArray &serializedKeyframeArray){
  for(auto pKF: setKeyFrame)
    serialize(*pKF, serializedKeyframeArray.add_keyframe());

  return setKeyFrame.size();
}
*/

int Osmap::deserialize(const SerializedKeyframeArray &serializedKeyframeArray, set<KeyFrame*>& setKeyFrames){
  int i, n = serializedKeyframeArray.keyframe_size();
  for(i=0; i<n; i++)
	setKeyFrames.insert(deserialize(serializedKeyframeArray.keyframe(i)));

  return i;
}


// Feature ================================================================================================
void Osmap::serialize(const KeyFrame &keyframe, SerializedKeyframeFeatures *serializedKeyframeFeatures){
  serializedKeyframeFeatures->set_keyframe_id(keyframe.mnId);
  for(unsigned int i=0; i<keyframe.N; i++){
    SerializedFeature &serializedFeature = *serializedKeyframeFeatures->add_feature();
    serialize(keyframe.mvKeysUn[i], serializedFeature.mutable_keypoint());
    if(keyframe.mvpMapPoints[i])
      serializedFeature.set_mappoint_id(keyframe.mvpMapPoints[i]->mnId);
    if(
         !options[NO_FEATURES_DESCRIPTORS]	// Skip if chosen to not save descriptor
	  && (!options[ONLY_MAPPOINTS_FEATURES] || keyframe.mvpMapPoints[i]) // If chosen to only save mappoints features, check if there is a mappoint.
	)
      serialize(keyframe.mDescriptors.row(i), serializedFeature.mutable_briefdescriptor());
  }
}


KeyFrame *Osmap::deserialize(const SerializedKeyframeFeatures &serializedKeyframeFeatures){
  KeyFrame *pKF = getKeyFrame(serializedKeyframeFeatures.keyframe_id());
  if(pKF){
	  unsigned int n = serializedKeyframeFeatures.feature_size();
	  pKF->N = n;
	  pKF->mvKeysUn.resize(n);
	  pKF->mvpMapPoints.resize(n);
	  pKF->mDescriptors = Mat(n, 8, CV_32S);	// n descriptors
	  for(unsigned int i=0; i<n; i++){
		const SerializedFeature &feature = serializedKeyframeFeatures.feature(i);
		if(feature.mappoint_id())		  pKF->mvpMapPoints[i] = getMapPoint(feature.mappoint_id());
		if(feature.has_keypoint())    	  deserialize(feature.keypoint(), pKF->mvKeysUn[i]);
		if(feature.has_briefdescriptor()){
			Mat descriptor;
			deserialize(feature.briefdescriptor(), descriptor);
			descriptor.copyTo(pKF->mDescriptors.row(i));
		}
	  }
  } else {
	  cout << "KeyFrame id not found while deserializing features: skipped.  Inconsistence between keyframes and features serialization files." << endl;
  }
  return pKF;
}


int Osmap::serialize(const vector<KeyFrame*> &vectorKF, SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray){
  unsigned int nFeatures = 0;
  for(auto pKF:vectorKF){
    serialize(*pKF, serializedKeyframeFeaturesArray.add_feature());
    nFeatures += pKF->N;
  }

  return nFeatures;
}


int Osmap::deserialize(const SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray, set<KeyFrame*> &setKeyFrame){
  int nFeatures = 0, i, n = serializedKeyframeFeaturesArray.feature_size();
  for(i=0; i<n; i++)
	nFeatures += deserialize(serializedKeyframeFeaturesArray.feature(i))->N;

  return nFeatures;
}


// Utilities
MapPoint *Osmap::getMapPoint(unsigned int id){
  for(auto pMP : map.mspMapPoints)
    if(pMP->mnId == id) return pMP;
  // Not found
  return NULL;
}

KeyFrame *Osmap::getKeyFrame(unsigned int id){
  for(auto it = map.mspKeyFrames.begin(); it != map.mspKeyFrames.end(); ++it)
	if((*it)->mnId == id)
	  return *it;

  // If not found
  return NULL;
}



// Kendon Varda's code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja

bool Osmap::writeDelimitedTo(
    const google::protobuf::MessageLite& message,
    google::protobuf::io::ZeroCopyOutputStream* rawOutput
){
  cout << "call to writeDelimitedTo" << endl;
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
    if (output.HadError()) return false;
  }

  return true;
}

bool Osmap::readDelimitedFrom(
    google::protobuf::io::ZeroCopyInputStream* rawInput,
    google::protobuf::MessageLite* message
){
  cout << "call to readDelimitedFrom" << endl;

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
}
