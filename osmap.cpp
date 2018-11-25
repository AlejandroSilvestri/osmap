#include "osmap.h"
#include <fstream>
#include <iostream>
#include <assert.h>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

unsigned int KeyFrame::nNextId = 0;

void Osmap::mapSave(string baseFilename){
  // Map depuration

  // Savings

  // Open YAML file for write, it will be the last file to close.
  // FileStorage https://docs.opencv.org/3.1.0/da/d56/classcv_1_1FileStorage.html
  cv::FileStorage headerFile(baseFilename + ".yaml", cv::FileStorage::WRITE);
  if(!headerFile.isOpened()){
    // Is this necesary?
     cerr << "No se pudo crear el archivo " << baseFilename << ".yaml" << endl;
     return;
  }

  // Options
  if(options.any()){
    headerFile << "Options" << "[:";
    if(options[ONLY_MAPPOINTS_FEATURES]) headerFile << "ONLY_MAPPOINTS_FEATURES";
    if(options[NO_ID]) headerFile << "NO_ID";
    if(options[SAVE_TIMESTAMP]) headerFile << "SAVE_TIMESTAMP";
    if(options[NO_LOOPS]) headerFile << "NO_LOOPS";
    if(options[NO_FEATURES_DESCRIPTORS]) headerFile << "NO_FEATURES_DESCRIPTORS";
    if(options[K_IN_KEYFRAME]) headerFile << "K_IN_KEYFRAME";
    headerFile << ":]";
  }

  // Other files
  ofstream file;
  string filename;

  // MapPoints
  filename = baseFilename + ".mappoints";
  file.open(filename, std::ofstream::binary);
  SerializedMappointArray serializedMappointArray;
  headerFile << "mappointsFile" << filename;
  headerFile << "nMappoints" << serialize(map.mspMapPoints, serializedMappointArray);
  if (!serializedMappointArray.SerializeToOstream(&file)) {/*error*/}
  file.close();

  // K: grab camera calibration matrices.  Will be saved to yaml file later.
  getVectorKFromKeyframes();

  // KeyFrames
  filename = baseFilename + ".keyframes";
  file.open(filename, ofstream::binary);
  SerializedKeyframeArray serializedKeyFrameArray;
  headerFile << "keyframesFile" << filename;
  headerFile << "nKeyframes" << serialize(map.mspKeyFrames, serializedKeyFrameArray);
  if (!serializedKeyFrameArray.SerializeToOstream(&file)) {/*error*/}
  file.close();

  // Features
  filename = baseFilename + ".features";
  file.open(filename, ofstream::binary);
  SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
  headerFile << "featuresFile" << filename;
  headerFile << "nFeatures"
    << serialize(map.mspKeyFrames, serializedKeyframeFeaturesArray);
  if (!serializedKeyframeFeaturesArray.SerializeToOstream(&file)) {/*error*/}
  file.close();



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

  // K

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



  // MapPoints
  headerFile["mappointsFile"] >> filename;
  file.open(filename, ifstream::binary);
  SerializedMappointArray serializedMappointArray;
  serializedMappointArray.ParseFromIstream(&file);
  cout << "Mappoints deserialized: "
    << deserialize(serializedMappointArray, map.mspMapPoints);
  file.close();

  // KeyFrames
  headerFile["keyfranesFile"] >> filename;
  file.open(filename, ifstream::binary);
  SerializedKeyframeArray serializedKeyFrameArray;
  serializedKeyFrameArray.ParseFromIstream(&file);
  cout << "Keyframes deserialized: "
    << deserialize(serializedKeyFrameArray, map.mspKeyFrames);
  file.close();

  // Features
  headerFile["featuresFile"] >> filename;
  file.open(filename, ifstream::binary);
  SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
  serializedKeyframeFeaturesArray.ParseFromIstream(&file);
  cout << "Features deserialized: "
    << deserialize(serializedKeyframeFeaturesArray, map.mspKeyFrames);
  if (!serializedKeyframeFeaturesArray.ParseFromIstream(&file)) {/*error*/}
  file.close();

  // Close yaml file
  headerFile.release();
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
  if(!options[NO_ID])
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


int Osmap::serialize(const set<MapPoint*>& setMapPoints, SerializedMappointArray &serializedMappointArray){
  //int n = 0;
  //for(auto it = setMapPoints.begin(); it != setMapPoints.end(); it++, n++;)
  for(auto pMP : setMapPoints)
    serialize(*pMP, serializedMappointArray.add_mappoint());
    //n++;

  return setMapPoints.size();
}


int Osmap::deserialize(const SerializedMappointArray &serializedMappointArray, set<MapPoint*>& setMapPoints){
  int i;
  for(i=0; i<serializedMappointArray.mappoint_size(); i++)
	  setMapPoints.insert(deserialize(serializedMappointArray.mappoint(i)));

  return i;
}


// KeyFrame ================================================================================================
void Osmap::serialize(const KeyFrame &keyframe, SerializedKeyframe *serializedKeyframe){
  if(!options[NO_ID]) serializedKeyframe->set_id(keyframe.mnId);
  serialize(keyframe.mTcw, serializedKeyframe->mutable_pose());
  if(options[K_IN_KEYFRAME])
	;	// TODO: serialize K in keyframe, need to add this field in proto file
  else
	serializedKeyframe->set_k(keyframeid2vectork[keyframe.mnId]);
}

KeyFrame *Osmap::deserialize(const SerializedKeyframe &serializedKeyframe){
  KeyFrame *pKeyframe = new KeyFrame();

  pKeyframe->mnId = serializedKeyframe.id();
  if(serializedKeyframe.has_pose()) deserialize(serializedKeyframe.pose(), pKeyframe->mTcw);
  pKeyframe->mK = *vectorK[serializedKeyframe.k()];

  return pKeyframe;
}


int Osmap::serialize(const set<KeyFrame*>& setKeyFrame, SerializedKeyframeArray &serializedKeyframeArray){
  for(auto pKF: setKeyFrame)
    serialize(*pKF, serializedKeyframeArray.add_keyframe());

  return setKeyFrame.size();
}


int Osmap::deserialize(const SerializedKeyframeArray &serializedKeyframeArray, set<KeyFrame*>& setKeyFrame){
  int i;
  for(i=0; i<serializedKeyframeArray.keyframe_size(); i++)
		setKeyFrame.insert(deserialize(serializedKeyframeArray.keyframe(i)));

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
    if(!options[NO_FEATURES_DESCRIPTORS])
      serialize(keyframe.mDescriptors.row(i), serializedFeature.mutable_briefdescriptor());
  }
}


KeyFrame *Osmap::deserialize(const SerializedKeyframeFeatures &serializedKeyframeFeatures){
  KeyFrame *pKF = getKeyFrame(serializedKeyframeFeatures.keyframe_id());
  unsigned int n = serializedKeyframeFeatures.feature_size();
  for(unsigned int i=0; i<n; i++){
    const SerializedFeature &feature = serializedKeyframeFeatures.feature(i);
    if(feature.mappoint_id())		  pKF->mvpMapPoints[i] = getMapPoint(feature.mappoint_id());
    if(feature.has_keypoint())    	  deserialize(feature.keypoint(), pKF->mvKeysUn[i]);
    if(feature.has_briefdescriptor()){
    	Mat descriptor;
    	deserialize(feature.briefdescriptor(), descriptor);
    	pKF->mDescriptors.push_back(descriptor);
    }
  }

  return pKF;
}


int Osmap::serialize(const set<KeyFrame*> &setKeyFrame, SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray){
  //unsigned int n = 0;
  //for(auto it = start; it != end; it++, n++;)
  for(auto pKF:setKeyFrame)
    serialize(*pKF, serializedKeyframeFeaturesArray.add_feature());

  return setKeyFrame.size();
}


int Osmap::deserialize(const SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray, set<KeyFrame*> &setKeyFrame){
  itLastKF = setKeyFrame.begin();
  int i;
  for(i=0; i<serializedKeyframeFeaturesArray.feature_size(); i++)
	setKeyFrame.insert(deserialize(serializedKeyframeFeaturesArray.feature(i)));

  return i;
}


// Utilities
MapPoint *Osmap::getMapPoint(unsigned int id){
  for(auto pMP : map.mspMapPoints)
    if(pMP->mnId == id) return pMP;
  // Not found
  return NULL;
}

KeyFrame *Osmap::getKeyFrame(unsigned int id){
  // Starts from itLastKF
  for(auto it = itLastKF; it != map.mspKeyFrames.end(); ++it){
    if((*it)->mnId == id){
      itLastKF = it;
      return *it;
    }
  }

  // After the end, restarts from the begining
  for(auto it = map.mspKeyFrames.begin(); it != itLastKF; ++it){
    if((*it)->mnId == id){
      itLastKF = it;
      return *it;
    }
  }

  // After a whole cycle, not found
  return NULL;
}






/*
int Osmap::serialize(fstream *file){
  int n = 0;
  SerializedFeature serializedFeature;
  for(auto pKF:map.mspKeyFrames){
    unsigned int kfId = pKF->mnId;
    for(unsigned int i=0; i < pKF->N; i++){
      serialize(*pKF, i, &serializedFeature);
      writeDelimitedTo(serializedFeature, file);
      n++;
    }
  }
  return n;

}
*//*
int Osmap::deserialize(fstream *file){
  int n = 0;
  SerializedFeature serializedFeature;
  auto it = map.mspKeyFrames.begin();
  for(
    auto itKF = map.mspKeyFrames.begin();
    itKF != map.mspKeyFrames.end();
    itKF++;
  ){
    unsigned int kfId = itKF->mnId;
    for(unsigned int i=0; i<itKF->N; i++){
      readDelimitedTo(serializedKeyframe, file)
      if(!readDelimitedTo(serializedFeature, file)) return -1;
      deserializeFeature(&serializedFeature, *itKF, i);
      n++;
    }
    it++;
    n++;
  }
  return n;

}
*/




// Kendon Varda code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja

bool Osmap::writeDelimitedTo(
    const google::protobuf::MessageLite& message,
    google::protobuf::io::ZeroCopyOutputStream* rawOutput) {
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
    google::protobuf::MessageLite* message) {
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
