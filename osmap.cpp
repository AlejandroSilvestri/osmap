#include "osmap.h"
#include <assert.h>



Osmap::Osmap(Map* _map): map(_map){}

void Osmap::mapSave(std::string baseFilename){
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
  if(options){
    headerFile << "Options" << "[:";
    if(options[ONLY_MAPPOINTS_FEATURES]) headerFile << "ONLY_MAPPOINTS_FEATURES";
    if(options[NO_ID]) headerFile << "NO_ID";
    if(options[SAVE_TIMESTAMP]) headerFile << "SAVE_TIMESTAMP";
    if(options[NO_LOOPS]) headerFile << "NO_LOOPS";
    if(options[NO_FEATURES_DESCRIPTORS]) headerFile << "NO_FEATURES_DESCRIPTORS";
    if(options[K_FILE]) headerFile << "K_FILE";
    headerFile << ":]";
  }

  // Other files
  ofstream file;
  string filename;

  // MapPoints
  filename = baseFilename + ".mappoints";
  file.open(filename, std::ofstream::binary);
  unsigned int nMappoints = SerializedMapPointArray serializedMapPointArray;
  headerFile << "mappointsFile" << filename;
  headerFile << "nMappoints"
    << serialize(map.mspMapPoints.begin(), map.mspMapPoints.end(), serializedMapPointArray);
  if (!serializedMapPointArray.SerializeToOstream(&file)) {/*error*/}
  file.close();

  // KeyFrames
  filename = baseFilename + ".keyframes";
  file.open(directory + filename, std::ofstream::binary);
  unsigned int nKeyframes = SerializedKeyFrameArray serializedKeyFrameArray;
  headerFile << "keyframesFile" << filename;
  headerFile << "nKeyframes"
    << serialize(map.mspKeyFrames.begin(), map.mspKeyFrames.end(), serializedKeyFrameArray);
  if (!serializedKeyFrameArray.SerializeToOstream(&file)) {/*error*/}
  file.close();

  // Features
  filename = baseFilename + ".features";
  file.open(directory + filename, std::ofstream::binary);
  unsigned int nFeatures = SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
  headerFile << "featuresFile" << filename;
  headerFile << "nFeatures"
    << serialize(map.mspKeyFrames.begin(), map.mspKeyFrames.end(), serializedKeyframeFeaturesArray);
  if (!serializedKeyframeFeaturesArray.SerializeToOstream(&file)) {/*error*/}
  file.close();



  // K: camera calibration matrices
  if(options(K_FILE)){
    // Save K matrices in a separate file with protocol buffers
    /*
    filename = baseFilename + ".kmatrices";
    headerFile << "kmatricesFile"  << filename;
    ...
    */
  } else {
    // Save K matrices in header file yaml
    headerFile << "cameraMatrices" << "[";
    for(auto it=vectorK.begin(); it<vectorK.end(); it++)
       headerFile << "{:"  << "fx" << *it.at<float>(0,0) << "fy" << *it.at<float>(1,1) << "cx" << *it.at<float>(0,2) << "cy" << *it.at<float>(1,2) << "}";
    headerFile << "]";
  }

  // Save yaml file
  headerFile.release();
}

void Osmap::mapLoad(std::string baseFilename){
  // Open YAML
  cv::FileStorage headerFile(baseFilename + ".yaml", cv::FileStorage::READ);
  ifstream file;
  string filename;

  // K
  FileNode cameraMatrices = headerFile["cameraMatrices"];
  // TODO
  for(auto it=vectorK.begin(); it<vectorK.end(); it++)
     headerFile << "{:"  << "fx" << *it.at<float>(0,0) << "fy" << *it.at<float>(1,1) << "cx" << *it.at<float>(0,2) << "cy" << *it.at<float>(1,2) << "}";
  headerFile << "]";

  // MapPoints
  filename = headerFile["mappointsFile"];
  file.open(filename, std::ofstream::binary);
  SerializedMapPointArray serializedMapPointArray;
  serializedMapPointArray.ParseFromIstream(file);
  cout << "Mappoints deserialized: "
    << deserialize(serializedMapPointArray, std::inserter(map.mspMapPoints, map.mspMapPoints.end()));
  if (!serializedMapPointArray.SerializeToOstream(&file)) {/*error*/}
  file.close();

  // KeyFrames
  filename = headerFile["keyfranesFile"];
  file.open(directory + filename, std::ofstream::binary);
  SerializedKeyFrameArray serializedKeyFrameArray;
  serializedKeyFrameArray.ParseFromIstream(file);
  cout << "Keyframes deserialized: "
    << deserialize(serializedKeyFrameArray, std::inserter(map.mspKeyFrames, map.mspKeyFrames.end()));
  if (!serializedKeyFrameArray.SerializeToOstream(&file)) {/*error*/}
  file.close();

  // Features
  filename = headerFile["featuresFile"];
  file.open(directory + filename, std::ofstream::binary);
  SerializedKeyframeFeaturesArray serializedKeyframeFeaturesArray;
  serializedKeyframeFeaturesArray.ParseFromIstream(file);
  cout << "Features deserialized: "
    << deserialize(serializedKeyframeFeaturesArray);
  if (!serializedKeyframeFeaturesArray.SerializeToOstream(&file)) {/*error*/}
  file.close();

  // Close yaml file
  headerFile.release();
}


void Osmap::getVectorKFromKeyframes(){
  vectorK.clear();
  getVectorKFromKeyframes.resize(KeyFrame::nNextId);
  for(auto iKF=map.mspKeyFrames.begin(); iKF<map.mspKeyFrames.end(); iKF++){
    // Test if K is new
    Mat &K = *iKF.mK;
    unsigned int i=0;
    for(; i<vectorK.size(); i++){
      Mat &vK = vectorK[i];
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
    if(i<vectorK.size()){
      // add new K
      vectorK.pushback(K);
    }
    // i is the vectorK index for this keyframe
    getVectorKFromKeyframes[iKF.mnId] = i;
  }
}

// K matrix ================================================================================================
void Osmap::serialize(Mat &k, SerializedK &serializedK){
  serializedK.set_fx(k.at<float>(0,0));
  serializedK.set_fy(k.at<float>(1,1));
  serializedK.set_cx(k.at<float>(0,2));
  serializedK.set_cy(k.at<float>(1,2));
}

void Osmap::deserialize(SerializedK &serializedK, Mat &m){
  m = Mat::eye(3,3,CV_32F);
  m.at<float>(0,0) = serializedK.fx();
  m.at<float>(1,1) = serializedK.fy();
  m.at<float>(0,2) = serializedK.cx();
  m.at<float>(1,2) = serializedK.cy();
}

void Osmap::serialize(Mat &*ks, SerializedKArray &serializedKArray){

}

Mat* Osmap::deserialize(SerializedKArray &serializedKArray){

}



// Descriptor ================================================================================================
void Osmap::serialize(Mat &m, SerializedDescriptor &serializedDescriptor){
  assert(m.rows == 1 && m.columns == 8);
  for(unsigned int i = 0; i<8; i++)
    serializedDescriptor.add_block(m.data[i]);
}

void Osmap::deserialize(SerializedDescriptor &serializedDescriptor, Mat &m){
  assert(serializedDescriptor.descriptor_size() == 8);
  m = Mat(1,8,CV_32S);
  for(unsigned int i = 0; i<8; i++)
    m.data[i] = serializedDescriptor.block(i);
}

// Pose ================================================================================================
void Osmap::serialize(Mat &m, SerializedPose &serializedPose){
  for(unsigned int i = 0; i<12; i++)
    serializedPose.add_element(m.data[i]);
}

void Osmap::deserialize(SerializedPose &serializedPose, Mat &m){
  m = Mat::eye(4,4,CV_32F);
  for(unsigned int i = 0; i<12; i++)
    m.data[i] = serializedPose.element(i);
}


// Position ================================================================================================
void Osmap::serialize(Mat &m, SerializedPosition &serializedPosition){
  serializedPosition.set_x(m.data[0]);
  serializedPosition.set_y(m.data[1]);
  serializedPosition.set_z(m.data[2]);
}

void Osmap::deserialize(SerializedPosition &serializedPosition, Mat &m){
  m = Mat(3,1,CV_32F);
  m.data[0] = serializedPosition.x();
  m.data[1] = serializedPosition.y();
  m.data[2] = serializedPosition.z();
}

// KeyPoint ================================================================================================
void Osmap::serialize(KeyPoint &kp, SerializedKeypoint &serializedKeypoint){
  serializedKeypoint.set_x(kp.pt.x);
  serializedKeypoint.set_y(kp.pt.y);
  serializedKeypoint.set_octave(kp.octave);
  serializedKeypoint.set_angle(kp.angle);
}

void Osmap::deserialize(SerializedKeypoint &serializedKeypoint, KeyPoint &kp){
  kp.pt.x   = serializedKeypoint.x();
  kp.pt.y   = serializedKeypoint.y();
  kp.octave = serializedKeypoint.octave();
  kp.angle  = serializedKeypoint.angle();
}



// MapPoint ================================================================================================
void Osmap::serialize(MapPoint *mappoint, SerializedMappoint &serializedMappoint){
  if(!options[NO_ID])
    serializedMappoint.set_id(mappoint.mnId);
  serialize(mappoint.mWorldPos, *serializedMappoint.mutable_position());
  serializedMappoint.set_visible(mappoint.mnVisible);
  serializedMappoint.set_found(mappoint.mnFound);
  if(options[NO_FEATURES_DESCRIPTORS])
    serialize(mappoint.mDescriptor, *serializedMappoint.mutable_descriptor());
}

// TODO: handle NO_ID
MapPoint *Osmap::deserialize(SerializedMappoint &serializedMappoint){
  MapPoint *pMappoint = new MapPoint();

  if(serializedMappoint.has_id())         pMappoint->mnId        = serializedMappoint.id();
  if(serializedMappoint.has_visible())    pMappoint->mnVisible   = serializedMappoint.visible();
  if(serializedMappoint.has_found())      pMappoint->mnFound     = serializedMappoint.found();
  if(serializedMappoint.has_descriptor()) deserialize(serializedMappoint.descriptor(), pMappoint->mDescriptor);
  if(serializedMappoint.has_position())   deserialize(serializedMappoint.position(),   pMappoint->mWorldPos  );

  return pMappoint;
}


int Osmap::serialize(std::iterator<std::input_iterator_tag, MapPoint*> start, std::iterator<std::input_iterator_tag, MapPoint*> end, SerializedMapPointArray &serializedMapPointArray){
  int n = 0;
  for(auto it = start; it != end; it++, n++)
    serialize(**it, *serializedMapPointArray.add_mappoint());
  return n;
}


int Osmap::deserialize(SerializedMapPointArray &serializedMapPointArray, std::iterator<std::ouput_iterator_tag, MapPoint*> output){
  for(unsigned int=0; i<serializedMapPointArray.mappoint_size(); i++){
    *output = deserialize(serializedMapPointArray.mappoint(i));
    output++;
  }
  return i;
}


// KeyFrame ================================================================================================
void Osmap::serialize(KeyFrame &keyframe, SerializedKeyframe &serializedKeyframe){
  if(!options[NO_ID]) serializedKeyframe.set_id(keyframe.mnId);
  serialize(keyframe.mTcw, *serializedKeyframe.mutable_pose());
  serializedKeyframe.set_k(keyframeid2vectork[keyframe.mnId]);
}

KeyFrame *Osmap::deserialize(SerializedKeyframe &serializedKeyframe){
  KeyFrame *pKeyframe = new KeyFrame();

  if(serializedKeyframe.has_id()) pKeyframe->mnId = serializedKeyframe.mnid();
  if(serializedKeyframe.has_pose()) deserialize(serializedKeyframe.pose(), pKeyframe->mTcw);
  if(serializedKeyframe.has_k()) pKeyframe->mK = *vectorK[serializedKeyframe.k()];

  return pKeyframe;
}


int Osmap::serialize(std::iterator<std::input_iterator_tag, KeyFrame*> start, std::iterator<std::input_iterator_tag, KeyFrame*> end, SerializedKeyFrameArray &serializedKeyFrameArray){
  int n = 0;
  for(auto it = start; it != end; it++, n++;)
    serialize(**it, *serializedKeyframeArray.add_keyframe());
  return n;
}


int Osmap::deserialize(SerializedKeyFrameArray &serializedKeyFrameArray, std::iterator<std::ouput_iterator_tag, MapPoint*> output){
  for(int=0; i<serializedKeyframeArray.keyframe_size(); i++){
    *output = deserialize(serializedKeyframeArray.keyframe(i));
    output++;
  }
  return i;
}


// Festure ================================================================================================
void Osmap::serialize(KeyFrame &keyframe, SerializedKeyframeFeatures &serializedKeyframeFeatures){
  serializedKeyframeFeatures.keyframe_id(keyframe.mnId);
  for(unsigned int i=0; i<keyframe.N; i++){
    SerializedFeature &serializedFeature = *serializedKeyframeFeatures.add_feature();
    serialize(keyframe.mvKeysUn[i], serializedFeature.keypoint());
    if(keyframe.mvpMapPoints[i])
      serializedFeature.mappoint_id(keyframe.mvpMapPoints[i]->mnId);
    if(!options[NO_FEATURES_DESCRIPTORS])
      serialize(keyframe.mDescriptors, serializedFeature.descriptor());
  }
  return serializedKeyframeFeatures;
}


KeyFrame *Osmap::deserialize(SerializedKeyframeFeatures &serializedKeyframeFeatures){
  KeyFrame *pKeyframe = getKeyFrame(serializedKeyframeFeatures.keyframe_id());
  unsigned int n = serializedKeyframeFeatures.feature_size();
  for(unsigned int i=0; i<n; i++){
    SerializedFeature &feature = serializedKeyframeFeatures.feature(i);
    if(feature.has_mappoint_id()) pKeyframe->mvpMapPoints[i] = getMapPoint(feature.mappoint_id());
    if(feature.has_keypoint())    deserialize(feature.keypoint(), pKeyframe->mvKeysUn[i]);
    if(feature.has_descriptor())  deserialize(feature.descriptor(), pKeyframe->mDescriptors[i]);
  }

  return pKeyframe;
}


int Osmap::serialize(std::iterator<std::input_iterator_tag, KeyFrame*> start, std::iterator<std::input_iterator_tag, KeyFrame*> end, SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray){
  int n = 0;
  for(auto it = start; it != end; it++, n++;)
    serialize(**it, *serializedKeyframeFeaturesArray.add_fesature());
  return n;
}


int Osmap::deserialize(SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray, std::iterator<std::ouput_iterator_tag, MapPoint*> output){
  for(int=0; i<serializedKeyframeFeaturesArray.keyframe_size(); i++){
    *output = deserialize(serializedKeyframeFeaturesArray.feature(i));
    output++;
  }
  return i;
}


// Utilities
MapPoint *Osmap::getMapPoint(unsigned int id){
  for(auto it = map->mspMapPoints.begin; it != map->mspMapPoints.end; it++)
    if(it->mnId == id) return *it;
  // Not found
  return NULL;
}

// ¿Se usa?
KeyFrame *Osmap::getKeyFrame(unsigned int id){
  // Starts from itLastKF
  for(auto it = itLastKF; it != map->mspKeyFrames.end; it++){
    if(it->mnId == id){
      itLastKF = it;
      return *it;
    }
  }

  // After the end, restarts from the begining
  for(auto it = map->mspKeyFrames.begin; it != itLastKF; it++){
    if(it->mnId == id){
      itLastKF = it;
      return *it;
    }
  }

  // After a whole cycle, not found
  return NULL;
}








int Osmap::serialize(fstream *file){
  int n = 0;
  SerializedFeature serializedFeature;
  for(
    auto itKF = map.mspKeyFrames.begin();
    itKF != map.mspKeyFrames.end();
    itKF++;
  ){
    unsigned int kfId = itKF->mnId;
    for(unsigned int i=0; i<it->N; i++){
      serializeFeature(*it, i, &serializedFeature);
      writeDelimitedTo(serializedFeature, file);
      n++;
    }
  }
  return n;

}

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
