#include "osmap.h"




Osmap::Osmap(Map* _map): map(_map){}

void Osmap::mapSave(std::string archivo){

}

void Osmap::mapLoad(std::string archivo){

}



/**
Serialize provided intrinsic camera matrix K to the protocol buffer message.
All 4 fields required.
*/
SerializedK *Osmap::serializeK(Mat *k, SerializedK *serializedK){
  serializedK->set_fx(k->at<float>(0,0));
  serializedK->set_fy(k->at<float>(1,1));
  serializedK->set_cx(k->at<float>(0,2));
  serializedK->set_cy(k->at<float>(1,2));
  return sk;
}

/**
Reconstruct the intrinsic camera matrix K from protocol buffer message.
All 4 fields are required.
*/
Mat *Osmap::deserializeK(SerializedK *serializedK, Mat *m){
  // Handle default
  if(m==NULL) m = new Mat();

  m = Mat::eye(3,3,CV_32F);
  m->at<float>(0,0) = serializedK->fx();
  m->at<float>(1,1) = serializedK->fy();
  m->at<float>(0,2) = serializedK->cx();
  m->at<float>(1,2) = serializedK->cy();
  return m;
}

SerializedDescriptor *Osmap::serializeDescriptor(Mat *m, SerializedDescriptor *serializedDescriptor){
  for(unsigned int i = 0; i<8; i++)
    serializedDescriptor->add_block(m->data[i]);
  return serializedDescriptor;
}

/**
Reconstruct a descriptor, an 1x8 int Mat (256 bits).
Exactly 8 int required.
*/
Mat *Osmap::deserializeDescriptor(SerializedDescriptor *serializedDescriptor, Mat *m){
  // Handle default
  if(m==NULL) m = new Mat();

  m = Mat(4,4,CV_32S);
  for(unsigned int i = 0; i<8; i++)
    m->data[i] = serializedDescriptor->block(i);
  return m;
}

/**
Serialize a 4x4 float Mat representing a pose in homogeneous coordinates.
Exactly 12 float required.
*/
SerializedPose *Osmap::serializePose(Mat *m, SerializedPose *serializedPose){
  for(unsigned int i = 0; i<12; i++)
    serializedPose->add_element(m->data[i]);
  return serializedPose;
}

/**
Reconstruct a 4x4 float Mat representing a pose in homogeneous coordinates.
Exactly 12 float required.
*/
Mat *Osmap::deserializePose(SerializedPose *serializedPose, Mat *m){
  // Handle default
  if(m==NULL) m = new Mat();

  m = Mat::eye(4,4,CV_32F);
  for(unsigned int i = 0; i<12; i++)
    m->data[i] = serializedPose->element(i);
  return m;
}



/**
Serialize 3D mappoint position, a 3x1 float Mat.
All 3 fields required.
*/
SerializedPosition *Osmap::serializePosition(Mat *m, SerializedPosition *serializedPosition){
  serializedPosition->set_x(m->data[0]);
  serializedPosition->set_y(m->data[1]);
  serializedPosition->set_z(m->data[2]);
  return serializedPosition;
}

/**
Reconstructs 3D mappoint position in a 3x1 float Mat.
All 3 fields required.
*/
Mat *Osmap::deserializePosition(SerializedPosition *serializedPosition, Mat *m){
  // Handle default
  if(m==NULL) m = new Mat();

  m = Mat(3,1,CV_32F);
  m->data[0] = serializedPosition->x();
  m->data[1] = serializedPosition->y();
  m->data[2] = serializedPosition->z();
  return m;
}

/**
Serialize 4 properties of a KeyPoint.
All 4 fields required.
*/
SerializedKeypoint *Osmap::serializeKeypoint(KeyPoint *pKP, SerializedKeypoint *serializedKeypoint){
  serializedKeypoint->set_x(pKP->pt.x);
  serializedKeypoint->set_y(pKP->pt.y);
  serializedKeypoint->set_octave(pKP->octave);
  serializedKeypoint->set_angle(pKP->angle);
  return serializedKeypoint;
}

/**
Reconstructs a KeyPoint.
All 4 fields required.
*/
KeyPoint *Osmap::deserializeKeypoint(SerializedKeypoint* serializedKeypoint, KeyPoint* pKP){
  // Handle default
  if(pKP == NULL) pKP = new KeyPoint();

  pKP->pt.x   = serializedKeypoint->x();
  pKP->pt.y   = serializedKeypoint->y();
  pKP->octave = serializedKeypoint->octave();
  pKP->angle  = serializedKeypoint->angle();
  return pKP;
}

/**
Serializes a MapPoint, according to options.
*/
SerializedMappoint *Osmap::serializeMappoint(MapPoint *pMP, SerializedMappoint *serializedMappoint){
  if(!options[NO_ID]) serializedMappoint->set_id(pMp->mnId);
  serializePosition(&pMp->mWorldPos, serializedMappoint->mutable_position());
  serializedMappoint->set_visible(pMp->mnVisible);
  serializedMappoint->set_found(pMp->mnFound);
  if(options[NO_FEATURES_DESCRIPTORS]) serializeDescriptor(&pMp->mDescriptor, serializedMappoint->mutable_descriptor());
}


/**
Reconstructs a MapPoint from optional fields.
It doesn't perform MapPoint initialization.  This should be done after deserialization.
*/
MapPoint *Osmap::deserializeMappoint(SerializedMappoint *serializedMappoint, MapPoint *pMP){
  // Handle default
  if(pMP == NULL) pMP = new MapPoint();

  if(serializedMappoint->has_id())         pMp->mnId        = serializedMappoint->id();
  if(serializedMappoint->has_visible())    pMp->mnVisible   = serializedMappoint->visible();
  if(serializedMappoint->has_found())      pMp->mnFound     = serializedMappoint->found();
  if(serializedMappoint->has_descriptor()) deserializeDescriptor(serializedMappoint->mutable_descriptor(), &pMp->mDescriptor);
  if(serializedMappoint->has_position())   deserializePosition  (serializedMappoint->mutable_position(),   &pMp->mWorldPos  );

  return pMp;
}

/**
*/
SerializedKeyframe *Osmap::serializeKeyframe(KeyFrame *pKF, SerializedKeyframe *serializedKeyframe){
  if(!options[NO_ID]) serializedKeyframe->set_id(pKF->mnId);
  serializePose(&pKF->mTcw, &serializedKeyframe->mutable_pose());
  serializedKeyframe->set_pose(pKF->mnId);
}

/**
Reconstructs a KeyFrame from optional fields.
It doesn't perform KeyFrame initialization.  This should be done after deserialization.
*/
KeyFrame *Osmap::deserializeKeyframe(SerializedKeyframe *serializedKeyframe, KeyFrame *pKF){
  // Handle default
  if(pKF == NULL) pKF = new KeyFrame();

  if(serializedKeyframe->has_id()) pKF.mnId = serializedKeyframe->mnid();
  if(serializedKeyframe->has_pose()) deserializePose(serializedKeyframe->mutable_pose(), &pKF.mTcw);
  if(serializedKeyframe->has_k()) pKF.mK = *vectorK[serializedKeyframe->k()];

  return pKf;
}


SerializedFeature *Osmap::serializeFeature(KeyFrame *pKF, int index, SerializedFeature *serializedFeature){
  serializedFeature->set_keyframe_id(pKF->mnId);
  if(pKF->mvpMapPoints[index]) serializedFeature->set_mappoint_id(pKF->mvpMapPoints[index]->mnId);
  serializeKeypoint(&pKF->mvKeysUn[index], serializedFeature->mutable_keypoint());
  if(!options[NO_FEATURES_DESCRIPTORS]) serializeDescriptor(&pKF->mDescriptor[index], serializedFeature->mutable_descriptor());
}

KeyFrame *Osmap::deserializeFeature(SerializedFeature *serializedFeature, KeyFrame* pKF, int index){
  if(serializedFeature->has_mappoint_id()) pKF->mvpMapPoints[index] = getMapPoint(serializedFeature->mappoint_id);
  if(serializedFeature->has_keypoint())   deserializeKeypoint  (serializedFeature->mutate_keypoint(),   &pKF->mvKeysUn[index]);
  if(serializedFeature->has_descriptor()) deserializeDescriptor(serializedFeature->mutate_descriptor(), &pKF->mDescriptor[index]));
}



MapPoint *Osmap::getMapPoint(unsigned int id){
  for(
    auto it = map->mspMapPoints.begin;
    it != map->mspMapPoints.end;
    it++
  ){
    if(it->mnId == id) return *it;
  }

  // Not found
  return NULL;
}

KeyFrame *Osmap::getKeyFrame(unsigned int id){
  // Starts from itLastKF
  for(
    auto it = itLastKF;
    it != map->mspKeyFrames.end;
    it++
  ){
    if(it->mnId == id){
      itLastKF = it;
      return *it;
    }
  }

  // After the end, restarts from the begining
  for(
    auto it = map->mspKeyFrames.begin;
    it != itLastKF;
    it++
  ){
    if(it->mnId == id){
      itLastKF = it;
      return *it;
    }
  }

  // After a whole cycle, not found
  return NULL;
}



int Osmap::serializeMapPointFile(fstream *file){
  int n = 0;
  SerializedMappoint serializedMappoint;
  for(
    auto it = map.mspMapPoints.begin();
    it != map.mspMapPoints.end();
    it++
  ){
    serializeMappoint(*it, &serializedMappoint);
    writeDelimitedTo(serializedMappoint, file);
    n++;
  }
  return n;
}


int Osmap::deserializeMapPointFile(fstream *file){
  int n = 0;
  SerializedMappoint serializedMappoint;
  auto it = map.mspMapPoints.begin();

  // Here I assume an error (readDelimitedTo returns 0) means end of file
  while(readDelimitedTo(serializedMappoint, file)){
    deserializeMappoint(&serializedMappoint, *it);
    it++;
    n++;
  }
  return n;
}


int Osmap::serializeKeyFrameFile(fstream *file){
  int n = 0;
  SerializedKeyframe serializedKeyframe;
  for(
    auto it = map.mspKeyFrames.begin();
    it != map.mspKeyFrames.end();
    it++;
  ){
    serializeKeyframe(*it, &serializedKeyframe);
    writeDelimitedTo(serializedKeyframe, file);
    n++;
  }
  return n;
}

int Osmap::deserializeKeyframeFile(fstream *file){
  int n = 0;
  SerializedKeyframe serializedKeyframe;
  auto it = map.mspKeyFrames.begin();
  while(readDelimitedTo(serializedKeyframe, file)){
    deserializeKeyframe(&serializedKeyframe, *it);
    it++;
    n++;
  }
  return n;
}



int Osmap::serializeFeatureFile(fstream *file){
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

int Osmap::deserializeFeatureFile(fstream *file){
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



void Osmap::mapSave(std::string folder);
void Osmap::mapLoad(std::string folder);



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
