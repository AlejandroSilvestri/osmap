#ifndef OSMAP_H_
#define OSMAP_H_

#include <string>
#include <vector>
#include "osmap.pb.h"
class Map;
class Mat;
class KeyPoint;
class MapPoint;
class KeyFrame;



/**
This is a class for a singleton attached to ORB-SLAM2's map.

Its constructor attaches it to the map.  Its methods mapLoad and mapSave load and save a map.

A map is saved to a set of files in a named folder:
- key frames.osmap
- map points.osmap
- features.osmap
- K.osmap
- osmap.yaml, the header

*.osmap files store binary data using an adaptation of protocol buffer.

Map serialization implies serializing each of these .somap files with the appropiate method, like Osmap::serializeMapPointFile.

This file serialization loops serializing object in protocol buffer messages, like serializeMapPoint.

Some of these objects has another object like KeyPoint, nested serialized with the appropiate serializeKeypoint.


*/
class Osmap{
  // Properties

  /** All the options available.*/
  enum class Options {ONLY_MAPPOINTS_FEATURES, NO_ID, SAVE_TIMESTAMP, NO_LOOPS, NO_FEATURES_DESCRIPTORS};

  /**
  Set of chosen options for serializing.  User must set options prior to saving a map.  Loading a map should reflect in this property the saved map options.
  */
  bitset<32> options = 0;

  /** ORB-SLAM2 map to be serialized. */
  Map *map;

  /**
  Usually there is only one common matrix K for all KeyFrames in the entire map, there can be more, but there won't be as many K as KeyFrames.
  This vector temporarily store different K for setiralization.  This avoids serializing one K per KeyFrame.
  */
  vector<Mat*> vectorK;

  /**
  Iterador apuntando al último KeyFrame encontrado.
  Variable pseudoestática usada en Osmap::getKeyFrame , que requiere ser inicializada fuera del método.
  */
  std::set<KeyFrame*>::iterator itLastKF;


  /* Methods, documented on code file.*/

  /**
  Saves the map to a set of files in the folder whose name is provided as an argument.
  If the folder doesn't exist, it creates one.  If the folder exists, it uses it an rewrites the files, para it doesn't erase other files.
  This is the entry point to save a map.  This method uses the Osmap object to serialize the map to files.
  */
  void mapSave(std::string);

  /**
  Loads the map from a set of files in the folder whose name is provided as an argument.
  This is the entry point to load a map.  This method uses the Osmap object to serialize the map to files.
  */
  void mapLoad(std::string);

  /*
  Preprocesador en línea.  En la línea de comando gcc quitar && -a.o y agregar -E
  g++ -std=c++17 -O2 -Wall -pedantic -pthread main.cpp -E
  http://coliru.stacked-crooked.com/

  #define SER(ser, obj) \
    Serialized ## ser *serialize ## ser(obj*, Serialized ## ser *op = NULL);\
    obj *deserialize ## ser(Serialized ## ser*, obj *op = NULL);

  SER(K, Mat)
  SER(Descriptor, Mat)
  SER(Pose, Mat)
  SER(Position, Mat)
  SER(Keypoint, KeyPoint)
  SER(Mappoint, MapPoint)
  SER(Keyframe, KeyFrame)
  */

  // Protocol buffer messages serialization for orb-slam2 objects


  /**
  Serialize provided intrinsic camera matrix K to the protocol buffer message.
  All 4 fields required.
  */
  SerializedK *serializeK(Mat*, SerializedK *op = NULL);

  /**
  Reconstruct the intrinsic camera matrix K from protocol buffer message.
  All 4 fields are required.
  */
  Mat *deserializeK(SerializedK*, Mat *op = NULL);


  SerializedDescriptor *serializeDescriptor(Mat*, SerializedDescriptor *op = NULL);


  /**
  Reconstruct a descriptor, an 1x8 int Mat (256 bits).
  Exactly 8 int required.
  */
  Mat *deserializeDescriptor(SerializedDescriptor*, Mat *op = NULL);


  /**
  Serialize a 4x4 float Mat representing a pose in homogeneous coordinates.
  Exactly 12 float required.
  */
  SerializedPose *serializePose(Mat*, SerializedPose *op = NULL);


  /**
  Reconstruct a 4x4 float Mat representing a pose in homogeneous coordinates.
  Exactly 12 float required.
  */
  Mat *deserializePose(SerializedPose*, Mat *op = NULL);

  /**
  Serialize 3D mappoint position, a 3x1 float Mat.
  All 3 fields required.
  */
  SerializedPosition *serializePosition(Mat*, SerializedPosition *op = NULL);

  /**
  Reconstructs 3D mappoint position in a 3x1 float Mat.
  All 3 fields required.
  */
  Mat *deserializePosition(SerializedPosition*, Mat *op = NULL);

  /**
  Serialize 4 properties of a KeyPoint.
  All 4 fields required.
  */
  SerializedKeypoint *serializeKeypoint(KeyPoint*, SerializedKeypoint *op = NULL);


  /**
  Reconstructs a KeyPoint.
  All 4 fields required.
  */
  KeyPoint *deserializeKeypoint(SerializedKeypoint*, KeyPoint *op = NULL);

  /**
  Serializes a MapPoint, according to options.
  */
  SerializedMappoint *serializeMappoint(MapPoint*, SerializedMappoint *op = NULL);

  /**
  Reconstructs a MapPoint from optional fields.
  It doesn't perform MapPoint initialization.  This should be done after deserialization.
  */
  MapPoint *deserializeMappoint(SerializedMappoint*, MapPoint *op = NULL);

  /**
  Serialized array of MapPoints.  This can make a file, or be appended to a multiobject file.
  @param start Inclusive begining of the range of MapPoints to be serialized.  Usually map.mspMapPoints.begin().
  @param end Exclusive end of the range of MapPoints to be serialized.  Usually map.mspMapPoints.end().
  @param serializedMapPointArray message to set up.  Data comes from the range iterated.
  @returns Number of MapPoints serialized or -1 if error.  The number of MapPoints serialized should be the same number of MapPoints in the map.
  */
  int serializeMapPointArray(InputIterator start, InputIterator end, SerializedMapPointArray *serializedMapPointArray);

  /**
  Retrieves MapPoints from an array, and append them to the map.
  @param serializedMapPointArray message to set up.  Data goes to the output iterator.
  @param output iterator on the destination container.  Usually std::inserter(mspMapPoints, mspMapPoints.end()).  For vector it can be a Back inserter iterator where output is placed in arrival order.  Usually mspMapPoints.begin().  About set back_inserter: https://stackoverflow.com/questions/908272/stdback-inserter-for-a-stdset
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  int deserializeMapPointArray(SerializedMapPointArray *serializedMapPointArray, OutputIterator output);

  /**
  Saves MapPoints to file.
  @param file output stream of the file being written.
  @returns Number of MapPoints serialized or -1 if error.  The number of MapPoints serialized should be the same number of MapPoints in the map.
  */
  int serializeMapPointFile(fstream*);

  /**
  Retrieves MapPoints from a file.
  @param file input stream of the file being read.
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  int deserializeMapPointFile(fstream *file);




  SerializedKeyframe *serializeKeyframe(KeyFrame*, SerializedKeyframe *op = NULL);
  KeyFrame *deserializeKeyframe(SerializedKeyframe*, KeyFrame *op = NULL);

  /**
  Serialized array of KeyFrames.  This can make a file, or be appended to a multiobject file.
  @param serializedKeyFrameArray message to set up.  Data comes from map.
  @returns Number of KeyFrames serialized or -1 if error.  The number of KeyFrames serialized should be the same number of MapPoints in the map.
  */
  int serializeKeyFrameArray(SerializedKeyFrameArray *serializedKeyFrameArray);


  /**
  Retrieves MapPoints from an array, and append them to the map.
  @param serializedMapPointArray message to set up.  Data goes from map.
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  int deserializeKeyFrameArray(SerializedKeyFrameArray *serializedKeyFrameArray);


  /**
  Saves KeyFrames to file.
  @param file output stream of the file being written.
  @returns Number of KeyFrames serialized or -1 if error.  The number of KeyFrames serialized should be the same number of KeyFrames in the map.
  */
  int serializeKeyFrameFile(fstream *file);

  /**
  Retrieves Keyframes from a file.
  @param file input stream of the file being read.
  @returns Number of Keyframes retrieved or -1 if error.
  Map's Keyframes set should be emptied before calling this method.
  */
  int deserializeKeyframeFile(fstream *file);


  // Feature

  /**
  Serialize one feature.
  Puts the KeyPoint, MapPoint* and descriptor in their respective containter, at the index provided.
  Containers must have that index available, no check is performed in this regard.
  @param pKF KeyFrame owner of the feature.  The features will be stored in this KeyFrame containers.
  @param i
  */
  SerializedFeature *serializeFeature(KeyFrame *pKF, SerializedFeature *serializedFeature);

  /**
  Retrieves one feature.  Keyframe must be provided, and to that end keyframe_id must be deserialized before.
  keyframe_id required.
  Puts the KeyPoint, MapPoint* and descriptor in their respective containters of the provided KeyFrame, at the index provided.
  Containers must have that index available, no check is performed in this regard.
  @param pKF KeyFrame owner of the feature.  The features will be stored in this KeyFrame's containers.
  @param index
  */
  KeyFrame *deserializeFeature(SerializedFeature *serializedFeature, KeyFrame *pKF);

  /**
  Writes a message to a vector file.  A vector file is an array of messages.
  In order to possibilitate deserialization, the message size is prepended to the message.
  @param message
  @param rawOutput output stream, writable destination file.
  @returns true if ok, false if error.

  Kendon Varda code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja
  */
  bool writeDelimitedTo(
    const google::protobuf::MessageLite& message,
    google::protobuf::io::ZeroCopyOutputStream* rawOutput
  );

  /**
  Reads a message to a vector file.  A vector file is an array of messages.
  @param message
  @param rawOutput input stream, readable source file.
  @returns true if ok, false if error.

  Kendon Varda code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja
  */
  bool readDelimitedFrom(
    google::protobuf::io::ZeroCopyInputStream* rawInput,
    google::protobuf::MessageLite* message
  );







  /**
  Saves KeyFrame's features.
  @param file output stream of the file being written.
  @returns Number of features serialized or -1 if error.  The number of features serialized should be the same number of features in all KeyFrames in the map.
  */
  int serializeFeatureFile(fstream *file);

  /**
  Retrieves KeyFrame's features from a file.
  @param file input stream of the file being read.
  @returns Number of features retrieved or -1 if error.  The number of features serialized should be the same number of features in all KeyFrames in the map.
  All KeyFrames should be deserialized and put in map before calling this method.
  */
  int deserializeFeatureFile(fstream *file);



  /**
  Looks for a mappoint by its id in the map.
  @param id Id of the MapPoint to look for.
  @returns a pointer to the MapPoint with the given id, or NULL if not found.
  */
  MapPoint *getMapPoint(unsigned int id);


  /**
  Looks for a KeyFrame id in the map.
  Keyframes are usually stored in ascending id order.  This function will be more probably called in the same way, so it is optimized for this expected behaviour.  It uses itLastKF.
  @param id Id of the KeyFrame to look for.
  @returns a pointer to the KeyFrame with the given id, or NULL if not found.
  */
  KeyFrame *getKeyFrame(unsigned int id);


  /**
  Only constructor, the only way to set the orb-slam2 map.
  */
  Osmap(Map*);
}

#endif /* OSMAP_H_ */
