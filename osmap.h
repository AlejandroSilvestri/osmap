#ifndef OSMAP_H_
#define OSMAP_H_

//#include <string>
#include <set>
#include <bitset>
#include <iterator>
#include "osmap.pb.h"

/* For debuging and examples porpouses, dummymap.h contains minimal definitions of these orb-slam2 classes:
 * - Map
 * - MapPoint
 * - KeyFrame
 *
 * dummymap.h allows osmap to run without orb-slam.  This is handy for:
 * - debuging porpouses
 * - running the examples
 * - implementing your own map analyser
 *
 * This way you should be able to serialize maps in your own application without the need of including the whole orb-slam2 code.
 *
 * In order to append osmap to orb-slam2, this include must be replaced by the following:

#include <set>
#include <opencv2/core.hpp>
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

 */
#include "dummymap.h"

using namespace std;
using namespace cv;

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


Methods:

- serialize(object, message): this method provides many signatures for many pairs {object, message},const
where object, passed by reference, is the instance to be serialized (Mat, MapPoint, KeyFrame, etc.) and message is the protocol buffers destination object.
- deserialize(message, object): this method provides many signatures for many pairs {object, message},
where message is the protocol buffers source object, and object (Mat, MapPoint, KeyFrame, etc.) is the actual destination.
For MapPoint and KeyFrame, object can be either passed by reference or not provided at all, the method creates an empty one, and returns it deserialized.
- messageArray is a message with only one field of repeated messages.  messageArray serialized MapPoints, KeyFrames, Features...

General rules:
- deserialize get a const message
- serialize need a mutable message, it asks for a pointer to mutable message, as provided by protocol buferrs' mutable_field and add_field
- serializeArray and deserializeArray serialize a messageArray, i.e. set of objects
- unlike serialize, serializeArray uses a messageArray, not a pointer to it.


*/
class Osmap{
public:
  // Properties

  /**
  All the options available.
  Options are persetted into options property.
  Default is zero.  Options are activated with 1.

  To test an option:
  if(options[ONLY_MAPPOINTS_FEATURES])...

  To set an option:
  options.set(ONLY_MAPPOINTS_FEATURES);

  - ONLY_MAPPOINTS_FEATURES: Do not save features not associated to mappoints.  It shrinks the map a lot.  Keyframes will not be suitable to add new mappoints.  Usefull for tracking only.
  - No_ID: Do not save mappoints id and keyframes id.  It shrinks mappoints and keyframes a little.  When loading, ids will be assigned automatically in increasing order.  Map will work perfectly.  The only drawback is the lack of traceability between two map instances.
  - SAVE_TIMESTAMP: Not saved by default, it will increase a little keyframes file.  It is usefull only if you will use timestamps.  ORB-SLAM2 doesn't use them.
  - NO_LOOPS: Don't save loop closure data, for debugging porpuses.
  - NO_FEATURES_DESCRIPTORS: Don't save descriptors in features file. Mappoints descriptors will be saved instead.  Descriptors take a huge amount of bytes, and this will shrink thw whole map a lot.  Not sure about drawbacks.
  - K_IN_KEYFRAME: Save K camera matrix in each keyframe, and not in YAML file.  By default K is saved in yaml file.  Usually maps has only one K or few different K.  This option is usefull when each keyframe has a different K.
  - NO_MAPPOINTS_FILE: Avoid saving MapPoints file.  Useful when analysing some other file, to save serialization time.
  - NO_KEYFRAMES_FILE: Avoid saving KeyFrames file.  Useful when analysing some other file, to save serialization time.
  - NO_FEATURES_FILE:   Avoid saving Features file.  Useful when analysing some other file, to save serialization time.

  New options can be added in the future, but the order of existing options must be keeped.  So, new options must be added to the end of enum.
  */
  enum Options {ONLY_MAPPOINTS_FEATURES, NO_ID, SAVE_TIMESTAMP, NO_LOOPS, NO_FEATURES_DESCRIPTORS, K_IN_KEYFRAME, NO_MAPPOINTS_FILE, NO_KEYFRAMES_FILE, NO_FEATURES_FILE};

  /**
  Set of chosen options for serializing.  User must set options prior to saving a map.  Loading a map should reflect in this property the saved map options.
  */
  bitset<32> options = 0;

  /** ORB-SLAM2 map to be serialized. */
  Map &map;

  /**
  Usually there is only one common matrix K for all KeyFrames in the entire map, there can be more, but there won't be as many K as KeyFrames.
  This vector temporarily store different K for serialization and deserialization.  This avoids serializing one K per KeyFrame.
  This vector is consumed in keyframe deserialization.
  */
  vector<Mat*> vectorK;

  /**
  For each index KeyFrame.mnId the value returned is an index to vectorK.
  The later is the index saved in each keyframe as k.
  It is populated only by getVectorKFromKeyframes, and consumed only while saving the map, during keyframe serialization.
  */
  vector<unsigned int> keyframeid2vectork;

  /**
  Iterador apuntando al último KeyFrame encontrado.
  Variable pseudoestática usada en Osmap::getKeyFrame , que requiere ser inicializada fuera del método.
  */
  set<KeyFrame*>::iterator itLastKF;


  /* Methods, documented on code file.*/

  /**
  Saves the map to a set of files in the actual directory, with the extensionless name provided as the only argument and different extensions for each file.
  If the path leads to a .yaml file name, mapSave takes it as the header file, and saves all other files in its folder.
  If path doesn't have an extension (it doesn't end with .yaml) the path is adopted as a folder, if it doesn't exists saveMap creates it, and saves the map's files in it.
  Any existing file is rewritten.
  This is the entry point to save a map.  This method uses the Osmap object to serialize the map to files.
  Before calling this method:
  - ORB-SLAM2 threads must be stopped to assure map is not being modify while saving.
  - Actual directory must be set to the desired destination.  Often a new directory exclusive for the map is created.

  @param path Path to .yaml or folder where map's files will be saved.
  */
  void mapSave(std::string path);

  /**
  Loads the map from a set of files in the folder whose name is provided as an argument.
  This is the entry point to load a map.  This method uses the Osmap object to serialize the map to files.
  Only these properties are read from yaml:
  - file nKeyframes
  - options
  - camera calibration matrices K
  */
  void mapLoad(std::string);

  /**
  Traverse map's keyframes looking for different K matrices, and stores them into vectorK.
  It also populates keyframeid2vectork.
  */
  void getVectorKFromKeyframes();

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


  // K matrix ====================================================================================================

  /**
  Serialize provided intrinsic camera matrix K to the protocol buffer message.
  All 4 fields required.
  @param k Source to be serialized.  K matrix, also known as calibration matrix, instrinsic matrix and camera matrix, 3x3 float Mat, usually from KeyFrame::mK.
  @param serializedK Protocol buffers destination message to be serialized.
  */
  void serialize(const Mat &k, SerializedK *serializedK);

  /**
  Reconstruct the intrinsic camera matrix K from protocol buffer message.
  All 4 fields are required.
  @param serializedK Protocol buffers source message.
  @param k Destination to be reconstruted.  K matrix, also known as calibration matrix, instrinsic matrix and camera matrix, 3x3 float Mat, usually from KeyFrame::mK.

  */
  void deserialize(const SerializedK&, Mat& k);


  /**
  Serialize an array of camera calibration matrix K.
  The array should be retrieved from keyframes analisys.
  Usually there is only one K common to all keyframes.
  @param vK The vector of K matrices to serialize.
  @param serializedKArray The serialization destination object.
  */
  void serialize(const vector<Mat*> &vK, SerializedKArray &serializedKArray);


  /**
  Reconstruct an array of camera calibration matrix K.
  The array should be retrieved from keyframes analisys.
  Usually there is only one K common to all keyframes.
  @param serializedKArray The serialization destination object.
  @param vK The vector of K matrices to serialize.  Usually Osmap::vectorK member.

  */
  void deserialize(const SerializedKArray &serializedKArray, vector<Mat*> &vK);


  // Descriptor ====================================================================================================

  /**
  Serializes a descriptor, an 1x8 int Mat (256 bits).
  Exactly 8 int required.
  */
  void serialize(const Mat&, SerializedDescriptor*);


  /**
  Reconstruct a descriptor, an 1x8 int Mat (256 bits).
  Exactly 8 int required.
  */
  void deserialize(const SerializedDescriptor&, Mat&);



  // Pose matrix ====================================================================================================

  /**
  Serialize a 4x4 float Mat representing a pose in homogeneous coordinates.
  Exactly 12 float required.
  */
  void serialize(const Mat&, SerializedPose*);


  /**
  Reconstruct a 4x4 float Mat representing a pose in homogeneous coordinates.
  Exactly 12 float required.
  */
  void deserialize(const SerializedPose&, Mat&);


  // Position vector ====================================================================================================

  /**
  Serialize 3D mappoint position, a 3x1 float Mat.
  All 3 fields required.
  */
  void serialize(const Mat&, SerializedPosition*);

  /**
  Reconstructs 3D mappoint position in a 3x1 float Mat.
  All 3 fields required.
  */
  void deserialize(const SerializedPosition&, Mat&);


  // KeyPoint ====================================================================================================
  /**
  Serialize 4 properties of a KeyPoint.
  All 4 fields required.
  */
  void serialize(const KeyPoint&, SerializedKeypoint*);


  /**
  Reconstructs a KeyPoint.
  All 4 fields required.
  */
  void deserialize(const SerializedKeypoint&, KeyPoint&);






  // MapPoint ====================================================================================================

  /**
  Serializes a MapPoint, according to options.
  */
  void serialize(const MapPoint&, SerializedMappoint*);

  /**
  Creates and fills a MapPoint from optional message fields.
  It doesn't perform MapPoint initialization.  This should be done after deserialization.
  @param serializedMappoint Protocol buffers source message.
  @returns *MapPoint A pointer to the newly created MapPoint, ready to be added to the map.
  */
  MapPoint *deserialize(const SerializedMappoint& serializedMappoint);

  /**
  Serialized array of MapPoints.  This can make a file, or be appended to a multiobject file.
  @param start Inclusive begining of the range of MapPoints to be serialized.  Usually map.mspMapPoints.begin().
  @param end Exclusive end of the range of MapPoints to be serialized.  Usually map.mspMapPoints.end().
  @param serializedMapPointArray message to set up.  Data comes from the range iterated.
  @returns Number of MapPoints serialized or -1 if error.  The number of MapPoints serialized should be the same number of MapPoints in the map.
  */
  //int serialize(iterator<input_iterator_tag, MapPoint*> start, iterator<input_iterator_tag, MapPoint*> end, SerializedMappointArray &serializedMapPointArray);
  int serialize(const set<MapPoint*>&, SerializedMappointArray &);

  /**
  Retrieves MapPoints from an array, and append them to the map.
  @param serializedMapPointArray message to set up.  Data goes to the output iterator.
  @param output iterator on the destination container.  Usually std::inserter(mspMapPoints, mspMapPoints.end()).  For vector it can be a Back inserter iterator where output is placed in arrival order.  Usually mspMapPoints.begin().  About set back_inserter: https://stackoverflow.com/questions/908272/stdback-inserter-for-a-stdset
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  //int deserialize(const SerializedMappointArray &serializedMapPointArray, iterator<output_iterator_tag, MapPoint*> output);
  int deserialize(const SerializedMappointArray &, set<MapPoint*>&);
  /**
  Saves MapPoints to file.
  @param file output stream of the file being written.
  @returns Number of MapPoints serialized or -1 if error.  The number of MapPoints serialized should be the same number of MapPoints in the map.
  */
  //int serializeMapPointFile(fstream*);

  /**
  Retrieves MapPoints from a file.
  @param file input stream of the file being read.
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  //int deserializeMapPointFile(fstream *file);





  // KeyFrame ====================================================================================================

  /**
  */
  void serialize(const KeyFrame&, SerializedKeyframe*);


  /**
  Reconstructs a KeyFrame from optional fields.
  It doesn't perform KeyFrame initialization.  This should be done after deserialization.
  */
  KeyFrame *deserialize(const SerializedKeyframe&);

  /**
  Serialized array of KeyFrames.  This can make a file, or be appended to a multiobject file.
  @param serializedKeyFrameArray message to set up.  Data comes from map.
  @returns Number of KeyFrames serialized or -1 if error.  The number of KeyFrames serialized should be the same number of MapPoints in the map.
  */
  int serialize(const set<KeyFrame*>&, SerializedKeyframeArray&);

  /**
  Retrieves MapPoints from an array, and append them to the map.
  @param serializedMapPointArray message to set up.  Data goes from map.
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  int deserialize(const SerializedKeyframeArray&, set<KeyFrame*>&);


  /**
  Saves KeyFrames to file.
  @param file output stream of the file being written.
  @returns Number of KeyFrames serialized or -1 if error.  The number of KeyFrames serialized should be the same number of KeyFrames in the map.
  */
  //int serializeKeyFrameFile(fstream *file);

  /**
  Retrieves Keyframes from a file.
  @param file input stream of the file being read.
  @returns Number of Keyframes retrieved or -1 if error.
  Map's Keyframes set should be emptied before calling this method.
  */
  //int deserializeKeyframeFile(fstream *file);


  // Feature ====================================================================================================

  /**
  Serialize all features observed by a KeyFrame.
  @param pKF KeyFrame owner of the feature.  The features will be stored in this KeyFrame containers.
  @param SerializedKeyframeFeatures Message destination of serialization.
  @returns The serialized message object.
  */
  void serialize(const KeyFrame&, SerializedKeyframeFeatures*);

  /**
  Retrieves one feature.  Keyframe must be provided, and to that end keyframe_id must be deserialized before.
  keyframe_id required.
  Puts the KeyPoint, MapPoint* and descriptor in their respective containters of the provided KeyFrame, at the index provided.
  Containers must have that index available, no check is performed in this regard.
  @param pKF KeyFrame owner of the feature.  The features will be stored in this KeyFrame's containers.
  @param index
  */
  KeyFrame *deserialize(const SerializedKeyframeFeatures&);

  //int serialize(iterator<input_iterator_tag, KeyFrame*> start, iterator<input_iterator_tag, KeyFrame*> end, SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray);
  int serialize(const set<KeyFrame*>&, SerializedKeyframeFeaturesArray&);

  //int deserialize(const SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray, iterator<output_iterator_tag, MapPoint*> output);
  int deserialize(const SerializedKeyframeFeaturesArray&, set<KeyFrame*>&);


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
  //int serializeFeatureFile(fstream *file);

  /**
  Retrieves KeyFrame's features from a file.
  @param file input stream of the file being read.
  @returns Number of features retrieved or -1 if error.  The number of features serialized should be the same number of features in all KeyFrames in the map.
  All KeyFrames should be deserialized and put in map before calling this method.
  */
  //int deserializeFeatureFile(fstream *file);



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
  Used only in Osmap::deserialize(const SerializedKeyframeFeatures&).
  */
  KeyFrame *getKeyFrame(unsigned int id);





  /**
  Only constructor, the only way to set the orb-slam2 map.
  */
  Osmap(Map &_map): map(_map){}

};

#endif /* OSMAP_H_ */
