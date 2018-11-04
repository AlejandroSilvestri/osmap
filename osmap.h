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


Methods:

- serialize(object, message): this method provides many signatures for many pairs {object, message},
where object, passed by reference, is the instance to be serialized (Mat, MapPoint, KeyFrame, etc.) and message is the protocol buffers destination object.
- deserialize(message, object): this method provides many signatures for many pairs {object, message},
where message is the protocol buffers source object, and object (Mat, MapPoint, KeyFrame, etc.) is the actual destination.
For MaPPoitn and KeyFrame, object can be either passed by reference or not provided at all, the method creates an empty one, and returns it deserialized.

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
  Saves the map to a set of files in the path provided as an argument.
  If the path leads to a .yaml file name, mapSave takes it as the header file, and saves all other files in its folder.
  If path doesn't have an extension (it doesn't end with .yaml) the path is adopted as a folder, if it doesn't exists saveMap creates it, and saves the map's files in it.
  Any existing file is rewritten.
  This is the entry point to save a map.  This method uses the Osmap object to serialize the map to files.
  ORB-SLAM2 threads must be stopped before calling this method to assure map is not being modify while saving.

  @param path Path to .yaml or folder where map's files will be saved.
  */
  void mapSave(std::string path);

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
  @param k Source to be serialized.  K matrix, also known as calibration matrix, instrinsic matrix and camera matrix, 3x3 float Mat, usually from KeyFrame::mK.
  @param serializedK Protocol buffers destination message to be serialized.
  */
  void serialize(Mat& k, SerializedK& serializedK);

  /**
  Reconstruct the intrinsic camera matrix K from protocol buffer message.
  All 4 fields are required.
  @param serializedK Protocol buffers source message.
  @param k Destination to be reconstruted.  K matrix, also known as calibration matrix, instrinsic matrix and camera matrix, 3x3 float Mat, usually from KeyFrame::mK.

  */
  void deserialize(SerializedK&, Mat& k);


  void serialize(Mat&, SerializedDescriptor&);


  /**
  Reconstruct a descriptor, an 1x8 int Mat (256 bits).
  Exactly 8 int required.
  */
  void deserialize(SerializedDescriptor&, Mat&);


  /**
  Serialize a 4x4 float Mat representing a pose in homogeneous coordinates.
  Exactly 12 float required.
  */
  void serialize(Mat&, SerializedPose&);


  /**
  Reconstruct a 4x4 float Mat representing a pose in homogeneous coordinates.
  Exactly 12 float required.
  */
  void deserialize(SerializedPose&, Mat&);

  /**
  Serialize 3D mappoint position, a 3x1 float Mat.
  All 3 fields required.
  */
  void serialize(Mat&, SerializedPosition&);

  /**
  Reconstructs 3D mappoint position in a 3x1 float Mat.
  All 3 fields required.
  */
  void deserialize(SerializedPosition&, Mat&);

  /**
  Serialize 4 properties of a KeyPoint.
  All 4 fields required.
  */
  void serialize(KeyPoint&, SerializedKeypoint&);


  /**
  Reconstructs a KeyPoint.
  All 4 fields required.
  */
  void deserialize(SerializedKeypoint&, KeyPoint&);

  /**
  Serializes a MapPoint, according to options.
  */
  void serialize(MapPoint&, SerializedMappoint&);

  /**
  Creates and fills a MapPoint from optional message fields.
  It doesn't perform MapPoint initialization.  This should be done after deserialization.
  @param serializedMappoint Protocol buffers source message.
  @returns *MapPoint A pointer to the newly created MapPoint, ready to be added to the map.
  */
  MapPoint *deserialize(SerializedMappoint& serializedMappoint);

  /**
  Serialized array of MapPoints.  This can make a file, or be appended to a multiobject file.
  @param start Inclusive begining of the range of MapPoints to be serialized.  Usually map.mspMapPoints.begin().
  @param end Exclusive end of the range of MapPoints to be serialized.  Usually map.mspMapPoints.end().
  @param serializedMapPointArray message to set up.  Data comes from the range iterated.
  @returns Number of MapPoints serialized or -1 if error.  The number of MapPoints serialized should be the same number of MapPoints in the map.
  */
  int serializeMapPointArray(std::iterator<std::input_iterator_tag, MapPoint*> start, std::iterator<std::input_iterator_tag, MapPoint*> end, SerializedMapPointArray *serializedMapPointArray);

  /**
  Retrieves MapPoints from an array, and append them to the map.
  @param serializedMapPointArray message to set up.  Data goes to the output iterator.
  @param output iterator on the destination container.  Usually std::inserter(mspMapPoints, mspMapPoints.end()).  For vector it can be a Back inserter iterator where output is placed in arrival order.  Usually mspMapPoints.begin().  About set back_inserter: https://stackoverflow.com/questions/908272/stdback-inserter-for-a-stdset
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  int deserializeMapPointArray(SerializedMapPointArray *serializedMapPointArray, std::iterator<std::ouput_iterator_tag, MapPoint*> output);

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







  /**
  */
  void serialize(KeyFrame&, SerializedKeyframe&);


  /**
  Reconstructs a KeyFrame from optional fields.
  It doesn't perform KeyFrame initialization.  This should be done after deserialization.
  */
  KeyFrame *deserialize(SerializedKeyframe*);

  /**
  Serialized array of KeyFrames.  This can make a file, or be appended to a multiobject file.
  @param serializedKeyFrameArray message to set up.  Data comes from map.
  @returns Number of KeyFrames serialized or -1 if error.  The number of KeyFrames serialized should be the same number of MapPoints in the map.
  */
  int serialize(std::iterator<std::input_iterator_tag, KeyFrame*> start, std::iterator<std::input_iterator_tag, KeyFrame*> end, SerializedKeyFrameArray *serializedKeyFrameArray);


  /**
  Retrieves MapPoints from an array, and append them to the map.
  @param serializedMapPointArray message to set up.  Data goes from map.
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  int deserializeKeyFrameArray(SerializedKeyFrameArray *serializedKeyFrameArray, std::iterator<std::ouput_iterator_tag, MapPoint*> output);


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


  // Feature ====================================================================================================

  /**
  Serialize all features observed by a KeyFrame.
  @param pKF KeyFrame owner of the feature.  The features will be stored in this KeyFrame containers.
  @param SerializedKeyframeFeatures Message destination of serialization.
  @returns The serialized message object.
  */
  void serialize(KeyFrame&, SerializedKeyframeFeatures&);

  /**
  Retrieves one feature.  Keyframe must be provided, and to that end keyframe_id must be deserialized before.
  keyframe_id required.
  Puts the KeyPoint, MapPoint* and descriptor in their respective containters of the provided KeyFrame, at the index provided.
  Containers must have that index available, no check is performed in this regard.
  @param pKF KeyFrame owner of the feature.  The features will be stored in this KeyFrame's containers.
  @param index
  */
  KeyFrame *deserialize(SerializedKeyframeFeatures&);

  int serialize(std::iterator<std::input_iterator_tag, KeyFrame*> start, std::iterator<std::input_iterator_tag, KeyFrame*> end, SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray);

  int deserialize(SerializedKeyframeFeaturesArray &serializedKeyframeFeaturesArray, std::iterator<std::ouput_iterator_tag, MapPoint*> output);



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
