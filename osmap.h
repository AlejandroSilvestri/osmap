#ifndef OSMAP_H_
#define OSMAP_H_

//#include <string>
#include <set>
#include <vector>
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

/**
 * FEATURES_MESSAGE_LIMIT is the maximum number of features allowed in a single protocol buffer's message, to avoid known size problems in protocol buffers.
 * When a map exceed this limit, mapSave will save features file in delimited form, sequencing many messages each of them below this limit.
 * This constant can be defined elsewhere before this point, or else is defined here.
 */
#ifndef FEATURES_MESSAGE_LIMIT
#define FEATURES_MESSAGE_LIMIT 1000000
#endif

using namespace std;
using namespace cv;

/**
This is a class for a singleton attached to ORB-SLAM2's map.

Its constructor attaches it to the map.  Its methods mapLoad and mapSave load and save a map.

A map is saved to a set of files in the actual folder, with a filename provided:
- filename.keyframes
- filename.mappoints
- filename.features
- filename.yaml, the header

These files execpt .yaml store binary data using an adaptation of protocol buffer.

Some of these objects has another object like KeyPoint, nested serialized with the appropiate serialize signature.


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

  - NO_ID: Do not save mappoints id and keyframes id.  It shrinks mappoints and keyframes a little.  When loading, ids will be assigned automatically in increasing order.  Map will work perfectly.  The only drawback is the lack of traceability between two map instances.
  - NO_LOOPS: Don't save loop closure data, for debugging porpuses.
  - NO_FEATURES_DESCRIPTORS: Don't save descriptors in features file. Mappoints descriptors will be saved instead.  Descriptors take a huge amount of bytes, and this will shrink thw whole map a lot.  Not sure about drawbacks.

  - K_IN_KEYFRAME: Save K camera matrix in each keyframe, and not in YAML file.  By default K is saved in yaml file.  Usually maps has only one K or few different K.  This option is usefull when each keyframe has a different K.
  - ONLY_MAPPOINTS_FEATURES: Do not save features not associated to mappoints.  It shrinks the map a lot.  Keyframes will not be suitable to add new mappoints.  Usefull for tracking only.

  - FEATURES_FILE_DELIMITED: Features file is delimited, Kenda Varda's function is needed to retrieve.  On save, force delimited.
  - FEATURES_FILE_NOT_DELIMITED: Features file is not delimited, file can be read with protocol buffers in the usual way.  On save, force not delimited.  If any of these is set on save, the decision is automatic.

  - NO_MAPPOINTS_FILE: Avoid saving MapPoints file.  Useful when analysing some other file, to save serialization time.
  - NO_KEYFRAMES_FILE: Avoid saving KeyFrames file.  Useful when analysing some other file, to save serialization time.
  - NO_FEATURES_FILE:   Avoid saving Features file.  Useful when analysing some other file, to save serialization time.

  New options can be added in the future, but the order of existing options must be keeped.  So, new options must be added to the end of enum.

  Time stamp is not optional because it always occupy place in protocol buffers, as all numeric fields.  Defaults to 0.
  */
  enum Options {
	  // Delimited to overcome a Protocol Buffers limitation.  It is automatic, but can be forced with this options:
	  FEATURES_FILE_DELIMITED,	// Implemented, load fails
	  FEATURES_FILE_NOT_DELIMITED,	// Implemented

	  // Skip files, for analisys propuses
	  NO_MAPPOINTS_FILE,	// Implemented
	  NO_KEYFRAMES_FILE,	// Implemented
	  NO_FEATURES_FILE, 	// Implemented

	  // Shrink file
	  NO_FEATURES_DESCRIPTORS,	// Implemented. Implicit in file, flag ignored on load.
	  ONLY_MAPPOINTS_FEATURES,	// Implemented. Implicit in file, flag ignored on load.

	  // Options
	  NO_LOOPS,	// Pending serializing loops
	  K_IN_KEYFRAME,	// Implemented. Implicit in file, flag ignored on load.
	  //RENUM_ID,	// Pending in depuration. Implicit in file, flag ignored on load.  Not clear what is the benefit, since id are saved to file anyway.

	  // Depuration options
	  NO_DEPURATION,
	  NO_ERASE_BAD_MAPPOINTS,
	  NO_APPEND_FOUND_MAPPOINTS,
	  NO_ERASE_ORPHAN_KEYFRAME_IN_LOOP,


	  OPTIONS_SIZE	// Not an option
  };

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
   * Buffer where map's mappoints are stored in ascending id order, to save them to file in this order.
   * This vector is set in mapSave, and it's left ontouched for user interest.
   * This vector is consumed in serialize
   */
  vector<MapPoint*> vectorMapPoints;

  /**
   * Buffer where map's keyframes are stored in ascending id order, to save them to file in this order.
   * This vector is set in mapSave, and it's left ontouched for user interest.
   */
  vector<KeyFrame*> vectorKeyFrames;

  /**
  Iterator pointing to last found KeyFrame.
  Variable pseudoestática usada en Osmap::getKeyFrame , que requiere ser inicializada fuera del método.
  */
  //set<KeyFrame*>::iterator itLastKF;


  /* Methods, documented on code file.*/

  /**
  Only constructor, the only way to set the orb-slam2 map.
  */
  Osmap(Map &_map): map(_map){}


  /**
   * Irons keyframes and mappoints sets in map, before save.
   *
   * Tests all keyframes in MapPoints.mObservations and mappoints in KeyFrames.mvpMapPoints, checking they are:
   *
   * - not bad (!isBad())
   * - in map (in Map::mspMapPoints and Map::mspKeyFrames)
   *
   * Those who not pass this check are eliminated, avoiding serialization of elements not belonging to the map.
   *
   * This depuration affects (improves) the actual map in memory.
   *
   * Invoked by mapSave.
   *
   */
  void depurate();

  void rebuild();

  /**
  Saves the map to a set of files in the actual directory, with the extensionless name provided as the only argument and different extensions for each file.
  If the path leads to a .yaml file name, mapSave takes it as the header file, and saves all other files in its folder.
  If path doesn't have an extension (it doesn't end with .yaml) the path is adopted as a folder, if it doesn't exists saveMap creates it, and saves the map's files in it.
  Any existing file is rewritten.
  This is the entry point to save a map.  This method uses the Osmap object to serialize the map to files.
  Before calling this method:
  - ORB-SLAM2 threads must be stopped to assure map is not being modify while saving.
  - Actual directory must be set to the desired destination.  Often a new directory exclusive for the map is created.
  - options must be set.

  @param basefilename File name without extenion.  Many files will be created with this filename and different extensions.

  MapSave copy map's mappoints and keyframes sets to vectorMapPoints and vectorKeyFrames and sort them, to save objects in ascending id order.
  MapLoad doesn't use those vector.

  If features number exceed an arbitrary maximum, in order to avoid size related protocol buffer problems,  mapSave limit the size of protocol buffer's messages saving features file in delimited form, using Kendon Varda writeDelimitedTo function.
  */
  void mapSave(std::string basefilename);

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
   * Count the number of features in vectorKeyFrames.
   * Invoked by mapSave to decide to use or not delimited form of feature file.
   */
  int countFeatures();



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
   * TODO: doc
  Serialized array of MapPoints.  This can make a file, or be appended to a multiobject file.
  @param start Inclusive begining of the range of MapPoints to be serialized.  Usually map.mspMapPoints.begin().
  @param end Exclusive end of the range of MapPoints to be serialized.  Usually map.mspMapPoints.end().
  @param serializedMapPointArray message to set up.  Data comes from the range iterated.
  @returns Number of MapPoints serialized or -1 if error.  The number of MapPoints serialized should be the same number of MapPoints in the map.
  */
  //int serialize(iterator<input_iterator_tag, MapPoint*> start, iterator<input_iterator_tag, MapPoint*> end, SerializedMappointArray &serializedMapPointArray);
  //int serialize(const set<MapPoint*>&, SerializedMappointArray &);
  int serialize(const vector<MapPoint*>&, SerializedMappointArray &);

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
  //int serialize(const set<KeyFrame*>&, SerializedKeyframeArray&);
  int serialize(const vector<KeyFrame*>&, SerializedKeyframeArray&);

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
  Retrieves all features belonging to one keyframe.
  First it takes the required keyframe_id field, and look in map for the keyframe with that id.  Hence, keyframes must be already deserialized in map.

  Puts all KeyPoints, MapPoints* and descriptors in their respective containters of the provided KeyFrame.

  @param SerializedKeyframeFeatures Object to be deserialized.
  */
  KeyFrame *deserialize(const SerializedKeyframeFeatures&);

  /**
   * Serialize all keyframe's features from provided keyframes container, to the specified serialization object.
   * @param vKF vector of KeyFrames to save, usually vectorKeyFrames member order by mnId.
   * @param serializedKeyframeFeaturesArray output serialization object.
   */
  int serialize(const vector<KeyFrame*>& vKF, SerializedKeyframeFeaturesArray& serializedKeyframeFeaturesArray);

  /**
   * Retrieves all keyframe's features to provided keyframes container, from the specified serialization object.
   */
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
};

#endif /* OSMAP_H_ */
