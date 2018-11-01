# Work in progress, not ready yet.

# Osmap

OSMAP stands for Orb-Slam2 Map.  It's a serialization addendum for ORB-SLAM2.

ORB-SLAM2 is a visual SLAM that generate a point cloud (sort of) map from a video stream, so it can localize itself in that map.  ORB-SLAM2 code in GitHub ar https://github.com/raulmur/ORB_SLAM2 is open source, and was made as a proof of concept to support its paper *ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras.*

ORB-SLAM2 is not a final product.  It generates maps but can't save them to a file, neither load them from a file.  Many proposals was made to add this functionality to ORB-SLAM2 code on GitHub, but saving maps to files ir beyond the project's porpuse.

After analizing many serializartion proposals, making one algorithm myself and having some experience with map files, I decide to restart this project from scratch and define a file format with broader goals in mind:

- Flexible evolution: let improve the format while maintaining compatibility
- Mind the size: shrink when possible, reconstruct when desirable.
- Format with options: map files can grow huge.  Let the user decide which parts to save and which parts to automatically reconstruct.
- Transparent, accesible and well documented: simplify format for debug and analisys, so third developers can do code with different porpuses than running on orb-slam2, like visualizing the map, editing it, analisyng its content, converting from a format to another, etc.  The serialization format can be used to real-time transfer the map to a visualizer via network.

This is why this serialization format has the following features:

- One map is serialized in many files.  If you want to analise mappoints only, you access that file alone.  In the future these files could be unified in a single zip file.
- Protocol buffers instead of boost::serialization.  proto files are a way to both document and generate serialization code.  Plus it applies some data compression.
- YAML as a header, mainly because ORB-SLAM2 already uses YAML, otherwise it could be JSON or XML.  That human readable header helps people who wants to analise or manipulate the map.


## The files in the project
osmap.proto is the only protocol buffer definition, that serves as format documentation.

osmap.pb.cc and osmap.pb.h are protocol buffer automatically generated code that defines messages classes.

osmap.cpp and osmap.h defines the osmap class responsible for saving and loading maps.

text folder has some test files, which create some dummy map, saves it, loads it and show its values to verify the whole process.


## How to bundle with ORB-SLAM2
Right now osmap aims monocular SLAM only, so it won't serialize some variables needed for stereo SLAM and RGB-D SLAM.  But osmap could be extended to do that.

Because I don't pretend osmap be added to Raúl Mur's ORB-SLAM2, and because that project could still receive minor changes, this is the recipe to merge osmap with up to date orb-slam2.  It need some editing and compiling.

1- Add default constructors to MapPoint and KeyFrame.
2- Add osmap as friend class to MapPoint, KeyFrame and Map, so osmap can access private and protected attributes that need to be serialized.
3- Add commands to save and load maps in the human machine interface.  One way is add buttons on visualizer.cc.  In order to call save and load, this file must include osmap.h and create one osmap instance.
4- Add Osmap files to ORB-SLAM2 project.  Copy the .cpp and .cc files to src folder, and .h files to include folder.
5- Compile and run.
