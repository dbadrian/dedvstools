# Tools for the depth/color augmented Embedded Dynamic Vision Sensor and Asus Xtion Pro Live

This "library" offers several tools:
* DEdvs: Library building upon the Edvstools by David Weikersdorf to support "Depth Events"
* RecordCalibrationData: Records calibration data, which can be used with e.g. NNCalib (based on FANN) to train a neural network, to learn the mapping between Xtion->Edvs
* ConvertCalibData: The calibration data is delivered as TSV files, which need to be converted to a suitable format for the FANN library/NNCalib to train the ANN
* DEdvsViewer: To evaluate the mapping "learned" by the neural network via video streams (depth, mapping, depth events)
* TO DO :)

# Installation

## Requirements
* Boost 
* OpenCV2
* OpenNI2 (if compiled: possibly needs a slight modifaction as DEdvstools are compiled with C++11 flag, but OpenNI2 still uses the deprecated linux macro instead of __linux__)
* FANN
* Your build essentials such as: gcc/g++ (should be recent due to C++11 code), cmake etc.

## Installation Instructions
Reconfigure CMakeLists.txt according to your local folder structure (FANN, OpenNI2, Edvstools)!

1. `git clone ...
2. `cd dedvs; mkdir build; cd build`
3. `cmake -DCMAKE_BUILD_TYPE=Release ..`
4. `make`

# Usage


TODO

bsd license