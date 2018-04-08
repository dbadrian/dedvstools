#!/bin/sh

#### YOU PROBABLY WANT TO ADJUST THINGS BY HAND !!!! ####
# This is meant for ubuntu, obviously, the demon child of the linux world. Urgh.
# This will create and download all dependencies in the folder you start it from. Urgh.
# This thing is still missing the PCL dependencies. Urgh.

## Install required libraries
sudo apt-get install git libboost-all-dev libeigen3-dev libqt4-dev g++ build-essential cmake cmake-qt-gui libopencv-dev libloki-dev libglew-dev

## Make EDVSTOOLS
git clone git@bitbucket.org:Danvil/edvstools.git
cd edvstools; mkdir build; cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j5
cd ../../

## Make FANN Library
git clone git://git.code.sf.net/p/fann/code fann
cd fann; mkdir build; cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j5
cd ../../

## Download and extract OpenNI2 (NOT FOR ARM PC; there compile by hand, and make sure to remove softfp flag from Platform.ARM
wget http://www.openni.org/wp-content/uploads/2013/07/OpenNI-Linux-x64-2.2.tar.zip
unzip OpenNI-Linux-x64-2.2.tar.zip
tar -xjvf OpenNI-Linux-x64-2.2.tar.bz2
mv OpenNI-Linux-x64-2.2 OpenNI2
cd OpenNI2
sudo sh install.sh
cd ..

## Clone dedvs...manually execute compile.sh, after configuring the paths to the various libraries as in src/CMakeLists.txt
git clone git@bitbucket.org:yezariael/dedvs.git
