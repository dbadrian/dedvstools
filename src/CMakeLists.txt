cmake_minimum_required(VERSION 2.8)

project(DEvdsTools)

# EIGEN LIBRARY
set(EIGEN_INCLUDE_DIR "/usr/include/eigen3" CACHE STRING "Eigen3 include folder")

# OPENCV2 LIBRARY
set(OPENCV2_INCLUDE_DIR "/usr/include/opencv2" CACHE STRING "OpenCV2 include folder")

# PCL LIBRARY
#set(PCL_INCLUDE_DIR "/home/david/git/pcl-trunk/build/include" CACHE STRING "PCL include folder")
#set(PCL_LIB_DIR "/home/david/git/pcl-trunk/lib" CACHE STRING "PCL lib folder")
#find_package(PCL 1.7 REQUIRED) # smaller version number will do the trick as well probably!

#EDVSTOOLS
set(EDVSTOOLS_GIT_DIR "/home/dadrian/git/edvstools/" CACHE STRING "edvstools git folder")
set(EDVSTOOLS_BIN_DIR "/home/dadrian/git/edvstools/build" CACHE STRING "edvstools build folder")

#OPENNI LIBRARY (2.1)
set(OPENNI_INCLUDE_DIR "/home/dadrian/git/OpenNI2/Include" CACHE STRING "openni include folder")
set(OPENNI_REDIST_DIR  "/home/dadrian/git/OpenNI2/Redist" CACHE STRING "openni redist folder")

#FANN LIBRARY (2.2.0)
#set(FANN_INCLUDE_DIR "/home/dadrian/git/fann/src/include/" CACHE STRING "fann include folder")
#set(FANN_BIN_DIR  "/home/dadrian/git/fann/build/src/" CACHE STRING "fann bin folder")

add_definitions(-std=c++0x)
#add_definitions(${PCL_DEFINITIONS})
#add_definitions(-march=armv7-a)

include_directories(
  ${EIGEN_INCLUDE_DIR}
  ${OPENCV2_INCLUDE_DIR}
  ${EDVSTOOLS_GIT_DIR}
  ${OPENNI_INCLUDE_DIR}
#  ${FANN_INCLUDE_DIR}
#  ${PCL_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)
link_directories(
  ${EDVSTOOLS_BIN_DIR}/Edvs
  ${OPENNI_REDIST_DIR}
#  ${FANN_BIN_DIR}
  ${OPENNI_REDIST_DIR}/OpenNI2/Drivers
#  ${PCL_LIB_DIR}
)

add_subdirectory(DEdvs)
add_subdirectory(DEdvsRecorder)
add_subdirectory(GDCalib)
#add_subdirectory(NNCalib)
#add_subdirectory(ConvertCalibData)
#add_subdirectory(RecordCalibrationData)
#add_subdirectory(Examples)
add_subdirectory(Tools)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/src ${PROJECT_BINARY_DIR}
)
