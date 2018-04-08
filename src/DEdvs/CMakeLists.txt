PROJECT(DEdvs)

SET(DEdvs_SOURCES
  xtion_io.cpp
  opencv_iop.cpp
  dedvs_auxiliary.cpp
  dedvs.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${DEdvs_SOURCES}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} OpenNI2 Edvs opencv_core opencv_calib3d 
                      opencv_highgui opencv_imgproc opencv_contrib)