PROJECT(xtiontest)

SET(xtiontest_SOURCES
  xtiontest.cpp
)


ADD_EXECUTABLE(${PROJECT_NAME}
  ${xtiontest_SOURCES}
)

TARGET_LINK_LIBRARIES(xtiontest DEdvs OpenNI2 opencv_core opencv_calib3d opencv_highgui
                      opencv_imgproc opencv_contrib opencv_features2d)
