PROJECT(GDCalib)

ADD_EXECUTABLE(GDCalibLUT
  gdcalib_lut_creator.cpp
)
TARGET_LINK_LIBRARIES(GDCalibLUT DEdvs boost_system boost_timer)

