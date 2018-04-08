PROJECT(NNCalib)

ADD_EXECUTABLE(NNCalibTrain
  nncalib_train.cpp
)
TARGET_LINK_LIBRARIES(NNCalibTrain fann DEdvs)

#################################################

ADD_EXECUTABLE(NNCalibTest
  nncalib_test.cpp
)

TARGET_LINK_LIBRARIES(NNCalibTest fann)

#################################################

ADD_EXECUTABLE(NNCalibLookupTable
  nncalib_lookup_creator.cpp
)

TARGET_LINK_LIBRARIES(NNCalibLookupTable fann)
