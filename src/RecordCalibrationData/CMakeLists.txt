PROJECT(RecordCalibrationData)

SET(RecordCalibrationData_SOURCES
  main.cpp
  record_calib_data.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME}
  ${RecordCalibrationData_SOURCES}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} DEdvs Edvs OpenNI2 boost_system
            opencv_core opencv_calib3d opencv_highgui opencv_imgproc)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_directory ${OPENNI_REDIST_DIR} ${CMAKE_CURRENT_BINARY_DIR}
)