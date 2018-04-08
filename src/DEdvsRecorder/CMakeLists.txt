PROJECT(DEdvsRecorder)

SET(DEdvsViewer_SOURCES
  main.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME}
  ${DEdvsViewer_SOURCES}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} DEdvs Edvs OpenNI2 opencv_core opencv_highgui
                      opencv_imgproc opencv_contrib boost_system boost_filesystem boost_program_options)


ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_directory ${OPENNI_REDIST_DIR} ${CMAKE_CURRENT_BINARY_DIR}
)