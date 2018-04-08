PROJECT(ConvertCalibData)

SET(ccalibdata_SOURCES
  main.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME}
  ${ccalibdata_SOURCES}
)

ADD_EXECUTABLE(quickconv
  quick_conv.cpp
)
TARGET_LINK_LIBRARIES(quickconv DEdvs)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} DEdvs)
