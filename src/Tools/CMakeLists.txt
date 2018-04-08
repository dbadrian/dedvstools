PROJECT(Tools)

#ADD_EXECUTABLE(raw2pcd
#  raw2pcd.cpp
#)
#TARGET_LINK_LIBRARIES(raw2pcd DEdvs boost_system boost_timer boost_filesystem boost_program_options boost_regex pcl_common pcl_io)

ADD_EXECUTABLE(profile
  profile.cpp
)
TARGET_LINK_LIBRARIES(profile DEdvs boost_system boost_timer boost_filesystem boost_program_options)
