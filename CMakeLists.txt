cmake_minimum_required(VERSION 2.8.3)
project(people_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_geometry
  roscpp
  sensor_msgs
  std_msgs
  tf	
  dynamic_reconfigure
  ecl_linear_algebra
)

find_package(Boost REQUIRED COMPONENTS 
  system
)

find_package(orocos_kdl REQUIRED) 

# Find OpenNI
#find_package(PkgConfig)
#pkg_check_modules(OpenNI REQUIRED libopenni)

generate_dynamic_reconfigure_options(
  cfg/vision_people_detector_hog.cfg
)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES
	PointWithCovariance.msg
	PointWithCovarianceStamped.msg
)

generate_messages(
	DEPENDENCIES 
	std_msgs 
	geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #/usr/local/include/opencv2
  ${Boost_INCLUDE_DIRS}
  ${Openni_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  /usr/include/ni
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(
	people_detector_hog_node
	src/common/vision_node.cpp
	src/common/vision_object_detector.cpp 
	src/vision_people_detector_hog/vision_people_detector_hog.cpp 
	src/vision_people_detector_hog/vision_people_detector_hog_node.cpp 
	
)



add_dependencies(
	people_detector_hog_node
	${PROJECT_NAME}_gencfg
)

target_link_libraries(
  people_detector_hog_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #/usr/local/lib/libopencv_core.so
  #/usr/local/lib/libopencv_imgproc.so
  #/usr/local/lib/libopencv_objdetect.so
)


