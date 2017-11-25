cmake_minimum_required(VERSION 2.8)
project( hex )

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_INCLUDE_DIRS
	/usr/local/include
	/usr/local/include/opencv2
)

set(OpenCV_LIB_DIR
	/usr/local/lib
)

set(OpenCV_LIBS
	opencv_world
	opencv_contrib_world
	pthread
)

set(SOURCES
	src/main.cpp
	src/leg.h src/leg.cpp
	src/robot.h src/robot.cpp
	src/robotcontroler.h src/robotcontroler.cpp
	src/view.h src/view.cpp
	src/util.h
	src/tcpconnector.h src/tcpconnector.cpp
	src/tcpacceptor.h src/tcpacceptor.cpp
	src/tcpstream.h src/tcpstream.cpp)


include_directories( ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
link_directories( ${OpenCV_LIB_DIR} )
add_executable( hex ${SOURCES} )
target_link_libraries( hex ${OpenCV_LIBS} )
