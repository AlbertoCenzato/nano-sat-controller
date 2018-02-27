cmake_minimum_required(VERSION 2.8)
project(NanoSatController)

add_definitions(-std=c++14)


## find opencv and if not found download, compile and install it
if (UNIX) 
	find_package(OpenCV COMPONENTS video videoio imgproc)
	if (NOT OpenCV_FOUND)
		message(STATUS "OpenCV not found. Downloading and installing")
		set (OpenCV_INSTALL_PREFIX /usr/local/opencv-3.3)
		execute_process(COMMAND bash ./scripts/opencv-installer.sh ${OpenCV_INSTALL_PREFIX} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
		set (OpenCV_DIR ${OpenCV_INSTALL_PREFIX})
		find_package(OpenCV REQUIRED COMPONENTS video videoio imgproc)
	endif (NOT OpenCV_FOUND)
else (WIN32) # used to find includes when modifying the project on windows
	find_package(opencv REQUIRED COMPONENTS video videoio imgproc)
endif (UNIX)

set(OpenCV_LIBS opencv_core opencv_video opencv_videoio opencv_highgui opencv_imgproc opencv_imgcodecs)

## add configuration file
set (NANOSAT_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
set (NANOSAT_VERSION_MAJOR 1)
set (NANOSAT_VERSION_MINOR 2)
set (NANOSAT_VERSION_PATCH 0)
configure_file (config.h.in ${CMAKE_SOURCE_DIR}/include/config.h )

###########
## Build ##
###########

# disable gcc-6 warning about compatibility with gcc-7 compiled code
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-psabi")

include_directories(include 3rdparty)

# add 3rd party code as library
file (GLOB_RECURSE GNUBLIN_API_SRC 3rdparty/gnublin-api/*.cpp)
file (GLOB_RECURSE JSONCPP_SRC 3rdparty/jsoncpp/*.cpp)
add_library (3rdparty ${GNUBLIN_API_SRC} ${JSONCPP_SRC})

file (GLOB_RECURSE UTILS_SRC	 src/utils/*.cpp)
file (GLOB_RECURSE UI_SRC		 src/ui/*.cpp)		
file (GLOB_RECURSE SELF_TEST_SRC src/self_test/*.cpp)
file (GLOB_RECURSE TEST_SRC		 src/test/*.cpp)	
file (GLOB_RECURSE DEVICES_SRC	 src/devices/*.cpp)
file (GLOB_RECURSE CTRL_SRC		 src/ctrl/*.cpp)
file (GLOB		   GLOB_SRC		 src/*.cpp)

set (EXECUTABLE ${PROJECT_NAME}.exe)
add_executable(${EXECUTABLE} ${GLOB_SRC} ${CTRL_SRC} ${DEVICES_SRC} ${TEST_SRC} ${SELF_TEST_SRC} ${UI_SRC} ${UTILS_SRC})

add_dependencies(${EXECUTABLE} 3rdparty)
target_link_libraries(${EXECUTABLE} 3rdparty ${OpenCV_LIBS} pthread stdc++fs)

## Mark executables and/or libraries for installation
#install(TARGETS ${EXECUTABLE} RUNTIME DESTINATION .)

## Mark cpp header files for installation
#install(DIRECTORY resources DESTINATION resources)