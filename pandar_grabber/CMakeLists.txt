cmake_minimum_required(VERSION 2.8.0)
project(pandar_grabber)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(PCAP REQUIRED)

if(NOT PCL_FOUND)
	find_package(PCL REQUIRED)
endif()

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
if(CMAKE_CURRENT_LIST_DIR STREQUAL CMAKE_SOURCE_DIR)
	add_executable (test_grabber test/grabber_test.cpp)
	target_link_libraries (test_grabber
		pandar_grabber
	)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_library(${PROJECT_NAME}
	src/pandar_grabber.cpp
	)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
	$<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
	${PCL_INCLUDE_DIRS}
	${PCAP_INCLUDE_DIR}
	)
target_link_libraries(${PROJECT_NAME}
	${PCL_LIBRARIES}
	${PCAP_LIBRARY}
	)
