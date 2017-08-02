add_library(pandar_rawdata rawdata.cc calibration.cc)
target_link_libraries(pandar_rawdata 
                      ${catkin_LIBRARIES}
                      ${YAML_CPP_LIBRARIES})
install(TARGETS pandar_rawdata
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set_target_properties(pandar_rawdata PROPERTIES
	COMPILE_FLAGS -std=c++11)
