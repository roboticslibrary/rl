find_package(libdc1394)

if(libdc1394_FOUND)
	add_executable(
		rlCameraDemo
		rlCameraDemo.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlCameraDemo
		hal
	)
endif()
