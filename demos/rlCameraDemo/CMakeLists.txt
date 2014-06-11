project(rlCameraDemo)

find_package(Libdc1394)

if(LIBDC1394_FOUND)
	add_executable(
		rlCameraDemo
		rlCameraDemo.cpp
	)
	
	target_link_libraries(
		rlCameraDemo
		rlhal
	)
endif(LIBDC1394_FOUND)
