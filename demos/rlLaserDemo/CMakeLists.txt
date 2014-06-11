project(rlLaserDemo)

add_executable(
	rlLaserDemo
	rlLaserDemo.cpp
)

target_link_libraries(
	rlLaserDemo
	rlhal
)
