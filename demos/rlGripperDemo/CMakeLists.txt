project(rlGripperDemo)

add_executable(
	rlGripperDemo
	rlGripperDemo.cpp
)

target_link_libraries(
	rlGripperDemo
	rlhal
	rlutil
)
