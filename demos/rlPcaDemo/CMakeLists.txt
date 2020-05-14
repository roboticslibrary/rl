add_executable(
	rlPcaDemo
	rlPcaDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlPcaDemo
	math
)
