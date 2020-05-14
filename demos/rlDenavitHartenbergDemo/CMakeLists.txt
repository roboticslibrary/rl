add_executable(
	rlDenavitHartenbergDemo
	rlDenavitHartenbergDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlDenavitHartenbergDemo
	math
)
