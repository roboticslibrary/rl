add_executable(
	rlCircularTest
	rlCircularTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlCircularTest
	math
)

add_test(
	NAME rlCircularTest
	COMMAND rlCircularTest
)
