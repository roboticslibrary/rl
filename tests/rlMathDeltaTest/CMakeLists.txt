add_executable(
	rlMathDeltaTest
	rlMathDeltaTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlMathDeltaTest
	math
)

add_test(
	NAME rlMathDeltaTest
	COMMAND rlMathDeltaTest
)
