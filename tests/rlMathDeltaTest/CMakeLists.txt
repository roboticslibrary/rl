add_executable(
	rlMathDeltaTest
	rlMathDeltaTest.cpp
)

target_link_libraries(
	rlMathDeltaTest
	math
)

add_test(
	NAME rlMathDeltaTest
	COMMAND rlMathDeltaTest
)
