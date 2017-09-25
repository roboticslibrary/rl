add_executable(
	rlMathDeltaTest
	rlMathDeltaTest.cpp
)

target_link_libraries(
	rlMathDeltaTest
	math
)

add_test(
	rlMathDeltaTest1
	${CMAKE_CURRENT_BINARY_DIR}/rlMathDeltaTest
)
