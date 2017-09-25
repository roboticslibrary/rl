add_executable(
	rlCircularTest
	rlCircularTest.cpp
)

target_link_libraries(
	rlCircularTest
	math
)

add_test(
	rlCircularTest1
	${CMAKE_CURRENT_BINARY_DIR}/rlCircularTest
)
