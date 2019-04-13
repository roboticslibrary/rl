add_executable(
	rlCircularTest
	rlCircularTest.cpp
)

target_link_libraries(
	rlCircularTest
	math
)

add_test(
	NAME rlCircularTest
	COMMAND rlCircularTest
)
