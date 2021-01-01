find_package(Boost REQUIRED)

add_executable(
	rlNearestNeighborsTest
	iterator.h
	rlNearestNeighborsTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlNearestNeighborsTest
	math
	Boost::headers
)

add_test(
	NAME rlNearestNeighborsTest
	COMMAND rlNearestNeighborsTest
)
