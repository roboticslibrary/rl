find_package(Boost REQUIRED)

add_executable(
	rlInversePositionDemo
	rlInversePositionDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_include_directories(
	rlInversePositionDemo
	PUBLIC
	${Boost_INCLUDE_DIRS}
)

target_link_libraries(
	rlInversePositionDemo
	hal
	mdl
	util
)
