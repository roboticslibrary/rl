project(rlInversePositionDemo)

find_package(Boost REQUIRED)

add_executable(
	rlInversePositionDemo
	rlInversePositionDemo.cpp
)

target_include_directories(
	rlInversePositionDemo
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlInversePositionDemo
	rlhal
	rlmdl
	rlutil
)
