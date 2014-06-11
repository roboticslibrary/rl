project(rlKalmanDemo)

add_executable(
	rlKalmanDemo
	rlKalmanDemo.cpp
)

target_link_libraries(
	rlKalmanDemo
	rlmath
)
