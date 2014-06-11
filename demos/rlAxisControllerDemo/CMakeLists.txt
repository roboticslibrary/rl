project(rlAxisControllerDemo)

add_executable(
	rlAxisControllerDemo
	rlAxisControllerDemo.cpp
)

target_link_libraries(
	rlAxisControllerDemo
	rlhal
	rlmath
)
