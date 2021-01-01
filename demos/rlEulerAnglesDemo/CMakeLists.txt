find_package(Boost REQUIRED)

add_executable(
	rlEulerAnglesDemo
	rlEulerAnglesDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlEulerAnglesDemo
	math
	Boost::headers
)
