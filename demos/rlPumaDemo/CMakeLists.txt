find_package(Boost REQUIRED)

add_executable(
	rlPumaDemo
	rlPumaDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlPumaDemo
	kin
	Boost::headers
)
