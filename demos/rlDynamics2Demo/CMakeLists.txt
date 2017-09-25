find_package(Boost REQUIRED)

add_executable(
	rlDynamics2Demo
	rlDynamics2Demo.cpp
)

target_include_directories(
	rlDynamics2Demo
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlDynamics2Demo
	mdl
)
