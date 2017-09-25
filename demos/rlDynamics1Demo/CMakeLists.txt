find_package(Boost REQUIRED)

add_executable(
	rlDynamics1Demo
	rlDynamics1Demo.cpp
)

target_include_directories(
	rlDynamics1Demo
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlDynamics1Demo
	mdl
)
