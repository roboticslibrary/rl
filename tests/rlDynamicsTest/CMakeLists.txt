project(rlDynamicsTest)

find_package(Boost REQUIRED)

add_executable(
	rlDynamicsTest
	rlDynamicsTest.cpp
)

target_include_directories(
	rlDynamicsTest
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlDynamicsTest
	rlmdl
	rlutil
)

add_test(
	rlDynamicsTestMitsubishiRv6sl
	${CMAKE_CURRENT_BINARY_DIR}/rlDynamicsTest
	${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlmdl/mitsubishi-rv6sl.xml
	100
)

add_test(
	rlDynamicsTestPlanar2
	${CMAKE_CURRENT_BINARY_DIR}/rlDynamicsTest
	${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlmdl/planar2.xml
	100
)
