project(rlInverseKinematicsTest)

find_package(Boost REQUIRED)

add_executable(
	rlInverseKinematicsTest
	rlInverseKinematicsTest.cpp
)

target_compile_definitions(
	rlInverseKinematicsTest
	PRIVATE
	-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_include_directories(
	rlInverseKinematicsTest
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlInverseKinematicsTest
	rlkin
)

add_test(
	rlInverseKinematicsTestMitsubishiRv6sl
	${CMAKE_CURRENT_BINARY_DIR}/rlInverseKinematicsTest
	${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/mitsubishi-rv6sl.xml
)

add_test(
	rlInverseKinematicsTestStaeubliTx60l
	${CMAKE_CURRENT_BINARY_DIR}/rlInverseKinematicsTest
	${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/staeubli-tx60l.xml
)
