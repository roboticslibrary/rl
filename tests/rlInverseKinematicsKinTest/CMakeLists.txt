add_executable(
	rlInverseKinematicsKinTest
	rlInverseKinematicsKinTest.cpp
)

target_compile_definitions(
	rlInverseKinematicsKinTest
	PRIVATE
	-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(
	rlInverseKinematicsKinTest
	kin
)

add_test(
	rlInverseKinematicsKinTestMitsubishiRv6sl
	${CMAKE_CURRENT_BINARY_DIR}/rlInverseKinematicsKinTest
	${rl_SOURCE_DIR}/examples/rlkin/mitsubishi-rv6sl.xml
)

add_test(
	rlInverseKinematicsKinTestStaeubliTx60l
	${CMAKE_CURRENT_BINARY_DIR}/rlInverseKinematicsKinTest
	${rl_SOURCE_DIR}/examples/rlkin/staeubli-tx60l.xml
)
