add_executable(
	rlInverseKinematicsKinTest
	rlInverseKinematicsKinTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlInverseKinematicsKinTest
	kin
)

add_test(
	NAME rlInverseKinematicsKinTestMitsubishiRv6sl
	COMMAND rlInverseKinematicsKinTest
	${rl_SOURCE_DIR}/examples/rlkin/mitsubishi-rv6sl.xml
)

add_test(
	NAME rlInverseKinematicsKinTestStaeubliTx60l
	COMMAND rlInverseKinematicsKinTest
	${rl_SOURCE_DIR}/examples/rlkin/staeubli-tx60l.xml
)
