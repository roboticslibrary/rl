add_executable(
	rlJacobianKinTest
	rlJacobianKinTest.cpp
)

target_link_libraries(
	rlJacobianKinTest
	kin
)

add_test(
	rlJacobianKinTestMitsubishiRv6sl
	${CMAKE_CURRENT_BINARY_DIR}/rlJacobianKinTest
	${rl_SOURCE_DIR}/examples/rlkin/mitsubishi-rv6sl.xml
)

add_test(
	rlJacobianKinTestStaeubliTx60l
	${CMAKE_CURRENT_BINARY_DIR}/rlJacobianKinTest
	${rl_SOURCE_DIR}/examples/rlkin/staeubli-tx60l.xml
)
