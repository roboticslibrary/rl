add_executable(
	rlJacobianMdlTest
	rlJacobianMdlTest.cpp
)

target_link_libraries(
	rlJacobianMdlTest
	mdl
)

add_test(
	rlJacobianMdlTestMitsubishiRv6sl
	${CMAKE_CURRENT_BINARY_DIR}/rlJacobianMdlTest
	${rl_SOURCE_DIR}/examples/rlmdl/mitsubishi-rv6sl.xml
)

add_test(
	rlJacobianMdlTestComauSmart5Nj422027
	${CMAKE_CURRENT_BINARY_DIR}/rlJacobianMdlTest
	${rl_SOURCE_DIR}/examples/rlmdl/comau-smart5-nj4-220-27.xml
)
