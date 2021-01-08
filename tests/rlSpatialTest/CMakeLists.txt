set(
	TESTS
	rlEuclideanCrossProductTest
	rlSpatialArticulatedBodyInertiaTest
	rlSpatialCrossProductTest
	rlSpatialDotProductTest
	rlSpatialForceVectorTest
	rlSpatialMotionVectorTest
	rlSpatialPlueckerTransformTest
	rlSpatialRigidBodyInertiaTest
)

foreach(TEST IN LISTS TESTS)
	add_executable(
		${TEST}
		${TEST}.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	target_link_libraries(${TEST} math)
	add_test(NAME ${TEST} COMMAND ${TEST})
endforeach()
