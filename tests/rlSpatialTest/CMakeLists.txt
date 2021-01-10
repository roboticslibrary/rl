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

foreach(test IN LISTS TESTS)
	add_executable(
		${test}
		${test}.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	target_link_libraries(${test} math)
	add_test(NAME ${test} COMMAND ${test})
endforeach()
