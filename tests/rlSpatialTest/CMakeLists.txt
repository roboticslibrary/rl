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

foreach(TEST ${TESTS})
	add_executable(${TEST} ${TEST}.cpp)
	target_link_libraries(${TEST} math)
	add_test(${TEST} ${CMAKE_CURRENT_BINARY_DIR}/${TEST})
endforeach()
