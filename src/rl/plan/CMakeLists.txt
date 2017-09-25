find_package(Boost REQUIRED)

set(
	HDRS
	AddRrtConCon.h
	AdvancedOptimizer.h
	BridgeSampler.h
	DistanceModel.h
	Eet.h
	Exception.h
	GaussianSampler.h
	GnatNearestNeighbors.h
	KdtreeBoundingBoxNearestNeighbors.h
	KdtreeNearestNeighbors.h
	LinearNearestNeighbors.h
	MatrixPtr.h
	Metric.h
	Model.h
	NearestNeighbors.h
	Optimizer.h
	Planner.h
	Prm.h
	PrmUtilityGuided.h
	RealList.h
	RecursiveVerifier.h
	Rrt.h
	RrtCon.h
	RrtConCon.h
	RrtDual.h
	RrtExtCon.h
	RrtExtExt.h
	RrtGoalBias.h
	Sampler.h
	SequentialVerifier.h
	SimpleModel.h
	SimpleOptimizer.h
	TransformPtr.h
	UniformSampler.h
	Vector3List.h
	Vector3Ptr.h
	VectorList.h
	VectorPtr.h
	Verifier.h
	Viewer.h
	WorkspaceMetric.h
	WorkspaceSphere.h
	WorkspaceSphereExplorer.h
	WorkspaceSphereList.h
)

set(
	SRCS
	AddRrtConCon.cpp
	AdvancedOptimizer.cpp
	BridgeSampler.cpp
	DistanceModel.cpp
	Eet.cpp
	Exception.cpp
	GaussianSampler.cpp
	GnatNearestNeighbors.cpp
	KdtreeBoundingBoxNearestNeighbors.cpp
	KdtreeNearestNeighbors.cpp
	LinearNearestNeighbors.cpp
	Metric.cpp
	Model.cpp
	NearestNeighbors.cpp
	Optimizer.cpp
	Planner.cpp
	Prm.cpp
	PrmUtilityGuided.cpp
	RecursiveVerifier.cpp
	Rrt.cpp
	RrtCon.cpp
	RrtConCon.cpp
	RrtDual.cpp
	RrtExtCon.cpp
	RrtExtExt.cpp
	RrtGoalBias.cpp
	Sampler.cpp
	SequentialVerifier.cpp
	SimpleModel.cpp
	SimpleOptimizer.cpp
	UniformSampler.cpp
	Verifier.cpp
	Viewer.cpp
	WorkspaceMetric.cpp
	WorkspaceSphere.cpp
	WorkspaceSphereExplorer.cpp
)

add_library(
	plan
	${HDRS}
	${SRCS}
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_compile_definitions(
		plan
		PUBLIC
		-DBOOST_ALL_NO_LIB
		-DBOOST_CHRONO_HEADER_ONLY
		-DBOOST_ERROR_CODE_HEADER_ONLY
		-DBOOST_SYSTEM_NO_DEPRECATED
	)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(plan PUBLIC cxx_std_11)
endif()

target_include_directories(
	plan
	PUBLIC
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	plan
	kin
	math
	mdl
	sg
	util
	xml
)

set_target_properties(
	plan
	PROPERTIES
	OUTPUT_NAME rlplan
	POSITION_INDEPENDENT_CODE ON
	VERSION ${VERSION}
)

if(WIN32)
	if(BUILD_SHARED_LIBS)
		set_target_properties(
			plan
			PROPERTIES
			DEBUG_POSTFIX d
		)
	else()
		set_target_properties(
			plan
			PROPERTIES
			DEBUG_POSTFIX sd
			MINSIZEREL_POSTFIX s
			RELEASE_POSTFIX s
			RELWITHDEBINFO_POSTFIX s
		)
	endif()
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/plan COMPONENT development)

install(
	TARGETS plan
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)

if(BUILD_SHARED_LIBS)
	install(
		TARGETS plan
		EXPORT rl
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
	)
endif()

if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
	install(FILES $<TARGET_PDB_FILE:plan> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()
