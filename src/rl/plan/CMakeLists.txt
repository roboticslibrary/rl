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
	${rl_BINARY_DIR}/robotics-library.rc
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(plan PUBLIC cxx_std_11)
endif()

target_include_directories(
	plan
	PUBLIC
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}>
)

target_link_libraries(
	plan
	kin
	math
	mdl
	sg
	util
	xml
	Boost::headers
)

set_target_properties(
	plan
	PROPERTIES
	C_VISIBILITY_PRESET hidden
	CXX_VISIBILITY_PRESET hidden
	OUTPUT_NAME rlplan
	POSITION_INDEPENDENT_CODE ON
	VERSION ${PROJECT_VERSION}
	VISIBILITY_INLINES_HIDDEN ON
)

if(MSVC)
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

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/plan COMPONENT development)

if(NOT CMAKE_VERSION VERSION_LESS 3.12)
	install(
		TARGETS plan
		EXPORT rl
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_COMPONENT development
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
else()
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
endif()

if(MSVC AND BUILD_SHARED_LIBS)
	install(FILES $<TARGET_PDB_FILE:plan> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()

generate_export_header(
	plan
	EXPORT_FILE_NAME export.h
	PREFIX_NAME RL_
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/plan COMPONENT development)
