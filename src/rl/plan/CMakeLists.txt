project(rlplan)

find_package(Boost COMPONENTS thread system REQUIRED)

find_package(Cgal REQUIRED)

set(
	HDRS
	AddRrtConCon.h
	AdvancedOptimizer.h
	BridgeSampler.h
	DistanceModel.h
	Eet.h
	Exception.h
	GaussianSampler.h
	MatrixPtr.h
	Model.h
	Optimizer.h
	Orthogonal_k_neighbor_search.h
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
	Model.cpp
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
	WorkspaceSphere.cpp
	WorkspaceSphereExplorer.cpp
)

if(WIN32)
	add_library(
		rlplan
		STATIC
		${HDRS}
		${SRCS}
	)
else(WIN32)
	add_library(
		rlplan
		SHARED
		${HDRS}
		${SRCS}
	)
endif(WIN32)

target_compile_definitions(
	rlplan
	PUBLIC
	-DBOOST_ALL_NO_LIB
	-DCGAL_DISABLE_ROUNDING_MATH_CHECK
)

target_include_directories(
	rlplan
	PUBLIC
	${Boost_INCLUDE_DIR}
	${CGAL_INCLUDE_DIRS}
)

target_link_libraries(
	rlplan
	rlkin
	rlmath
	rlmdl
	rlsg
	rlutil
	rlxml
	${Boost_LIBRARIES}
	${CGAL_LIBRARIES}
)

set_target_properties(
	rlplan
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/plan COMPONENT headers)

install(
	TARGETS rlplan
	EXPORT rl
	COMPONENT libraries
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
