include(ExternalProject)

list(INSERT CMAKE_PREFIX_PATH 0 ${CMAKE_INSTALL_PREFIX})

ExternalProject_Add(
	soqt
	DEPENDS coin cpack.d soanydata sogui
	#GIT_REPOSITORY https://github.com/coin3d/soqt
	#GIT_TAG SoQt-1.6.0
	URL https://github.com/coin3d/soqt/archive/SoQt-1.6.0.zip
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/soqt-SoQt-1.6.0.zip
	URL_MD5 856c38b464a4986f39fbc52f86a1f4a0
	PATCH_COMMAND
	${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/../cpack.d/cpack.d-prefix/src/cpack.d <SOURCE_DIR>/cpack.d &&
	${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/../soanydata/soanydata-prefix/src/soanydata <SOURCE_DIR>/data &&
	${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/../sogui/sogui-prefix/src/sogui <SOURCE_DIR>/src/Inventor/Qt/common
	CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_CFG_INTDIR=${CMAKE_CFG_INTDIR} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH} -DCMAKE_TRY_COMPILE_CONFIGURATION=${CMAKE_TRY_COMPILE_CONFIGURATION} -DSOQT_BUILD_DOCUMENTATION=OFF
	INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
)
