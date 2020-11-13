ExternalProject_Add(
	boost
	URL
		https://sourceforge.net/projects/boost/files/boost/1.74.0/boost_1_74_0.7z/download
		https://dl.bintray.com/boostorg/release/1.74.0/source/boost_1_74_0.7z
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/boost_1_74_0.7z
	URL_MD5 3d513f8830c3c8aa308d782e740c4931
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	BUILD_IN_SOURCE 1
	INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>/boost ${CMAKE_INSTALL_FULL_INCLUDEDIR}/boost
)

ExternalProject_Add_Step(
	boost license
	DEPENDEES build
	COMMAND ${CMAKE_COMMAND} -E copy <SOURCE_DIR>/LICENSE_1_0.txt ${CMAKE_INSTALL_FULL_DATADIR}/boost/LICENSE_1_0.txt
)
