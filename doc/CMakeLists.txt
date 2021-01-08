find_package(Doxygen)

cmake_dependent_option(RL_BUILD_DOCUMENTATION "Build documentation" OFF "DOXYGEN_FOUND" OFF)

if(DOXYGEN_FOUND)
	find_package(LATEX)
	
	configure_file(Doxyfile.in Doxyfile)
	
	if(RL_BUILD_DOCUMENTATION)
		add_custom_target(doc ALL ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	else()
		add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	endif()
	
	install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/rl-${PROJECT_VERSION} OPTIONAL COMPONENT documentation)
endif()
