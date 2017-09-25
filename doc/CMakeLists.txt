find_package(Doxygen)
find_package(LATEX)
find_package(Perl)

configure_file(Doxyfile.in Doxyfile)

if(BUILD_DOCUMENTATION)
	add_custom_target(doc ALL ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
elseif(DOXYGEN_FOUND)
	add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
endif()

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/rl-${VERSION} OPTIONAL COMPONENT documentation)
