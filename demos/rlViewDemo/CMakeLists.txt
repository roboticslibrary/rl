project(rlViewDemo)

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
include(${QT_USE_FILE})

find_package(SoQt)

if(QT_FOUND AND SOQT_FOUND)
	add_executable(
		rlViewDemo
		rlViewDemo.cpp
		${rl_SOURCE_DIR}/robotics-library.rc
	)
	
	target_compile_definitions(
		rlViewDemo
		PUBLIC
		${QT_DEFINITIONS}
		${SOQT_DEFINITIONS}
	)
	
	include_directories(
		rlViewDemo
		PUBLIC
		${QT_INCLUDES}
		${SOQT_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		rlViewDemo
		rlsg
		${QT_LIBRARIES}
		${SOQT_LIBRARIES}
	)
	
	set_target_properties(
		rlViewDemo
		PROPERTIES
		VERSION ${VERSION}
		DEBUG_POSTFIX d
	)
	
	install(
		TARGETS rlViewDemo
		COMPONENT demos
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(UNIX)
		install(FILES rlViewDemo.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
	endif(UNIX)
	
	set(
		CPACK_NSIS_EXTRA_INSTALL_COMMANDS
		${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlViewDemo.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlViewDemo.exe\\\" \\\\
			\\\"\\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl\\\\examples\\\\rlsg\\\\unimation-puma560_boxes.xml\\\$\\\\\\\"\\\\
			\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Visualization demo with Unimation Puma 560\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlViewDemo.lnk\\\"" PARENT_SCOPE)
endif(QT_FOUND AND SOQT_FOUND)
