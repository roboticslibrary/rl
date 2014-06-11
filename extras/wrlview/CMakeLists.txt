project(wrlview)

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
set(QT_USE_QTMAIN TRUE)
include(${QT_USE_FILE})

find_package(Coin)
find_package(OpenGL)
find_package(SoQt)

if(COIN_FOUND AND OPENGL_FOUND AND QT_FOUND AND SOQT_FOUND)
	qt4_wrap_cpp(
		MOC_SRCS
		MainWindow.h
	)
	
	add_executable(
		wrlview
		WIN32
		${MOC_SRCS}
		MainWindow.h
		MainWindow.cpp
		wrlview.cpp
		wrlview.rc
	)
	
	target_compile_definitions(
		wrlview
		PRIVATE
		${COIN_DEFINITIONS}
		${QT_DEFINITIONS}
		${SOQT_DEFINITIONS}
	)
	
	target_include_directories(
		wrlview
		PRIVATE
		${COIN_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
		${QT_INCLUDES}
		${SOQT_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		wrlview
		${COIN_LIBRARIES}
		${OPENGL_LIBRARIES}
		${QT_LIBRARIES}
		${SOQT_LIBRARIES}
	)
	
	set_target_properties(
		wrlview
		PROPERTIES
		VERSION 0.1.9
	)
	
	install(
		TARGETS
		wrlview
		COMPONENT extras
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(UNIX)
		install(FILES wrlview.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT extras)
		install(FILES wrlview.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps COMPONENT extras)
	endif(UNIX)
	
	set(
		CPACK_NSIS_EXTRA_INSTALL_COMMANDS
		${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\wrlview.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\wrlview.exe\\\" \\\\
			\\\"\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Viewer for VRML and Open Inventor files\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\wrlview.lnk\\\"" PARENT_SCOPE)
endif(COIN_FOUND AND OPENGL_FOUND AND QT_FOUND AND SOQT_FOUND)
