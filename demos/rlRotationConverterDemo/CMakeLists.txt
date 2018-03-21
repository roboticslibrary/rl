if(USE_QT5)
	find_package(Qt5 COMPONENTS Core Gui Widgets QUIET)
endif()

if(Qt5_FOUND)
	set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES})
	set(QT_FOUND Qt5_FOUND)
else()
	set(QT_USE_IMPORTED_TARGETS ON)
	find_package(Qt4 COMPONENTS QtCore QtGui)
	set(QT_USE_QTMAIN ON)
	include(${QT_USE_FILE})
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(QT_FOUND)
	set(
		HDRS
		AngleAxisModel.h
		EulerAnglesModel.h
		GroupBox.h
		MainWindow.h
		QuaternionModel.h
		RotationMatrixModel.h
		TableView.h
	)
	
	set(
		SRCS
		AngleAxisModel.cpp
		EulerAnglesModel.cpp
		GroupBox.cpp
		MainWindow.cpp
		QuaternionModel.cpp
		rlRotationConverterDemo.cpp
		RotationMatrixModel.cpp
		TableView.cpp
		${rl_SOURCE_DIR}/robotics-library.rc
	)
	add_executable(
		rlRotationConverterDemo
		WIN32
		${HDRS}
		${SRCS}
	)
	
	target_link_libraries(
		rlRotationConverterDemo
		math
		${QT_LIBRARIES}
	)
	
	set_target_properties(
		rlRotationConverterDemo
		PROPERTIES
		VERSION ${VERSION}
	)
	
	if(WIN32)
		set_target_properties(
			rlRotationConverterDemo
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
	
	install(
		TARGETS rlRotationConverterDemo
		COMPONENT demos
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
		install(FILES $<TARGET_PDB_FILE:rlRotationConverterDemo> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT demos)
	endif()
	
	if(UNIX)
		configure_file(rlRotationConverterDemo.desktop.in rlRotationConverterDemo.desktop @ONLY)
		install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rlRotationConverterDemo.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
	endif()
	
	set(
		CPACK_NSIS_CREATE_ICONS_EXTRA
		${CPACK_NSIS_CREATE_ICONS_EXTRA}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlRotationConverterDemo.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlRotationConverterDemo.exe\\\" \\\\
			\\\"\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Rotation conversion demo\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlRotationConverterDemo.lnk\\\"" PARENT_SCOPE)
	
	set(
		WIX_SHORTCUTS
		${WIX_SHORTCUTS}
		"<Shortcut
			Description=\"Rotation conversion demo\"
			Id=\"CM_SP_bin.rlRotationConverterDemo.exe\"
			Name=\"rlRotationConverterDemo\"
			Target=\"[#CM_FP_bin.rlRotationConverterDemo.exe]\"
			WorkingDirectory=\"CM_DP_bin\"
		/>"
		PARENT_SCOPE
	)
endif()
