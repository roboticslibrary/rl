if(RL_USE_QT6)
	find_package(Qt6 COMPONENTS Core Gui Widgets QUIET)
	set(QT_FOUND ${Qt6_FOUND})
endif()

if(NOT Qt6_FOUND AND RL_USE_QT5)
	find_package(Qt5 COMPONENTS Core Gui Widgets QUIET)
	set(QT_FOUND ${Qt5_FOUND})
endif()

if(NOT Qt6_FOUND AND NOT Qt5_FOUND)
	set(QT_USE_IMPORTED_TARGETS ON)
	find_package(Qt4 COMPONENTS QtCore QtGui)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(QT_FOUND)
	set(
		HDRS
		AngleAxisModel.h
		Delegate.h
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
		Delegate.cpp
		EulerAnglesModel.cpp
		GroupBox.cpp
		MainWindow.cpp
		QuaternionModel.cpp
		rlRotationConverterDemo.cpp
		RotationMatrixModel.cpp
		TableView.cpp
	)
	add_executable(
		rlRotationConverterDemo
		WIN32
		${HDRS}
		${SRCS}
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlRotationConverterDemo
		math
	)
	
	if(Qt6_FOUND)
		target_link_libraries(rlRotationConverterDemo Qt6::Core Qt6::Gui Qt6::Widgets)
	elseif(Qt5_FOUND)
		target_link_libraries(rlRotationConverterDemo Qt5::Core Qt5::Gui Qt5::Widgets)
	elseif(Qt4_FOUND)
		target_link_libraries(rlRotationConverterDemo Qt4::QtCore Qt4::QtGui)
	endif()
	
	set_target_properties(
		rlRotationConverterDemo
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
	
	if(MSVC)
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
	
	if(MSVC AND BUILD_SHARED_LIBS)
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
