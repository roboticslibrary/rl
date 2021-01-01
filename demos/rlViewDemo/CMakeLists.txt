find_package(SoQt)

if(RL_USE_QT5 AND SoQt_HAVE_QT5)
	find_package(Qt5 COMPONENTS Core Gui OpenGL Widgets QUIET)
endif()

if(Qt5_FOUND)
	set(QT_FOUND Qt5_FOUND)
else()
	set(QT_USE_IMPORTED_TARGETS ON)
	find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(QT_FOUND AND SoQt_FOUND)
	add_executable(
		rlViewDemo
		WIN32
		rlViewDemo.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlViewDemo
		sg
		SoQt::SoQt
	)
	
	if(Qt5_FOUND)
		target_link_libraries(rlViewDemo Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets)
	elseif(Qt4_FOUND)
		target_link_libraries(rlViewDemo Qt4::QtCore Qt4::QtGui Qt4::QtOpenGL)
	endif()
	
	set_target_properties(
		rlViewDemo
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
	
	if(MSVC)
		set_target_properties(
			rlViewDemo
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
	
	install(
		TARGETS rlViewDemo
		COMPONENT demos
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(MSVC AND BUILD_SHARED_LIBS)
		install(FILES $<TARGET_PDB_FILE:rlViewDemo> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT demos)
	endif()
	
	if(UNIX)
		configure_file(rlViewDemo.desktop.in rlViewDemo.desktop @ONLY)
		install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rlViewDemo.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
	endif()
	
	set(
		CPACK_NSIS_CREATE_ICONS_EXTRA
		${CPACK_NSIS_CREATE_ICONS_EXTRA}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlViewDemo.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlViewDemo.exe\\\" \\\\
			\\\"\\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl-${PROJECT_VERSION}\\\\examples\\\\rlsg\\\\unimation-puma560_boxes.xml\\\$\\\\\\\"\\\\
			\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Visualization demo with Unimation Puma 560\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlViewDemo.lnk\\\"" PARENT_SCOPE)
	
	set(
		WIX_SHORTCUTS
		${WIX_SHORTCUTS}
		"<Shortcut
			Arguments=\"&quot;[INSTALL_ROOT]${CMAKE_INSTALL_DATADIR}\\rl-${PROJECT_VERSION}\\examples\\rlsg\\unimation-puma560_boxes.xml&quot;\"
			Description=\"Visualization demo with Unimation Puma 560\"
			Id=\"CM_SP_bin.rlViewDemo.exe\"
			Name=\"rlViewDemo\"
			Target=\"[#CM_FP_bin.rlViewDemo.exe]\"
			WorkingDirectory=\"CM_DP_bin\"
		/>"
		PARENT_SCOPE
	)
endif()
