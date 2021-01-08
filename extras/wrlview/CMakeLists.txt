project(wrlview VERSION 0.1.14)

if(NOT PROJECT_VERSION_TWEAK)
	set(PROJECT_VERSION_TWEAK 0)
endif()

include(CMakePushCheckState)

find_package(SoQt)

if(RL_USE_QT5 AND SoQt5_FOUND)
	find_package(Qt5 COMPONENTS Core Gui Network OpenGL Widgets QUIET)
endif()

if(Qt5_FOUND)
	set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES} ${Qt5Network_LIBRARIES} ${Qt5Widgets_LIBRARIES})
	set(QT_FOUND Qt5_FOUND)
else()
	set(QT_USE_IMPORTED_TARGETS ON)
	find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork QtOpenGL)
	set(QT_USE_QTMAIN ON)
	include(${QT_USE_FILE})
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Coin)
find_package(OpenGL)

if(Coin_FOUND AND OPENGL_FOUND AND QT_FOUND AND SoQt_FOUND)
	configure_file(wrlview.rc.in wrlview.rc)
	
	add_executable(
		wrlview
		WIN32
		MainWindow.cpp
		MainWindow.h
		SoGradientBackground.cpp
		SoGradientBackground.h
		wrlview.cpp
		${CMAKE_CURRENT_BINARY_DIR}/wrlview.rc
	)
	
	target_compile_definitions(
		wrlview
		PRIVATE
		${Coin_DEFINITIONS}
		${SoQt_DEFINITIONS}
	)
	
	cmake_push_check_state(RESET)
	set(CMAKE_REQUIRED_DEFINITIONS ${Coin_DEFINITIONS})
	set(CMAKE_REQUIRED_INCLUDES ${Coin_INCLUDE_DIRS})
	set(CMAKE_REQUIRED_LIBRARIES ${Coin_LIBRARIES})
	check_include_file_cxx(ForeignFiles/SoSTLFileKit.h HAVE_SOSTLFILEKIT_H)
	if(HAVE_SOSTLFILEKIT_H)
		target_compile_definitions(wrlview PRIVATE HAVE_SOSTLFILEKIT_H)
		check_cxx_source_compiles("
			#include <ForeignFiles/SoSTLFileKit.h>
			int main() { SoSTLFileKit* stlFileKit = new SoSTLFileKit(); stlFileKit->convert(); return 0; }
		" HAVE_SOSTLFILEKIT_CONVERT)
		if(HAVE_SOSTLFILEKIT_CONVERT)
			target_compile_definitions(wrlview PRIVATE HAVE_SOSTLFILEKIT_CONVERT)
		endif()
	endif()
	cmake_pop_check_state()
	
	target_include_directories(
		wrlview
		PRIVATE
		${Coin_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
		${SoQt_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		wrlview
		${Coin_LIBRARIES}
		${OPENGL_LIBRARIES}
		${QT_LIBRARIES}
		${SoQt_LIBRARIES}
	)
	
	set_target_properties(
		wrlview
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
	
	if(MSVC)
		set_target_properties(
			wrlview
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
	
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
	endif()
	
	set(
		CPACK_NSIS_CREATE_ICONS_EXTRA
		${CPACK_NSIS_CREATE_ICONS_EXTRA}
		"SetOutPath \\\"%HOMEDRIVE%%HOMEPATH%\\\"
		CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\wrlview.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\wrlview.exe\\\" \\\\
			\\\"\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Viewer for VRML and Open Inventor files\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\wrlview.lnk\\\"" PARENT_SCOPE)
	
	set(
		WIX_SHORTCUTS
		${WIX_SHORTCUTS}
		"<Shortcut
			Description=\"Viewer for VRML and Open Inventor files\"
			Id=\"CM_SP_bin.wrlview.exe\"
			Name=\"wrlview\"
			Target=\"[#CM_FP_bin.wrlview.exe]\"
			WorkingDirectory=\"CM_DP_bin\"
		/>"
		PARENT_SCOPE
	)
endif()
