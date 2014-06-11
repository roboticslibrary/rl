project(rlxml)

find_package(Boost REQUIRED)

find_package(Iconv)
find_package(LibXml2 REQUIRED)
find_package(ZLIB)

set(
	HDRS
	Attribute.h
	Document.h
	DomParser.h
	Exception.h
	Node.h
	Namespace.h
	Object.h
	Path.h
	Schema.h
)

add_library(
	rlxml
	${HDRS}
	dummy.cpp
)

target_include_directories(
	rlxml
	PUBLIC
	${Boost_INCLUDE_DIR}
	${LIBXML2_INCLUDE_DIRS}
)

target_link_libraries(
	rlxml
	${LIBXML2_LIBRARIES}
)

if(ICONV_FOUND)
	target_include_directories(rlxml PUBLIC ${ICONV_INCLUDE_DIRS})
	target_link_libraries(rlxml ${ICONV_LIBRARIES})
endif(ICONV_FOUND)

if(ZLIB_FOUND)
	target_include_directories(rlxml PUBLIC ${ZLIB_INCLUDE_DIRS})
	target_link_libraries(rlxml ${ZLIB_LIBRARIES})
endif(ZLIB_FOUND)

set_target_properties(
	rlxml
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/xml COMPONENT headers)

install(
	TARGETS rlxml
	EXPORT rl
	COMPONENT libraries
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
