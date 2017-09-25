find_package(Boost REQUIRED)

find_package(libiconv)
find_package(libxml2 REQUIRED)
find_package(libxslt REQUIRED)
find_package(zlib)

set(
	HDRS
	Attribute.h
	Document.h
	DomParser.h
	Exception.h
	Node.h
	NodeSet.h
	Namespace.h
	Object.h
	Path.h
	Schema.h
	Stylesheet.h
)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
	add_library(xml INTERFACE)
	add_custom_target(xml_sources SOURCES ${HDRS})
else()
	add_library(xml STATIC ${HDRS} dummy.cpp)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(xml INTERFACE cxx_std_11)
endif()

target_include_directories(
	xml
	INTERFACE
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
	${Boost_INCLUDE_DIR}
	${LIBXML2_INCLUDE_DIRS}
	${LIBXSLT_INCLUDE_DIRS}
)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
	target_link_libraries(xml INTERFACE ${LIBXML2_LIBRARIES} ${LIBXSLT_LIBRARIES})
else()
	target_link_libraries(xml ${LIBXML2_LIBRARIES} ${LIBXSLT_LIBRARIES})
endif()

if(libiconv_FOUND)
	target_include_directories(xml INTERFACE ${libiconv_INCLUDE_DIRS})
	if(NOT CMAKE_VERSION VERSION_LESS 3.0)
		target_link_libraries(xml INTERFACE ${libiconv_LIBRARIES})
	else()
		target_link_libraries(xml ${libiconv_LIBRARIES})
	endif()
endif()

if(zlib_FOUND)
	target_include_directories(xml INTERFACE ${zlib_INCLUDE_DIRS})
	if(NOT CMAKE_VERSION VERSION_LESS 3.0)
		target_link_libraries(xml INTERFACE ${zlib_LIBRARIES})
	else()
		target_link_libraries(xml ${zlib_LIBRARIES})
	endif()
endif()

if(CMAKE_VERSION VERSION_LESS 3.0)
	set_target_properties(
		xml
		PROPERTIES
		OUTPUT_NAME rlxml
		POSITION_INDEPENDENT_CODE ON
		VERSION ${VERSION}
	)
	
	if(WIN32)
		set_target_properties(
			xml
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/xml COMPONENT development)

install(
	TARGETS xml
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
