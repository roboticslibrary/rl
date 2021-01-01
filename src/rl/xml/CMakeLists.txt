find_package(Boost REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(LibXslt REQUIRED)

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

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	add_library(xml INTERFACE ${HDRS})
else()
	add_library(xml INTERFACE)
	add_custom_target(xml_sources SOURCES ${HDRS})
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(xml INTERFACE cxx_std_11)
endif()

target_include_directories(
	xml
	INTERFACE
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}>
)

target_link_libraries(xml INTERFACE Boost::headers LibXml2::LibXml2 LibXslt::LibXslt)

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	set_target_properties(
		xml
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/xml COMPONENT development)

install(
	TARGETS xml
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
