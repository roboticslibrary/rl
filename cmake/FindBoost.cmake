set(_CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
list(REMOVE_ITEM CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

find_package(Boost)

if(Boost_FOUND AND NOT TARGET Boost::headers)
	add_library(Boost::headers INTERFACE IMPORTED)
	set_target_properties(Boost::headers PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()
if(TARGET Boost::headers AND NOT TARGET Boost::boost)
	add_library(Boost::boost INTERFACE IMPORTED)
	set_target_properties(Boost::boost PROPERTIES INTERFACE_LINK_LIBRARIES "Boost::headers")
endif()

set(CMAKE_MODULE_PATH ${_CMAKE_MODULE_PATH})
unset(_CMAKE_MODULE_PATH)
