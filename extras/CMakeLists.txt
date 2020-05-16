cmake_minimum_required(VERSION 2.8.11)

if(POLICY CMP0072)
	cmake_policy(SET CMP0072 NEW)
endif()

project(extras)

add_subdirectory(byu2wrl)
add_subdirectory(csv2wrl)
add_subdirectory(tris2wrl)
add_subdirectory(wrlview)

set(CPACK_NSIS_CREATE_ICONS_EXTRA ${CPACK_NSIS_CREATE_ICONS_EXTRA} PARENT_SCOPE)
set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} PARENT_SCOPE)
set(WIX_SHORTCUTS ${WIX_SHORTCUTS} PARENT_SCOPE)
