cmake_minimum_required(VERSION 3.2)

message(STATUS "NuttX Host Tools")

project(NuttX-tools LANGUAGES C)

#=============================================================================
# ccache
#
option(CCACHE "Use ccache if available" ON)
find_program(CCACHE_PROGRAM ccache)
if (CCACHE AND CCACHE_PROGRAM AND NOT DEFINED ENV{CCACHE_DISABLE})

	get_filename_component(ccache_real_path ${CCACHE_PROGRAM} REALPATH)
	get_filename_component(c_real_path ${CMAKE_C_COMPILER} REALPATH)
	get_filename_component(c_abs_path ${CMAKE_C_COMPILER} ABSOLUTE)

	if ("${ccache_real_path}" STREQUAL "${c_real_path}")
		#message(STATUS "ccache enabled via symlink (${c_abs_path} -> ${c_real_path})")
	else()
		#message(STATUS "ccache enabled (export CCACHE_DISABLE=1 to disable)")
		set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
	endif()

endif()

#=============================================================================

add_executable(mkconfig mkconfig.c cfgdefine.c)
install(TARGETS mkconfig DESTINATION bin)

add_executable(mkversion mkversion.c cfgdefine.c)
install(TARGETS mkversion DESTINATION bin)
