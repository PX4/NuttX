cmake_minimum_required(VERSION 3.2)

string(REPLACE "/" ";" BOARD_CONFIG ${CONFIG})
list(GET BOARD_CONFIG 0 NUTTX_BOARD)
list(GET BOARD_CONFIG 1 NUTTX_CONFIG)

message(STATUS "NuttX: ${NUTTX_BOARD}/${NUTTX_CONFIG}")
message(STATUS "NuttX board: ${NUTTX_BOARD}")
message(STATUS "NuttX config: ${NUTTX_CONFIG}")

# CMake build type (Debug Release RelWithDebInfo MinSizeRel Coverage)
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(NUTTX_DIR ${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# must be done early to bring all CONFIG variables into cmake
include(defconfig)

# TODO: generalize like in arch/arm/src/armv7-m/Toolchain.defs
if("${CONFIG_ARCH_CORTEXM0}" STREQUAL "y")
	set(CMAKE_SYSTEM_PROCESSOR cortex-m0)
elseif("${CONFIG_ARCH_CORTEXM3}" STREQUAL "y")
	set(CMAKE_SYSTEM_PROCESSOR cortex-m3)
elseif("${CONFIG_ARCH_CORTEXM4}" STREQUAL "y")
	set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
elseif("${CONFIG_ARCH_CORTEXM7}" STREQUAL "y")
	set(CMAKE_SYSTEM_PROCESSOR cortex-m7)
else()
	message(FATAL_ERROR "processor not set")
endif()

set(CMAKE_TOOLCHAIN_FILE Toolchain-arm-none-eabi)

project(NuttX LANGUAGES ASM C CXX)

include(ccache)
include(nuttx_add_library)
include(set_parent_srcs)

#=============================================================================

# .version
add_custom_command(
	OUTPUT ${CMAKE_BINARY_DIR}/.version
	COMMAND tools/version.sh -v 0.0 -b 0 ${CMAKE_BINARY_DIR}/.version
	DEPENDS ${NUTTX_DIR}/configs/${NUTTX_BOARD}/${NUTTX_CONFIG}/defconfig
	WORKING_DIRECTORY ${NUTTX_DIR}
	)

#=============================================================================
# mkconfig & mkversion
include(ExternalProject)
ExternalProject_Add(nuttx_host_tools
	SOURCE_DIR ${CMAKE_SOURCE_DIR}/tools
	INSTALL_DIR ${CMAKE_BINARY_DIR}
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	USES_TERMINAL_INSTALL true
	)

# mkconfig and mkversion
add_custom_command(
	OUTPUT
		${CMAKE_BINARY_DIR}/include/nuttx/config.h
		${CMAKE_BINARY_DIR}/include/nuttx/version.h
	COMMAND
		${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include/nuttx
	COMMAND
		${CMAKE_BINARY_DIR}/bin/mkconfig ${CMAKE_BINARY_DIR} > ${CMAKE_BINARY_DIR}/include/nuttx/config.h
	COMMAND
		${CMAKE_BINARY_DIR}/bin/mkversion ${CMAKE_BINARY_DIR} > ${CMAKE_BINARY_DIR}/include/nuttx/version.h
	DEPENDS
		nuttx_host_tools
		${CMAKE_BINARY_DIR}/.config
		${CMAKE_BINARY_DIR}/.version
	WORKING_DIRECTORY ${NUTTX_DIR}
	)

#=============================================================================
# symlinks (TODO: move to build directory)
add_custom_command(
	OUTPUT nuttx_symlinks.stamp
	#COMMAND ${CMAKE_COMMAND} -E remove
			#${NUTTX_DIR}/include/arch
			#${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/board
			#${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/chip
			#${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/board
			#${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/chip

	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include			${NUTTX_DIR}/include/arch			# include/arch
	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/configs/${NUTTX_BOARD}/include			${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/board	# arch/arm/include/board
	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/${CONFIG_ARCH_CHIP}	${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/chip	# arch/arm/include/chip
	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/configs/${NUTTX_BOARD}/src			${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/board	# arch/arm/src/board
	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/${CONFIG_ARCH_CHIP}	${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/chip	# arch/arm/src/chip

	COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include

	COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include			${CMAKE_BINARY_DIR}/include/arch
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_DIR}/configs/${NUTTX_BOARD}/include				${CMAKE_BINARY_DIR}/include/arch/board	# include/arch/board
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/${CONFIG_ARCH_CHIP}		${CMAKE_BINARY_DIR}/include/arch/chip	# include/arch/chip

	COMMAND ${CMAKE_COMMAND} -E touch nuttx_symlinks.stamp
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
)


add_custom_target(nuttx_context
	DEPENDS
		nuttx_symlinks.stamp
		${CMAKE_BINARY_DIR}/include/nuttx/config.h
		${CMAKE_BINARY_DIR}/include/nuttx/version.h
)

#=============================================================================
# compiler options

if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
	if(CMAKE_C_COMPILER_VERSION VERSION_GREATER 4.9)
		# force color for gcc > 4.9
		add_compile_options(-fdiagnostics-color=always)
	endif()
endif()

add_compile_options(
	-fno-builtin
	-fno-strength-reduce
	-fno-strict-aliasing

	-fomit-frame-pointer

	-Wno-cpp
	)

#=============================================================================
# create NuttX executable
#add_library(nuttx empty.c)
add_executable(nuttx empty.c)

#=============================================================================
# add all subdirectories

include_directories(include)
include_directories(${CMAKE_BINARY_DIR}/include)

include_directories(sched) # TODO: use target dependencies for linking

add_definitions("-D__KERNEL")

add_subdirectory(arch)
add_subdirectory(audio)
add_subdirectory(binfmt)
add_subdirectory(configs)
add_subdirectory(drivers)
add_subdirectory(fs)
add_subdirectory(graphics)
add_subdirectory(libs)
add_subdirectory(mm)
add_subdirectory(net)
add_subdirectory(sched)
add_subdirectory(wireless)

#=============================================================================
# NuttX Apps
if(CONFIG_APPS_DIR)
	add_subdirectory(${CONFIG_APPS_DIR} apps)
endif()

#=============================================================================
# link final binary
get_property(nuttx_app_libs GLOBAL PROPERTY NUTTX_APP_LIBS)
get_property(nuttx_libs GLOBAL PROPERTY NUTTX_LIBRARIES)

set(LD_SCRIPT "${NUTTX_DIR}/configs/${NUTTX_BOARD}/scripts/ld.script")

if(NOT EXISTS ${LD_SCRIPT})
	set(LD_SCRIPT "${NUTTX_DIR}/configs/${NUTTX_BOARD}/scripts/flash.ld")
endif()

target_link_libraries(nuttx PRIVATE

	--entry=__start

	-nostartfiles
	-nodefaultlibs

	-Wl,--script=${LD_SCRIPT}
	#-Wl,-Map=${PX4_CONFIG}.map
	#-Wl,--warn-common
	#-Wl,--gc-sections

	-Wl,--start-group
		${nuttx_libs}
		${nuttx_app_libs}
	-Wl,--end-group

	gcc
)
