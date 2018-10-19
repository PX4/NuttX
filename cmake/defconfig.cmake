
set(NUTTX_DEFCONFIG ${NUTTX_DIR}/configs/${NUTTX_BOARD}/${NUTTX_CONFIG}/defconfig CACHE FILEPATH "path to defconfig" FORCE)

if(DEFINED ENV{APPSDIR})
	# APPS directory set as environment variable
	set(NUTTX_APPS_DIR $ENV{APPSDIR} CACHE PATH "specified path to apps directory" FORCE)
	message(STATUS "Apps directory set by NUTTX_APPS_DIR environment variable: $ENV{APPSDIR}")
else()

	# parse apps directory from defconfig
	file(STRINGS ${NUTTX_DEFCONFIG} ConfigContents)
	foreach(NameAndValue ${ConfigContents})
		if("${NameAndValue}" MATCHES "CONFIG_APPS_DIR")
			# Strip leading spaces
			string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})
		
			# Find variable name
			string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})
		
			if (Name)
				# Find the value
				string(REPLACE "${Name}=" "" Value ${NameAndValue})
		
				# remove extra quotes
				string(REPLACE "\"" "" Value ${Value})

				message(STATUS "CONFIG_APPS_DIR set by defconfig: ${Value}")
				set(NUTTX_APPS_DIR ${Value} CACHE PATH "specified path to apps directory" FORCE)
			endif()
		endif()
	endforeach()
	

	# if apps directory still not set, then try default path
	get_filename_component(default_apps_dir "${CMAKE_CURRENT_SOURCE_DIR}/../apps" ABSOLUTE)
	if(EXISTS "${default_apps_dir}")
		set(NUTTX_APPS_DIR ${default_apps_dir} CACHE PATH "specified path to apps directory" FORCE)
	endif()
endif()


if(NOT EXISTS ${NUTTX_APPS_DIR})
	# try converting to absolute path
	get_filename_component(NUTTX_APPS_DIR
		"${NUTTX_DIR}/${NUTTX_APPS_DIR}"
		ABSOLUTE
	)
endif()

message(STATUS "NuttX apps: ${NUTTX_APPS_DIR}")

execute_process(
	COMMAND ${CMAKE_COMMAND} -E create_symlink dummy ${NUTTX_APPS_DIR}/platform/board
	WORKING_DIRECTORY ${NUTTX_APPS_DIR}
)
#execute_process(
#	COMMAND make --no-print-directory --silent TOPDIR=${NUTTX_DIR} APPDIR=${NUTTX_APPS_DIR} Kconfig
#	OUTPUT_FILE ${CMAKE_BINARY_DIR}/kconfig_apps.log
#	ERROR_FILE ${CMAKE_BINARY_DIR}/kconfig_apps.log
#	WORKING_DIRECTORY ${NUTTX_APPS_DIR}
#)

execute_process(
	COMMAND ${CMAKE_COMMAND} -E copy_if_different dummy_kconfig Kconfig
	WORKING_DIRECTORY ${NUTTX_DIR}/configs/dummy
)


execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config)
set(ENV{APPSDIR} ${NUTTX_APPS_DIR})
execute_process(
	COMMAND kconfig-conf --olddefconfig Kconfig
	WORKING_DIRECTORY ${NUTTX_DIR}
	OUTPUT_FILE ${CMAKE_BINARY_DIR}/kconfig.log
	ERROR_FILE ${CMAKE_BINARY_DIR}/kconfig.log
)
execute_process(
	COMMAND kconfig-tweak --set-str CONFIG_APPS_DIR "${NUTTX_APPS_DIR}"
	WORKING_DIRECTORY ${NUTTX_DIR}
)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/.config ${CMAKE_BINARY_DIR}/.config)

# copy original defconfig, inflate, then save reference copy in binary dir
add_custom_command(
	OUTPUT
		${CMAKE_BINARY_DIR}/.config
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
	COMMAND
		APPSDIR=${NUTTX_APPS_DIR} kconfig-conf --olddefconfig Kconfig
	COMMAND
		kconfig-tweak --set-str CONFIG_APPS_DIR "${NUTTX_APPS_DIR}"
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/.config ${CMAKE_BINARY_DIR}/.config
	DEPENDS ${NUTTX_DEFCONFIG}
	WORKING_DIRECTORY ${NUTTX_DIR}
)

###############################################################################
# parse nuttx config options for cmake
file(STRINGS ${NUTTX_DIR}/.config ConfigContents)
foreach(NameAndValue ${ConfigContents})
	# Strip leading spaces
	string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

	# Find variable name
	string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

	if (Name)
		# Find the value
		string(REPLACE "${Name}=" "" Value ${NameAndValue})

		# remove extra quotes
		string(REPLACE "\"" "" Value ${Value})

		# Set the variable
		#message(STATUS "${Name} ${Value}")
		set(${Name} ${Value} CACHE INTERNAL "NUTTX DEFCONFIG: ${Name}" FORCE)
	endif()
endforeach()

###############################################################################
# menuconfig
add_custom_target(menuconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/.config .config
	COMMAND
		APPSDIR=${NUTTX_APPS_DIR} kconfig-mconf Kconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different .config ${CMAKE_BINARY_DIR}/.config
	COMMAND
		${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/include/nuttx/config.h	# invalidate existing config
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	USES_TERMINAL
)

# qconfig
add_custom_target(qconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/.config .config
	COMMAND
		APPSDIR=${NUTTX_APPS_DIR} kconfig-qconf Kconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different .config ${CMAKE_BINARY_DIR}/.config	# copy back to binary directory
	COMMAND
		${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/include/nuttx/config.h	# invalidate existing config
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	USES_TERMINAL
)

# savedefconfig
add_custom_target(savedefconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/.config .config
	COMMAND
		APPSDIR=${NUTTX_APPS_DIR} kconfig-conf --savedefconfig defconfig.tmp Kconfig
	COMMAND
		sed -i -e "/CONFIG_APPS_DIR=/d" defconfig.tmp			# remove CONFIG_APPS_DIR
	COMMAND
		grep "CONFIG_ARCH=" .config >> defconfig.tmp || true		# preserve CONFIG_ARCH=
	COMMAND
		grep "^CONFIG_ARCH_CHIP_" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_CHIP_
	COMMAND
		grep "CONFIG_ARCH_BOARD=" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_BOARD
	COMMAND
		grep "^CONFIG_ARCH_CUSTOM" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_CUSTOM
	COMMAND
		#sed -i -e "/^#/d" defconfig.tmp					# remove commented lines
	COMMAND
		cat defconfig.tmp | LC_ALL=C sort | uniq > defconfig.updated	# sort and save back to original defconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different defconfig.updated ${NUTTX_DEFCONFIG}
	COMMAND
		${CMAKE_COMMAND} -E remove -f defconfig.tmp defconfig.updated			# cleanup
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
	COMMENT "Compressing .config and saving back to ${NUTTX_DEFCONFIG}"
	WORKING_DIRECTORY ${NUTTX_DIR}
)

# menuconfig and save
add_custom_target(menuconfig_save
	COMMAND
		${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} -- savedefconfig # this is hacky, but forces menuconfig to run before savedefconfig
	DEPENDS menuconfig
)
# qconfig and save
add_custom_target(qconfig_save
	COMMAND
		${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} -- savedefconfig # this is hacky, but forces qconfig to run before savedefconfig
	DEPENDS qconfig
)
