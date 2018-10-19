
include(nuttx_parse_function_args)

define_property(GLOBAL PROPERTY NUTTX_LIBRARIES
                 BRIEF_DOCS "NuttX libs"
                 FULL_DOCS "List of all NuttX libraries"
                 )

#=============================================================================
#
#	nuttx_add_library
#
#	Wrapper of cmake add_library but with nuttx platform dependencies
#
function(nuttx_add_library target)
	add_library(${target} ${ARGN})

	# all PX4 libraries have access to parameters and uORB
	add_dependencies(${target} nuttx_context)

	set_property(GLOBAL APPEND PROPERTY NUTTX_LIBRARIES ${target})
endfunction()
