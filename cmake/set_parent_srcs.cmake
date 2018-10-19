
macro(set_parent_srcs)

	get_filename_component(CURR_DIR ${CMAKE_CURRENT_LIST_DIR} NAME)

	foreach(file ${ARGN})
		list(APPEND SRCS ${CURR_DIR}/${file})
		set(SRCS "${SRCS}" PARENT_SCOPE)
	endforeach()

endmacro()
