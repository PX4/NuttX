
set(CSRCS)

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CSRCS up_task_start.c up_pthread_start.c
	)
endif()

if("${CONFIG_STACK_COLORATION}" STREQUAL "y")
	list(APPEND CSRCS up_checkstack.c)
endif()

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CSRCS up_idle.c)
endif()

list(APPEND CSRCS
	up_allocateheap.c
	up_createstack.c
	#up_etherstub.c
	up_exit.c
	up_hostfs.c
	up_initialize.c
	up_interruptcontext.c
	up_lowputs.c
	up_mdelay.c
	up_modifyreg16.c
	up_modifyreg32.c
	up_modifyreg8.c
	up_puts.c
	up_releasestack.c
	up_semi_syslog.c
	up_stackframe.c
	up_udelay.c
	up_usestack.c
	up_vfork.c
)
set_parent_srcs(${CSRCS})
