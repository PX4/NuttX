
set(CSRCS)

if("${CONFIG_ARMV7M_STACKCHECK}" STREQUAL "y")
	list(APPEND CSRCS up_stackcheck.c)
endif()

if("${CONFIG_ARMV7M_LAZYFPU}" STREQUAL "y")
	list(APPEND CSRCS gnu/up_lazyexception.S)
else()
	list(APPEND CSRCS gnu/up_exception.S)
endif()

if("${CONFIG_ARCH_RAMVECTORS}" STREQUAL "y")
	list(APPEND CSRCS up_ramvec_initialize.c up_ramvec_attach.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CSRCS up_mpu.c)
	if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
		list(APPEND CSRCS up_signal_dispatch.c)
		list(APPEND CSRCS gnu/up_signal_handler.S)
	endif()
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CSRCS gnu/up_fpu.S)
	list(APPEND CSRCS up_copyarmstate.c)
endif()

if("${CONFIG_ARMV7M_ITMSYSLOG}" STREQUAL "y")
	list(APPEND CSRCS up_itm_syslog.c)
endif()

if("${CONFIG_ARMV7M_DCACHE}" STREQUAL "y")
	list(APPEND CSRCS up_cache.c)
endif()

if("${CONFIG_ELF}" STREQUAL "y")
	list(APPEND CSRCS up_cache.c)
elseif("${CONFIG_MODULE}" STREQUAL "y")
	list(APPEND CSRCS up_cache.c)
endif()

set(ASRC_DIR "gnu")
if("${CONFIG_ARCH_TOOLCHAIN_IAR}" STREQUAL "y")
	set(ASRC_DIR "iar")
endif()

list(APPEND CSRCS
	${ASRC_DIR}/up_saveusercontext.S
	${ASRC_DIR}/up_fullcontextrestore.S
	${ASRC_DIR}/up_switchcontext.S
	${ASRC_DIR}/up_testset.S
	${ASRC_DIR}/up_fetchadd.S
	${ASRC_DIR}/vfork.S

	up_assert.c
	up_blocktask.c
	up_copyfullstate.c
	up_doirq.c
	up_hardfault.c
	up_initialstate.c
	up_memfault.c
	up_releasepending.c
	up_reprioritizertr.c
	up_schedulesigaction.c
	up_sigdeliver.c
	up_svcall.c
	up_systemreset.c
	up_trigger_irq.c
	up_unblocktask.c
	up_vectors.c
)
set_parent_srcs(${CSRCS})
