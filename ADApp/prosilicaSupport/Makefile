TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The files in this directory are installed first because they are
# used both by the NDPluginColorConvert and by the Prosilica driver
INC += PvApi.h

# Note, the following files were manually copied from the Prosilica GigESDK/bin-pc directory
# to this directory after building
ifeq ($(T_A), win32-x86)
LIB_INSTALLS_WIN32    += ../os/win32-x86/PvAPI.lib
BIN_INSTALLS_WIN32    += ../os/win32-x86/PvAPI.dll
endif
ifeq ($(T_A), win32-x86_64)
LIB_INSTALLS_WIN32    += ../os/win32-x86_64/PvAPI.lib
BIN_INSTALLS_WIN32    += ../os/win32-x86_64/PvAPI.dll
endif
ifeq ($(T_A),cygwin-x86)
LIB_INSTALLS_cygwin32    += ../os/win32-x86/PvAPI.lib
BIN_INSTALLS_cygwin32    += ../os/win32-x86/PvAPI.dll
endif
ifeq ($(T_A), linux-x86)
LIB_INSTALLS_Linux += ../os/linux-x86/libPvAPI.a
endif
ifeq ($(T_A), linux-x86_64) 
LIB_INSTALLS_Linux += ../os/linux-x86_64/libPvAPI.a
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

