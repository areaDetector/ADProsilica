TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

USR_CXXFLAGS_Linux += -D_LINUX -D_x86
USR_CXXFLAGS_Darwin += -D_OSX -D_x86

# The following is needed on win32-x86-debug because the Prosilica library 
# wants to use LIBCMT, and that conflicts with LIBCMTD
ifeq ($(T_A), win32-x86-debug)
USR_LDFLAGS += /NODEFAULTLIB:LIBCMT
endif
ifeq ($(T_A), windows-x64-debug)
USR_LDFLAGS += /NODEFAULTLIB:LIBCMT
endif

LIBRARY_IOC_WIN32 += prosilica
LIBRARY_IOC_Linux += prosilica
LIBRARY_IOC_Darwin += prosilica
LIB_SRCS += prosilica.cpp

DBD += prosilicaSupport.dbd

#include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

