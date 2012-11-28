TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

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

PROD_IOC_WIN32 += prosilicaApp
PROD_IOC_Linux += prosilicaApp
PROD_IOC_Darwin += prosilicaApp

# <name>.dbd will be created from <name>Include.dbd
DBD += prosilicaApp.dbd
DBD += prosilicaSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += prosilicaApp_registerRecordDeviceDriver.cpp prosilicaAppMain.cpp

# Add locally compiled object code
PROD_LIBS += prosilica

include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

