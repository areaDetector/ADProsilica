TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

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

PROD_NAME = prosilicaApp
PROD_IOC_WIN32  += $(PROD_NAME)
PROD_IOC_Linux  += $(PROD_NAME)
PROD_IOC_Darwin += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD +=  $(PROD_NAME).dbd
$(PROD_NAME)_DBD += prosilicaSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS += prosilica

# Add vendor library from prosilicaSupport
PROD_LIBS += PvAPI

# Seem to need to link readline on base 7.0.6.1
PROD_SYS_LIBS_Linux += readline

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

