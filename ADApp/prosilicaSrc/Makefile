TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS += 
#USR_CXXFLAGS +=

LIBRARY_IOC_WIN32 += prosilicaCCD
LIBRARY_IOC_cygwin32 += prosilicaCCD
LIB_SRCS += drvProsilica.c
LIB_LIBS += PvAPI
LIB_LIBS += ccd
LIB_LIBS += sscan autosave
LIB_LIBS += asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

# Note, the following files were manually copied from the Prosilica GigESDK/bin-pc directory
# to this directory after building with Visual Basic .NET 2003.
BIN_INSTALLS_WIN32    += ../PvAPI.dll
BIN_INSTALLS_cygwin32 += ../PvAPI.dll
LIB_INSTALLS_WIN32    += ../PvAPI.lib
LIB_INSTALLS_cygwin32 += ../PvAPI.lib

#==================================
PROD_IOC_WIN32 += prosilicaCCDApp
PROD_IOC_cygwin32 += prosilicaCCDApp
# <name>.dbd will be created from <name>Include.dbd
DBD += prosilicaCCDApp.dbd
DBD += prosilicaCCDSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
prosilicaCCDApp_SRCS_DEFAULT += prosilicaCCDApp_registerRecordDeviceDriver.cpp prosilicaCCDMain.cpp
prosilicaCCDApp_SRCS_vxWorks += prosilicaCCDApp_registerRecordDeviceDriver.cpp

# Add locally compiled object code
prosilicaCCDApp_LIBS += prosilicaCCD
prosilicaCCDApp_LIBS += PvAPI
prosilicaCCDApp_LIBS += ccd
prosilicaCCDApp_LIBS += sscan autosave 
prosilicaCCDApp_LIBS += asyn
prosilicaCCDApp_LIBS += $(EPICS_BASE_IOC_LIBS)

prosilicaCCDApp_SYS_LIBS_cygwin32 += rpc

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

