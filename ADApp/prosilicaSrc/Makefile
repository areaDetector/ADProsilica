TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS += 
#USR_CXXFLAGS +=

LIBRARY_IOC_WIN32 += prosilica
LIBRARY_IOC_cygwin32 += prosilica
LIB_SRCS += drvProsilica.c
LIB_LIBS += PvAPI ImageLib
LIB_LIBS += ccd
LIB_LIBS += sscan autosave
LIB_LIBS += asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

# Note, the following files were manually copied from the Prosilica GigESDK/bin-pc directory
# to this directory after building with Visual Basic .NET 2003.
BIN_INSTALLS_WIN32    += ../PvAPI.dll
BIN_INSTALLS_cygwin32 += ../PvAPI.dll
LIB_INSTALLS_WIN32    += ../PvAPI.lib ../ImageLib.lib
LIB_INSTALLS_cygwin32 += ../PvAPI.lib ../ImageLib.lib

#==================================
PROD_IOC_WIN32 += prosilicaApp
PROD_IOC_cygwin32 += prosilicaApp
# <name>.dbd will be created from <name>Include.dbd
DBD += prosilicaApp.dbd
DBD += prosilicaSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
prosilicaApp_SRCS_DEFAULT += prosilicaApp_registerRecordDeviceDriver.cpp prosilicaAppMain.cpp
prosilicaApp_SRCS_vxWorks += prosilicaApp_registerRecordDeviceDriver.cpp

# Add locally compiled object code
prosilicaApp_LIBS += prosilica
prosilicaApp_LIBS += PvAPI ImageLib
prosilicaApp_LIBS += areaDetector
prosilicaApp_LIBS += sscan autosave 
prosilicaApp_LIBS += asyn
prosilicaApp_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

