TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS += 
USR_CXXFLAGS_Linux += -D_LINUX -D_x86

# We install PvApi.h because the NDPluginColorConvert uses it for Bayer to RGB color conversion
INC += PvApi.h

LIBRARY_IOC_WIN32 += prosilica
#LIBRARY_IOC_cygwin32 += prosilica
LIBRARY_IOC_Linux += prosilica
LIB_SRCS += prosilica.cpp
LIB_SRCS += drvProsilicaEpics.c
LIB_LIBS += ADPlugin ADBase netCDF
LIB_LIBS += PvAPI ImageLib
LIB_LIBS += calc sscan autosave
LIB_LIBS += asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

# Note, the following files were manually copied from the Prosilica GigESDK/bin-pc directory
# to this directory after building with Visual Basic .NET 2003.
BIN_INSTALLS_WIN32    += ../PvAPI.dll
BIN_INSTALLS_cygwin32 += ../PvAPI.dll
LIB_INSTALLS_WIN32    += ../PvAPI.lib ../ImageLib.lib
LIB_INSTALLS_cygwin32 += ../PvAPI.lib ../ImageLib.lib
LIB_INSTALLS_linux    += ../libPvAPI.a ../libImageLib.a

#==================================
PROD_IOC_WIN32 += prosilicaApp
PROD_IOC_Linux += prosilicaApp
#PROD_IOC_cygwin32 += prosilicaApp
# <name>.dbd will be created from <name>Include.dbd
DBD += prosilicaApp.dbd
DBD += prosilicaSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
prosilicaApp_SRCS_DEFAULT += prosilicaApp_registerRecordDeviceDriver.cpp prosilicaAppMain.cpp
prosilicaApp_SRCS_vxWorks += prosilicaApp_registerRecordDeviceDriver.cpp

# Add locally compiled object code
prosilicaApp_LIBS += prosilica
prosilicaApp_LIBS += PvAPI ImageLib
prosilicaApp_LIBS += ADBase ADPlugin netCDF
prosilicaApp_LIBS += calc sscan autosave 
prosilicaApp_LIBS += asyn
prosilicaApp_LIBS += $(EPICS_BASE_IOC_LIBS)

#prosilicaApp_SYS_LIBS_cygwin32 += rpc
prosilicaApp_SYS_LIBS_Linux += tiff jpeg z
#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

