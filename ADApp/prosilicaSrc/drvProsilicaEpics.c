/* drvProsilicaEpics.c
 *
 * This is the EPICS dependent code for the driver for Prosilica cameras (GigE and CameraLink).
.
 * By making this separate file for the EPICS dependent code the driver itself
 * only needs libCom from EPICS for OS-independence.
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created:  March 20, 2008
 *
 */
 
#include <iocsh.h>
#include <drvSup.h>
#include <epicsExport.h>

#include "ADInterface.h"
#include "drvProsilica.h"

/* Code for iocsh registration */

/* prosilicaConfig */
static const iocshArg prosilicaConfigArg0  = {"Port name", iocshArgString};
static const iocshArg prosilicaConfigArg1 = {"Unique Id", iocshArgInt};
static const iocshArg * const prosilicaConfigArgs[2] = {&prosilicaConfigArg0,
                                                        &prosilicaConfigArg1};
static const iocshFuncDef configprosilica = {"prosilicaConfig", 2, prosilicaConfigArgs};
static void configprosilicaCallFunc(const iocshArgBuf *args)
{
    prosilicaConfig(args[0].sval, args[1].ival);
}


static void prosilicaRegister(void)
{

    iocshRegister(&configprosilica,     configprosilicaCallFunc);
}

epicsExportRegistrar(prosilicaRegister);
