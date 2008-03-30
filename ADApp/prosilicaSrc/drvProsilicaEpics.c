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

/* prosilicaSetup */
static const iocshArg prosilicaSetupArg0 = {"Number of Prosilica cameras", iocshArgInt};
static const iocshArg * const prosilicaSetupArgs[1] =  {&prosilicaSetupArg0};
static const iocshFuncDef setupprosilica = {"prosilicaSetup", 1, prosilicaSetupArgs};
static void setupprosilicaCallFunc(const iocshArgBuf *args)
{
    prosilicaSetup(args[0].ival);
}


/* prosilicaConfig */
static const iocshArg prosilicaConfigArg0 = {"Camera # being configured", iocshArgInt};
static const iocshArg prosilicaConfigArg1 = {"IP address", iocshArgString};
static const iocshArg * const prosilicaConfigArgs[2] = {&prosilicaConfigArg0,
                                                        &prosilicaConfigArg1};
static const iocshFuncDef configprosilica = {"prosilicaConfig", 2, prosilicaConfigArgs};
static void configprosilicaCallFunc(const iocshArgBuf *args)
{
    prosilicaConfig(args[0].ival, args[1].sval);
}


static void prosilicaRegister(void)
{

    iocshRegister(&setupprosilica,      setupprosilicaCallFunc);
    iocshRegister(&configprosilica,     configprosilicaCallFunc);
}

epicsExportRegistrar(prosilicaRegister);
epicsExportAddress(drvet, ADProsilica);
