#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <epicsFindSymbol.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <ellLib.h>
#include <iocsh.h>
#include <osiSock.h>  /* Needed for translating IP address to name */
#include <drvSup.h>
#include <epicsExport.h>

#define DEFINE_AREA_DETECTOR_PROTOTYPES 1
#include "ADParamLib.h"
#include "areaDetectorInterface.h"
#include "drvProsilica.h"
#include "PvAPI.h"

/* If we have any private driver commands they begin with ADFirstDriverCommand and should end
   with ADLastDriverCommand, which is used for setting the size of the parameter library table */
   
typedef enum {
   PSTrigger1 = ADFirstDriverParam,
   PSTrigger2,
   ADLastDriverParam
} PSParam_t;

/* This structure and array are used for conveniently looking up commands in ADFindParam() */
typedef struct {
    PSParam_t command;
    char *commandString;
} PSCommandStruct;

static PSCommandStruct PSCommands[] = {
    {PSTrigger1,   "TRIGGER1"},  
    {PSTrigger2,   "TRIGGER1"}  
};


ADDrvSET_t ADProsilica = 
  {
    18,
    ADReport,            /* Standard EPICS driver report function (optional) */
    ADInit,              /* Standard EPICS driver initialisation function (optional) */
    ADSetLog,            /* Defines an external logging function (optional) */
    ADOpen,              /* Driver open function */
    ADClose,             /* Driver close function */
    ADFindParam,         /* Parameter lookup function */
    ADSetInt32Callback,     /* Provides a callback function the driver can call when an int32 value updates */
    ADSetFloat64Callback,   /* Provides a callback function the driver can call when a float64 value updates */
    ADSetImageDataCallback, /* Provides a callback function the driver can call when the image data updates */
    ADGetInteger,        /* Pointer to function to get an integer value */
    ADSetInteger,        /* Pointer to function to set an integer value */
    ADGetDouble,         /* Pointer to function to get a double value */
    ADSetDouble,         /* Pointer to function to set a double value */
    ADGetString,         /* Pointer to function to get a string value */
    ADSetString,         /* Pointer to function to set a string value */
    ADGetImage,          /* Pointer to function to read image data */
    ADSetImage           /* Pointer to function to write image data */
  };

#define MAX_FRAMES  2  /* Number of frame buffers for PvApi */

epicsExportAddress(drvet, ADProsilica);

typedef struct ADHandle {
    /* The first set of items in this structure will be needed by all drivers */
    int camera;                        /* Index of this camera in list of controlled cameras */
    int wasFound;                      /* 1 if camera was found 0 if not */
    epicsMutexId prosilicaLock;        /* A Mutex to lock access to data structures. */
    ADLogFunc logFunc;                 /* These are for error and debug logging.*/
    void *logParam;
    PARAMS params;
    ADImageDataCallbackFunc pImageDataCallback;
    void *imageDataCallbackParam;

    /* These items are specific to the Prosilica API */
    tPvHandle PvHandle;                /* Handle for the Prosilica PvAPI library */
    tPvCameraInfo PvCameraInfo;
    tPvFrame frame[MAX_FRAMES];        /* Frame buffers. */
    tPvFrame *lastFrame;
    int framesRemaining;
    char sensorType[20];
    char IPAddress[50];
    int sensorBits;
    int sensorWidth;
    int sensorHeight;
} camera_t;

static int ADLogMsg(void * param, const ADLogMask_t logMask, const char *pFormat, ...);
static void PVDECL frameCallback(tPvFrame *frame);

#define PRINT   (pCamera->logFunc)
#define TRACE_FLOW    ADTraceFlow
#define TRACE_ERROR   ADTraceError
#define TRACE_IODRIVER  ADTraceIODriver

static int numCameras;

/* Pointer to array of controller strutures */
static camera_t *allCameras=NULL;

static void ADReport(int level)
{
    int i;
    DETECTOR_HDL pCamera;

    for(i=0; i<numCameras; i++) {
        pCamera = &allCameras[i];
        printf("Prosilica camera %d IP address=%s\n", i, pCamera->IPAddress);
        if (level > 0) {
            printf("  ID:                %ul\n", pCamera->PvCameraInfo.UniqueId);
            printf("  Serial number:     %s\n",  pCamera->PvCameraInfo.SerialString);
            printf("  Model:             %s\n",  pCamera->PvCameraInfo.DisplayName);
            printf("  Sensor type:       %s\n",  pCamera->sensorType);
            printf("  Sensor bits:       %d\n",  pCamera->sensorBits);
            printf("  Sensor width:      %d\n",  pCamera->sensorWidth);
            printf("  Sensor height:     %d\n",  pCamera->sensorHeight);
            printf("  Frame buffer size: %d\n",  pCamera->frame[0].ImageBufferSize);
        }
        if (level > 5) {
            printf("\nParameter library contents:\n");
            ADParam->dump(pCamera->params);
        }
    }   
}


static int ADInit(void)
{
    return AREA_DETECTOR_OK;
}

static int ADSetLog( DETECTOR_HDL pCamera, ADLogFunc logFunc, void * param )
{
    if (logFunc == NULL)
    {
        pCamera->logFunc=ADLogMsg;
        pCamera->logParam = NULL;
    }
    else
    {
        pCamera->logFunc=logFunc;
        pCamera->logParam = param;
    }
    return AREA_DETECTOR_OK;
}

static DETECTOR_HDL ADOpen(int card, char * param)
{
    DETECTOR_HDL pCamera;

    if (card >= numCameras) return(NULL);
    pCamera = &allCameras[card];
    if (pCamera->wasFound == 0) return(NULL);
    return pCamera;
}

static int ADClose(DETECTOR_HDL pCamera)
{
    return AREA_DETECTOR_OK;
}

static int ADFindParam( DETECTOR_HDL pDetector, const char *paramString, int *function )
{
    int i;
    int ncommands = sizeof(PSCommands)/sizeof(PSCommands[0]);

    for (i=0; i < ncommands; i++) {
        if (epicsStrCaseCmp(paramString, PSCommands[i].commandString) == 0) {
            *function = PSCommands[i].command;
            return AREA_DETECTOR_OK;
        }
    }
    return AREA_DETECTOR_ERROR;
}

static int ADSetInt32Callback(DETECTOR_HDL pCamera, ADInt32CallbackFunc callback, void * param)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    
    status = ADParam->setIntCallback(pCamera->params, callback, param);
    
    return(status);
}

static int ADSetFloat64Callback(DETECTOR_HDL pCamera, ADFloat64CallbackFunc callback, void * param)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    
    status = ADParam->setDoubleCallback(pCamera->params, callback, param);
    
    return(status);
}

static int ADSetImageDataCallback(DETECTOR_HDL pCamera, ADImageDataCallbackFunc callback, void * param)
{
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    pCamera->pImageDataCallback = callback;
    pCamera->imageDataCallbackParam = param;
    return AREA_DETECTOR_OK;
}

static int ADSetGeometry(DETECTOR_HDL pCamera)
{
    int status = AREA_DETECTOR_OK;
    tPvUint32 binX, binY, minY, minX, sizeX, sizeY;
    
    /* Get all of the current geometry parameters from the parameter library */
    status |= ADParam->getInteger(pCamera->params, ADBinX, &binX);
    status |= ADParam->getInteger(pCamera->params, ADBinY, &binY);
    status |= ADParam->getInteger(pCamera->params, ADMinX, &minX);
    status |= ADParam->getInteger(pCamera->params, ADMinY, &minY);
    status |= ADParam->getInteger(pCamera->params, ADSizeX, &sizeX);
    status |= ADParam->getInteger(pCamera->params, ADSizeY, &sizeY);
    
    status |= PvAttrUint32Set(pCamera->PvHandle, "BinningX", binX);
    status |= PvAttrUint32Set(pCamera->PvHandle, "BinningY", binY);
    status |= PvAttrUint32Set(pCamera->PvHandle, "RegionX", minX/binX);
    status |= PvAttrUint32Set(pCamera->PvHandle, "RegionY", minY/binY);
    status |= PvAttrUint32Set(pCamera->PvHandle, "Width",   sizeX/binX);
    status |= PvAttrUint32Set(pCamera->PvHandle, "Height",  sizeY/binY);
    
    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADSetGeometry error, status=%d\n", status);
    return(status);
}

static int ADGetGeometry(DETECTOR_HDL pCamera)
{
    int status = AREA_DETECTOR_OK;
    tPvUint32 binX, binY, minY, minX, sizeX, sizeY;

    status |= PvAttrUint32Get(pCamera->PvHandle, "BinningX", &binX);
    status |= PvAttrUint32Get(pCamera->PvHandle, "BinningY", &binY);
    status |= PvAttrUint32Get(pCamera->PvHandle, "RegionX",  &minX);
    status |= PvAttrUint32Get(pCamera->PvHandle, "RegionY",  &minY);
    status |= PvAttrUint32Get(pCamera->PvHandle, "Width",    &sizeX);
    status |= PvAttrUint32Get(pCamera->PvHandle, "Height",   &sizeY);
    
    status |= ADParam->setInteger(pCamera->params, ADBinX,  binX);
    status |= ADParam->setInteger(pCamera->params, ADBinY,  binY);
    status |= ADParam->setInteger(pCamera->params, ADMinX,  minX*binX);
    status |= ADParam->setInteger(pCamera->params, ADMinY,  minY*binY);
    status |= ADParam->setInteger(pCamera->params, ADImageSizeX, sizeX);
    status |= ADParam->setInteger(pCamera->params, ADImageSizeY, sizeY);
    
    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADGetGeometry error, status=%d\n", status);
    return(status);
}

static ADReadParameters(DETECTOR_HDL pCamera)
{
    int status = AREA_DETECTOR_OK;
    tPvUint32 intVal;
    tPvFloat32 fltVal;
    double dval;
    int nchars;
    char buffer[20];

    status |= PvAttrUint32Get(pCamera->PvHandle, "TotalBytesPerFrame", &intVal);
    ADParam->setInteger(pCamera->params, ADImageSize, intVal);

    intVal = -1;
    status |= PvAttrEnumGet(pCamera->PvHandle, "PixelFormat", buffer, sizeof(buffer), &nchars);
    if (!strcmp(buffer, "Mono8")) intVal = ADUInt8;
    else if (!strcmp(buffer, "Mono16")) intVal = ADUInt16;
    /* We don't support color modes yet */
    status |= ADParam->setInteger(pCamera->params, ADDataType, intVal);
    
    status |= ADGetGeometry(pCamera);

    status |= PvAttrUint32Get(pCamera->PvHandle, "AcquisitionFrameCount", &intVal);
    status |= ADParam->setInteger(pCamera->params, ADNumFrames, intVal);

    status |= PvAttrEnumGet(pCamera->PvHandle, "AcquisitionMode", buffer, sizeof(buffer), &nchars);
    if      (!strcmp(buffer, "SingleFrame")) intVal = ADSingleFrame;
    else if (!strcmp(buffer, "MultiFrame"))  intVal = ADMultipleFrame;
    else if (!strcmp(buffer, "Recorder"))    intVal = ADMultipleFrame;
    else if (!strcmp(buffer, "Continuous"))  intVal = ADContinuousFrame;
    status |= ADParam->setInteger(pCamera->params, ADFrameMode, intVal);

    /* Prosilica does not support more than 1 exposure per frame */
    status |= ADParam->setInteger(pCamera->params, ADNumExposures, 1);

    /* Prosilica uses integer microseconds */
    status |= PvAttrUint32Get(pCamera->PvHandle, "ExposureValue", &intVal);
    dval = intVal / 1.e6;
    status |= ADParam->setDouble(pCamera->params, ADAcquireTime, dval);

    /* Prosilica uses a frame rate in Hz */
    status |= PvAttrFloat32Get(pCamera->PvHandle, "FrameRate", &fltVal);
    dval = 1. / fltVal;
    status |= ADParam->setDouble(pCamera->params, ADAcquirePeriod, dval);

    /* Prosilica uses an integer value */
    status |= PvAttrUint32Get(pCamera->PvHandle, "GainValue", &intVal);
    dval = intVal;
    status |= ADParam->setDouble(pCamera->params, ADGain, dval);

    /* Call the callbacks to update the values in higher layers */
    ADParam->callCallbacks(pCamera->params);
    
    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADReadParameters error, status=%d\n", status);
    return(status);
}

static int ADGetInteger(DETECTOR_HDL pCamera, int function, int * value)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    
    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getInteger(pCamera->params, function, value);
    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADGetInteger error, status=%d function=%d, value=%d\n", 
                      status, function, *value);
    return(status);
}

static int ADSetInteger(DETECTOR_HDL pCamera, int function, int value)
{
    int status = AREA_DETECTOR_OK;
    tPvUint32 intVal = value;

    if (pCamera == NULL) return AREA_DETECTOR_ERROR;

    /* Set the parameter in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= ADParam->setInteger(pCamera->params, function, value);

    switch (function) {
    case ADBinX:
    case ADBinY:
    case ADMinX:
    case ADSizeX:
    case ADMinY:
    case ADSizeY:
        /* These commands change the chip readout geometry.  We need to cache them and apply them in the
         * correct order */
        status |= ADSetGeometry(pCamera);
        break;
    case ADNumFrames:
        status |= PvAttrUint32Set(pCamera->PvHandle, "AcquisitionFrameCount", intVal);
        break;
    case ADFrameMode:
        switch(value) {
        case ADSingleFrame:
            status |= PvAttrEnumSet(pCamera->PvHandle, "AcquisitionMode", "SingleFrame");
            break;
        case ADMultipleFrame:
            status |= PvAttrEnumSet(pCamera->PvHandle, "AcquisitionMode", "MultiFrame");
            break;
        case ADContinuousFrame:
            status |= PvAttrEnumSet(pCamera->PvHandle, "AcquisitionMode", "Continuous");
            break;
        }
        break;
    case ADAcquire:
        if (value) {
            /* We need to set the number of frames we expect to collect, so the frame callback function
               can know when acquisition is complete.  We need to find out what mode we are in and how
               many frames have been requested.  If we are in continuous mode then set the number of
               remaining frames to -1. */
            int frameMode, numFrames;
            status |= ADParam->getInteger(pCamera->params, ADFrameMode, &frameMode);
            status |= ADParam->getInteger(pCamera->params, ADNumFrames, &numFrames);
            switch(frameMode) {
            case ADSingleFrame:
                pCamera->framesRemaining = 1;
                break;
            case ADMultipleFrame:
                pCamera->framesRemaining = numFrames;
                break;
            case ADContinuousFrame:
                pCamera->framesRemaining = -1;
                break;
            }
            ADParam->setInteger(pCamera->params, ADStatus, ADStatusAcquire);
            status |= PvCommandRun(pCamera->PvHandle, "AcquisitionStart");
        } else {
            ADParam->setInteger(pCamera->params, ADStatus, ADStatusIdle);
            status |= PvCommandRun(pCamera->PvHandle, "AcquisitionAbort");
        }
        break;
    default:
        break;
    }
    
    /* Read the camera parameters and do callbacks */
    status |= ADReadParameters(pCamera);
    
    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADSetInteger error, status=%d, function=%d, value=%d\n", 
                      status, function, value);
    return status;
}


static int ADGetDouble(DETECTOR_HDL pCamera, int function, double * value)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;

    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getDouble(pCamera->params, function, value);
    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADGetDouble error, status=%d, function=%d, value=%f\n", 
                      status, function, *value);
    return(status);
}

static int ADSetDouble(DETECTOR_HDL pCamera, int function, double value)
{
    int status = AREA_DETECTOR_OK;
    tPvUint32 intVal;
    tPvFloat32 fltVal;

    if (pCamera == NULL) return AREA_DETECTOR_ERROR;

    /* Set the parameter in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= ADParam->setDouble(pCamera->params, function, value);

    switch (function) {
    case ADAcquireTime:
        /* Prosilica uses integer microseconds */
        intVal = (tPvUint32) (value * 1e6);
        status |= PvAttrUint32Set(pCamera->PvHandle, "ExposureValue", intVal);
        break;
    case ADAcquirePeriod:
        /* Prosilica uses a frame rate in Hz */
        if (value == 0.) value = .01;
        fltVal = (tPvFloat32) (1. / value);
        status |= PvAttrFloat32Set(pCamera->PvHandle, "FrameRate", fltVal);
        break;
    case ADGain:
        /* Prosilica uses an integer value */
        intVal = (tPvUint32) (value);
        status |= PvAttrUint32Set(pCamera->PvHandle, "GainValue", intVal);
        break;
    default:
        break;
    }

    /* Read the camera parameters and do callbacks */
    status |= ADReadParameters(pCamera);
    
    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADSetDouble error, status=%d, function=%d, value=%f\n", 
                      status, function, value);
    return status;
}

static int ADGetString(DETECTOR_HDL pCamera, int function, int maxChars, char * value)
{
    int status = AREA_DETECTOR_OK;
   
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;

    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getString(pCamera->params, function, maxChars, value);

    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADGetString error, status=%d, function=%d, value=%s\n", 
                      status, function, value);
    return(status);
}

static int ADSetString(DETECTOR_HDL pCamera, int function, const char *value)
{
    int status = AREA_DETECTOR_OK;

    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    switch (function) {
    default:
        break;
    }
    if (status) PRINT(pCamera->logParam, TRACE_ERROR, "ADGetString error, status=%d, function=%d, value=%s\n", 
                      status, function, value);
    return status;
}

static int ADGetImage(DETECTOR_HDL pCamera, int maxBytes, void *buffer)
{
    tPvFrame *pFrame; 
    int nCopy;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    if (pCamera->lastFrame == NULL) return AREA_DETECTOR_ERROR;
    pFrame = pCamera->lastFrame;    
    nCopy = pFrame->ImageSize;
    if (nCopy > maxBytes) nCopy = maxBytes;
    memcpy(buffer, pFrame->ImageBuffer, nCopy);
    
    return AREA_DETECTOR_OK;
}

static int ADSetImage(DETECTOR_HDL pCamera, int maxBytes, void *buffer)
{

    if (pCamera == NULL) return AREA_DETECTOR_ERROR;

    /* The Prosilica does not allow downloading image data */    
    return AREA_DETECTOR_ERROR;
}



static void PVDECL frameCallback(tPvFrame *pFrame)
{
    int status = AREA_DETECTOR_OK;
    ADDataType_t dataType;
    DETECTOR_HDL pCamera = (DETECTOR_HDL) pFrame->Context[0];
    
     /* Call the callback function for image data */
    if (pCamera->pImageDataCallback) {
        /* Convert from the PvApi data types to ADDataType */
        switch(pFrame->Format) {
        case ePvFmtMono8:
        case ePvFmtBayer8:
            dataType = ADUInt8;
            break;
        case ePvFmtMono16:
        case ePvFmtBayer16:
            dataType = ADUInt16;
            break;
        default:
            /* Note, this is wrong it does not work for ePvFmtRgb48, which is 48 bits */
            dataType = ADUInt32;
        }
        pCamera->pImageDataCallback(pCamera->imageDataCallbackParam, 
                                    (void *)pFrame->ImageBuffer,
                                    dataType, pFrame->ImageSize, pFrame->Width, pFrame->Height);
    }

    /* Set pointer that ADGetImage uses */
    pCamera->lastFrame = pFrame;
    
    /* See if acquisition is done */
    if (pCamera->framesRemaining > 0) pCamera->framesRemaining--;
    if (pCamera->framesRemaining == 0) {
        ADParam->setInteger(pCamera->params, ADAcquire, 0);
        ADParam->setInteger(pCamera->params, ADStatus, ADStatusIdle);
        ADParam->callCallbacks(pCamera->params);
    }

    /* Queue this frame to run again */
    status = PvCaptureQueueFrame(pCamera->PvHandle, pFrame, frameCallback); 
}

static int ADLogMsg(void * param, const ADLogMask_t mask, const char *pFormat, ...)
{

    va_list     pvar;
    int         nchar;

    va_start(pvar, pFormat);
    nchar = vfprintf(stdout,pFormat,pvar);
    va_end (pvar);
    printf("\n");
    return(nchar);
}


int prosilicaSetup(int num_cameras)   /* number of Prosilica cameras in system.  */
{

    if (num_cameras < 1) {
        printf("prosilicaSetup, num_cameras must be > 0\n");
        return AREA_DETECTOR_ERROR;
    }
    numCameras = num_cameras;
    allCameras = (camera_t *)calloc(numCameras, sizeof(camera_t)); 
    return AREA_DETECTOR_OK;
}


int prosilicaConfig(int camera,     /* Camera number */
                    char *ipAddr)   /* IP address of this camera.  If NULL then use first camera found */

{
    DETECTOR_HDL pCamera;
    tPvIpSettings ipSettings;
    int nFound;
    int status = AREA_DETECTOR_OK;
    int nchars;
    struct in_addr inetAddr;
    tPvFrame *pFrame;
    int i;
    size_t maxBytes;
    int bytesPerPixel;

    if (numCameras < 1) {
        printf("prosilicaConfig: no Prosilica cameras allocated, call prosilicaSetup first\n");
        return AREA_DETECTOR_ERROR;
    }
    if ((camera < 0) || (camera >= numCameras)) {
        printf("prosilicaConfig: camera must in range 0 to %d\n", numCameras-1);
        return AREA_DETECTOR_ERROR;
    }
    pCamera = &allCameras[camera];
    pCamera->camera = camera;
    
    /* Set the local log function, may be changed by higher layers */
    ADSetLog(pCamera, NULL, NULL);

    /* Initialize the Prosilica PvAPI library */
    status = PvInitialize();
    if (status) {
        printf("prosilicaConfig: PvInitialize failed for camera %d\n", camera);
        return AREA_DETECTOR_ERROR;
    }
    
    /* If the IP address of the camera was specified then translate to number and
     * open it explicity. */
    if (ipAddr && strlen(ipAddr)) {
        status = hostToIPAddr(ipAddr, &inetAddr);
        if (status) {
            printf("prosilicaConfig: IP address not found %s\n", ipAddr);
            return AREA_DETECTOR_ERROR;
        }
        status = PvCameraInfoByAddr(inetAddr.s_addr, &pCamera->PvCameraInfo, &ipSettings);
        if (status) {
            printf("prosilicaConfig: Cannot find camera %s\n", ipAddr);
            return AREA_DETECTOR_ERROR;
        }
    } else {
        /* The ipAddr was not specified, so get a list of all Prosilica cameras and open
         * the first one */
        nFound = PvCameraList(&pCamera->PvCameraInfo, 1, NULL);
        if (nFound <= 0) {
            printf("prosilicaConfig: No Prosilica cameras found\n");
            return AREA_DETECTOR_ERROR;
        }
    }
    if ((pCamera->PvCameraInfo.PermittedAccess & ePvAccessMaster) == 0) {
        printf("prosilicaConfig: Cannot get control of first Prosilica camera\n");
        return AREA_DETECTOR_ERROR;
    }
    status = PvCameraOpen(pCamera->PvCameraInfo.UniqueId, ePvAccessMaster, &pCamera->PvHandle);
    if (status) {
        printf("prosilicaConfig: unable to open camera %d\n", camera);
        return AREA_DETECTOR_ERROR;
    }
    
    /* Initialize the frame buffers and queue them */
    status = PvCaptureStart(pCamera->PvHandle);
    if (status) {
        printf("prosilicaConfig: unable to start capture\n");
        return AREA_DETECTOR_ERROR;
    }
    /* We allocate image buffers that are large enough for the biggest possible image.
       This is simpler than reallocating when readout parameters change.  It is also safer,
       since changing readout parameters happens instantly, but there will still be frames
       queued with the wrong size */
    /* Query the parameters of the image sensor */
    status = PvAttrEnumGet(pCamera->PvHandle, "SensorType", pCamera->sensorType, 
                             sizeof(pCamera->sensorType), &nchars);
    status |= PvAttrUint32Get(pCamera->PvHandle, "SensorBits", &pCamera->sensorBits);
    status |= PvAttrUint32Get(pCamera->PvHandle, "SensorWidth", &pCamera->sensorWidth);
    status |= PvAttrUint32Get(pCamera->PvHandle, "SensorHeight", &pCamera->sensorHeight);
    status |= PvAttrStringGet(pCamera->PvHandle, "DeviceIPAddress", pCamera->IPAddress, 
                              sizeof(pCamera->IPAddress), &nchars);
    if (status) {
        printf("prosilicaConfig: unable to get sensor data\n");
        return AREA_DETECTOR_ERROR;
    }
    
    bytesPerPixel = (pCamera->sensorBits-1)/8 + 1;
    /* If the camera supports color then there can be 4 values per pixel? */
    if (strcmp(pCamera->sensorType, "Mono") != 0) bytesPerPixel *= 4;
    maxBytes = pCamera->sensorWidth * pCamera->sensorHeight * bytesPerPixel;    
    for (i=0; i<MAX_FRAMES; i++) {
        pFrame = &pCamera->frame[i];
        pFrame->ImageBuffer = malloc(maxBytes);
        if (!pFrame->ImageBuffer) {
            printf("prosilicaConfig: unable to allocate ImageBuffer\n");
            return AREA_DETECTOR_ERROR;
        }
        pFrame->ImageBufferSize = maxBytes;
        pFrame->AncillaryBuffer = NULL;
        pFrame->AncillaryBufferSize = 0;
        pFrame->Context[0] = (void *)pCamera;
        status = PvCaptureQueueFrame(pCamera->PvHandle, pFrame, frameCallback); 
        if (status) {
            printf("prosilicaConfig: unable to queue frame %d\n", i);
            return AREA_DETECTOR_ERROR;
        }
    } 
    
    /* Initialize the parameter library */
    pCamera->params = ADParam->create(0, ADLastDriverParam);
    if (!pCamera->params) {
        printf("prosilicaConfig: unable to create parameter library\n");
        return AREA_DETECTOR_ERROR;
    }
    
    /* Set some parameters that need to be initialized */
    status =  ADParam->setString(pCamera->params, ADManufacturer, "Prosilica");
    status |= ADParam->setString(pCamera->params, ADModel, pCamera->PvCameraInfo.DisplayName);
    status |= ADParam->setInteger(pCamera->params, ADStatus, ADStatusIdle);
    status |= ADParam->setInteger(pCamera->params, ADAcquire, 0);
    status |= ADParam->setInteger(pCamera->params, ADSizeX, pCamera->sensorWidth);
    status |= ADParam->setInteger(pCamera->params, ADSizeY, pCamera->sensorHeight);
    status |= ADParam->setInteger(pCamera->params, ADMaxSizeX, pCamera->sensorWidth);
    status |= ADParam->setInteger(pCamera->params, ADMaxSizeY, pCamera->sensorHeight);
    if (status) {
        printf("prosilicaConfig: unable to set camera string parameters\n");
        return AREA_DETECTOR_ERROR;
    }
    
     /* Read the current camera settings */
    status = ADReadParameters(pCamera);
    if (status) {
        printf("prosilicaConfig: unable to read camera settings\n");
        return AREA_DETECTOR_ERROR;
    }

    /* We found the camera and everything is OK.  Set the flag. */
    pCamera->wasFound = 1;
    
    return AREA_DETECTOR_OK;
}

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
static const iocshArg prosilicaConfigArg0 = {"Camera being configured", iocshArgInt};
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

