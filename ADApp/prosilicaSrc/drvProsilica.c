/* drvProsilica.c
 *
 * This is a driver for Prosilica cameras (GigE and CameraLink).
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created:  March 20, 2008
 *
 */
 
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <osiSock.h>  /* Needed for translating IP address to name */

#define DEFINE_AREA_DETECTOR_PROTOTYPES 1
#include "ADParamLib.h"
#include "ADInterface.h"
#include "ADUtils.h"
#include "drvProsilica.h"
#include "PvAPI.h"
#include "ImageLib.h"

/* If we have any private driver commands they begin with ADFirstDriverCommand and should end
   with ADLastDriverCommand, which is used for setting the size of the parameter library table */
typedef enum {
    /* These parameters describe the trigger modes of the Prosilica
     * They must agree with the values in the mbbo/mbbi records in
     * the Prosilca database. */
    PSTriggerStartFreeRun,
    PSTriggerStartSyncIn1,
    PSTriggerStartSyncIn2,
    PSTriggerStartSyncIn3,
    PSTriggerStartSyncIn4,
    PSTriggerStartFixedRate,
    PSTriggerStartSoftware
} PSTriggerStartMode_t;

static char *PSTriggerStartStrings[] = {
    "Freerun","SyncIn1","SyncIn2","SyncIn3","SyncIn4","FixedRate","Software"
};
 
#define NUM_START_TRIGGER_MODES (sizeof(PSTriggerStartStrings) / sizeof(PSTriggerStartStrings[0]))

typedef enum {
    /* These parameters are for the camera statistics */
    PSReadStatistics = ADFirstDriverParam,
    PSStatDriverType,
    PSStatFilterVersion,
    PSStatFrameRate,
    PSStatFramesCompleted,
    PSStatFramesDropped,
    PSStatPacketsErroneous,
    PSStatPacketsMissed,
    PSStatPacketsReceived,
    PSStatPacketsRequested,
    PSStatPacketsResent,
    ADLastDriverParam
} DetParam_t;

/* This structure and array are used for conveniently looking up commands in ADFindParam() */
typedef struct {
    DetParam_t command;
    char *commandString;
} DetCommandStruct;

static DetCommandStruct DetCommands[] = {
    {PSReadStatistics,        "PS_READ_STATISTICS"},
    {PSStatDriverType,        "PS_DRIVER_TYPE"},
    {PSStatFilterVersion,     "PS_FILTER_VERSION"},
    {PSStatFrameRate,         "PS_FRAME_RATE"},
    {PSStatFramesCompleted,   "PS_FRAMES_COMPLETED"},
    {PSStatFramesDropped,     "PS_FRAMES_DROPPED"},
    {PSStatPacketsErroneous,  "PS_PACKETS_ERRONEOUS"},
    {PSStatPacketsMissed,     "PS_PACKETS_MISSED"},
    {PSStatPacketsReceived,   "PS_PACKETS_RECEIVED"},
    {PSStatPacketsRequested,  "PS_PACKETS_REQUESTED"},
    {PSStatPacketsResent,     "PS_PACKETS_RESENT"}
};

static char *driverName = "drvProsilica";

ADDrvSet_t ADProsilica = 
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
    ADSetStringCallback,    /* Provides a callback function the driver can call when a float64 value updates */
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

typedef struct ADHandle {
    /* The first set of items in this structure will be needed by all drivers */
    int camera;                        /* Index of this camera in list of controlled cameras */
    epicsMutexId mutexId;              /* A Mutex to lock access to data structures. */
    ADLogFunc logFunc;                 /* These are for error and debug logging.*/
    void *logParam;
    PARAMS params;
    ADImageDataCallbackFunc pImageDataCallback;
    void *imageDataCallbackParam;

    /* These items are specific to the Prosilica API */
    tPvHandle PvHandle;                /* Handle for the Prosilica PvAPI library */
    char *ipAddr;
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

#define PRINT   (pCamera->logFunc)

static int numCameras;

/* Pointer to array of controller strutures */
static camera_t *allCameras=NULL;


static int PSWriteFile(DETECTOR_HDL pCamera)
{
    /* Writes last frame to disk as a TIFF file. */
    int status = AREA_DETECTOR_OK, tiffStatus;
    char fullFileName[MAX_FILENAME_LEN];
    int fileFormat;
    int oldSize, actualSize;
    tPvFrame *pFrame = pCamera->lastFrame;

    if (pFrame == NULL) return AREA_DETECTOR_ERROR;

    status |= ADUtils->createFileName(pCamera->params, MAX_FILENAME_LEN, fullFileName);
    if (status) { 
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:PSWriteFile error creating full file name, fullFileName=%s, status=%d\n", 
              driverName, fullFileName, status);
        return(status);
    }
    
    /* There is a bug in ImageWriteTiff, it crashes if the ImageBufferSize is not the actual image size.
     * Temporarily replace the actual buffer size with the size of the current frame. */
    status |= PvAttrUint32Get(pCamera->PvHandle, "TotalBytesPerFrame", &actualSize);
    oldSize = pFrame->ImageBufferSize;
    pFrame->ImageBufferSize = actualSize;
    
    status |= ADParam->getInteger(pCamera->params, ADFileFormat, &fileFormat);
    /* We only support writing in TIFF format for now */
    tiffStatus = ImageWriteTiff(fullFileName, pFrame);
    /* Restore size */
    pFrame->ImageBufferSize = oldSize;
    if (tiffStatus != 1) status |= AREA_DETECTOR_ERROR;
    status |= ADParam->setString(pCamera->params, ADFullFileName, fullFileName);
    return(status);
}

static void PVDECL PSFrameCallback(tPvFrame *pFrame)
{
    int status = AREA_DETECTOR_OK;
    ADDataType_t dataType;
    int autoSave;
    DETECTOR_HDL pCamera = (DETECTOR_HDL) pFrame->Context[0];

    /* If this callback is coming from a shutdown operation rather than normal collection, 
     * we will not be able to take the mutex and things will hang.  Prevent this by looking at the frame
     * status and returning immediately if it is and in that case the mutex has already been taken.  Just return in
     * that case */
    if (pFrame->Status == ePvErrCancelled) return;

    epicsMutexLock(pCamera->mutexId);
    
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
                                    dataType, pFrame->Width, pFrame->Height);
    }

    /* Set pointer that ADGetImage uses */
    pCamera->lastFrame = pFrame;
    
    /* See if acquisition is done */
    if (pCamera->framesRemaining > 0) pCamera->framesRemaining--;
    if (pCamera->framesRemaining == 0) {
        ADParam->setInteger(pCamera->params, ADAcquire, 0);
        ADParam->setInteger(pCamera->params, ADStatus, ADStatusIdle);
     }
    
    /* If autoSave is set then save the image */
    status = ADParam->getInteger(pCamera->params, ADAutoSave, &autoSave);
    if (autoSave) status = PSWriteFile(pCamera);

    /* Update any changed parameters */
    ADParam->callCallbacks(pCamera->params);
    
    /* Queue this frame to run again */
    status = PvCaptureQueueFrame(pCamera->PvHandle, pFrame, PSFrameCallback); 
    epicsMutexUnlock(pCamera->mutexId);
}

static int PSSetGeometry(DETECTOR_HDL pCamera)
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
    
    if (status) PRINT(pCamera->logParam, ADTraceError, 
                      "%s:PSSetGeometry error, status=%d\n", 
                      driverName, status);
    return(status);
}

static int PSGetGeometry(DETECTOR_HDL pCamera)
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
    
    if (status) PRINT(pCamera->logParam, ADTraceError, 
                      "%s:PSGetGeometry error, status=%d\n", 
                      driverName, status);
    return(status);
}

static PSReadStats(DETECTOR_HDL pCamera)
{
    int status = AREA_DETECTOR_OK;
    char buffer[50];
    int nchars;
    epicsUInt32 uval;
    float fval;
    
    status |= PvAttrEnumGet      (pCamera->PvHandle, "StatDriverType", buffer, sizeof(buffer), &nchars);
    status |= ADParam->setString (pCamera->params,  PSStatDriverType, buffer);
    status |= PvAttrStringGet    (pCamera->PvHandle, "StatFilterVersion", buffer, sizeof(buffer), &nchars);
    status |= ADParam->setString (pCamera->params,  PSStatFilterVersion, buffer);
    status |= PvAttrFloat32Get   (pCamera->PvHandle, "StatFrameRate", &fval);
    status |= ADParam->setDouble (pCamera->params,  PSStatFrameRate, fval);
    status |= PvAttrUint32Get    (pCamera->PvHandle, "StatFramesCompleted", &uval);
    status |= ADParam->setInteger(pCamera->params,  PSStatFramesCompleted, (int)uval);
    status |= PvAttrUint32Get    (pCamera->PvHandle, "StatFramesDropped", &uval);
    status |= ADParam->setInteger(pCamera->params,  PSStatFramesDropped, (int)uval);
    status |= PvAttrUint32Get    (pCamera->PvHandle, "StatPacketsErroneous", &uval);
    status |= ADParam->setInteger(pCamera->params,  PSStatPacketsErroneous, (int)uval);
    status |= PvAttrUint32Get    (pCamera->PvHandle, "StatPacketsMissed", &uval);
    status |= ADParam->setInteger(pCamera->params,  PSStatPacketsMissed, (int)uval);
    status |= PvAttrUint32Get    (pCamera->PvHandle, "StatPacketsReceived", &uval);
    status |= ADParam->setInteger(pCamera->params,  PSStatPacketsReceived, (int)uval);
    status |= PvAttrUint32Get    (pCamera->PvHandle, "StatPacketsRequested", &uval);
    status |= ADParam->setInteger(pCamera->params,  PSStatPacketsRequested, (int)uval);
    status |= PvAttrUint32Get    (pCamera->PvHandle, "StatPacketsResent", &uval);
    status |= ADParam->setInteger(pCamera->params,  PSStatPacketsResent, (int)uval);
    if (status) PRINT(pCamera->logParam, ADTraceError, 
                      "%s:PSReadStatistics error, status=%d\n", 
                      driverName, status);
    return(status);
}

static PSReadParameters(DETECTOR_HDL pCamera)
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
    
    status |= PSGetGeometry(pCamera);

    status |= PvAttrUint32Get(pCamera->PvHandle, "AcquisitionFrameCount", &intVal);
    status |= ADParam->setInteger(pCamera->params, ADNumFrames, intVal);

    status |= PvAttrEnumGet(pCamera->PvHandle, "AcquisitionMode", buffer, sizeof(buffer), &nchars);
    if      (!strcmp(buffer, "SingleFrame")) intVal = ADFrameSingle;
    else if (!strcmp(buffer, "MultiFrame"))  intVal = ADFrameMultiple;
    else if (!strcmp(buffer, "Recorder"))    intVal = ADFrameMultiple;
    else if (!strcmp(buffer, "Continuous"))  intVal = ADFrameContinuous;
    else {intVal=0; status |= AREA_DETECTOR_ERROR;}
    status |= ADParam->setInteger(pCamera->params, ADFrameMode, intVal);

    status |= PvAttrEnumGet(pCamera->PvHandle, "FrameStartTriggerMode", buffer, sizeof(buffer), &nchars);
    for (intVal=0; intVal<NUM_START_TRIGGER_MODES; intVal++) {
        if (strcmp(buffer, PSTriggerStartStrings[intVal]) == 0) {
            status |= ADParam->setInteger(pCamera->params, ADTriggerMode, intVal);
            break;
        }
    }
    if (intVal == NUM_START_TRIGGER_MODES) {
        status |= ADParam->setInteger(pCamera->params, ADTriggerMode, 0);
        status |= AREA_DETECTOR_ERROR;
    }
    
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
    
    if (status) PRINT(pCamera->logParam, ADTraceError, 
                      "%s:PSReadParameters error, status=%d\n", 
                      driverName, status);
    return(status);
}

static int PSDisconnect(DETECTOR_HDL pCamera)
{
    int status = AREA_DETECTOR_OK;

    status |= PvCaptureQueueClear(pCamera->PvHandle);
    status |= PvCaptureEnd(pCamera->PvHandle);
    status |= PvCameraClose(pCamera->PvHandle);
    PRINT(pCamera->logParam, ADTraceFlow, 
          "%s:PSDisconnect: disconnecting camera %s\n", 
          driverName, pCamera->ipAddr);
    if (status) {
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:PSDisonnect: unable to close camera %s\n",
              driverName, pCamera->ipAddr);
       return AREA_DETECTOR_ERROR;
    }
    status |= ADParam->setInteger(pCamera->params, ADConnect, 0);
    return(status);
}

static int PSConnect(DETECTOR_HDL pCamera)
{
    struct in_addr inetAddr;
    int status = AREA_DETECTOR_OK;
    tPvIpSettings ipSettings;
    int nchars;
    tPvFrame *pFrame;
    int i;
    size_t maxBytes;
    int bytesPerPixel;

    /* Translate IP adress to number. */
    status = hostToIPAddr(pCamera->ipAddr, &inetAddr);
    if (status) {
        PRINT(pCamera->logParam, ADTraceError, 
              "s:PSConnect: IP address not found %s\n", 
              driverName, pCamera->ipAddr);
        return AREA_DETECTOR_ERROR;
    }
    status = PvCameraInfoByAddr(inetAddr.s_addr, &pCamera->PvCameraInfo, &ipSettings);
    if (status) {
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:PSConnect: Cannot find camera %s\n", 
              driverName, pCamera->ipAddr);
        return AREA_DETECTOR_ERROR;
    }

    if ((pCamera->PvCameraInfo.PermittedAccess & ePvAccessMaster) == 0) {
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:PSConnect: Cannot get control of camera %s\n", 
               driverName, pCamera->ipAddr);
        return AREA_DETECTOR_ERROR;
    }

    status = PvCameraOpen(pCamera->PvCameraInfo.UniqueId, ePvAccessMaster, &pCamera->PvHandle);
    if (status) {
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:PSConnect: unable to open camera %s\n",
              driverName, pCamera->ipAddr);
       return AREA_DETECTOR_ERROR;
    }
    
    /* Initialize the frame buffers and queue them */
    status = PvCaptureStart(pCamera->PvHandle);
    if (status) {
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:PSConnect: unable to start capture on camera %s\n",
              driverName, pCamera->ipAddr);
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
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:PSConnect: unable to get sensor data on camera\n",
              driverName, pCamera->ipAddr);
        return AREA_DETECTOR_ERROR;
    }
    
    bytesPerPixel = (pCamera->sensorBits-1)/8 + 1;
    /* If the camera supports color then there can be 4 values per pixel? */
    if (strcmp(pCamera->sensorType, "Mono") != 0) bytesPerPixel *= 4;
    maxBytes = pCamera->sensorWidth * pCamera->sensorHeight * bytesPerPixel;    
    for (i=0; i<MAX_FRAMES; i++) {
        pFrame = &pCamera->frame[i];
        free(pFrame->ImageBuffer);
        pFrame->ImageBuffer = malloc(maxBytes);
        free(pFrame->AncillaryBuffer);
        /* We are not using AncillaryBuffer, but maybe ImageWriteTiff needs it? */
        pFrame->AncillaryBuffer = malloc(1000);
        pFrame->AncillaryBufferSize = 1000;
        if (!pFrame->ImageBuffer) {
            PRINT(pCamera->logParam, ADTraceError, 
                  "%s:PSConnect: unable to allocate ImageBuffer frame %d on camera %s\n",
                  driverName, i, pCamera->ipAddr);
            return AREA_DETECTOR_ERROR;
        }
        pFrame->ImageBufferSize = maxBytes;
        pFrame->Context[0] = (void *)pCamera;
        status = PvCaptureQueueFrame(pCamera->PvHandle, pFrame, PSFrameCallback); 
        if (status) {
            PRINT(pCamera->logParam, ADTraceError, 
                  "%s:PSConnect: unable to queue frame %d on camera %s\n",
                  driverName, i, pCamera->ipAddr);
            return AREA_DETECTOR_ERROR;
        }
    }

    /* Set some initial values for other parameters */
    status =  ADParam->setString (pCamera->params, ADManufacturer, "Prosilica");
    status |= ADParam->setString (pCamera->params, ADModel, pCamera->PvCameraInfo.DisplayName);
    status |= ADParam->setInteger(pCamera->params, ADSizeX, pCamera->sensorWidth);
    status |= ADParam->setInteger(pCamera->params, ADSizeY, pCamera->sensorHeight);
    status |= ADParam->setInteger(pCamera->params, ADMaxSizeX, pCamera->sensorWidth);
    status |= ADParam->setInteger(pCamera->params, ADMaxSizeY, pCamera->sensorHeight);
    if (status) {
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:PSConnect: unable to set camera parameters on camera %s\n",
              driverName, pCamera->ipAddr);
        return AREA_DETECTOR_ERROR;
    }
    
     /* Read the current camera settings */
    status = PSReadParameters(pCamera);
    if (status) return(status);

    /* Read the current camera statistics */
    status = PSReadStats(pCamera);
    if (status) return(status);
        
    /* We found the camera and everything is OK.  Set the flag. */
    status |= ADParam->setInteger(pCamera->params, ADConnect, 1);
    return(status);
}

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

static DETECTOR_HDL ADOpen(int card, char * param)
{
    DETECTOR_HDL pCamera;

    if (card >= numCameras) return(NULL);
    pCamera = &allCameras[card];
    return pCamera;
}

static int ADClose(DETECTOR_HDL pCamera)
{
    int status = AREA_DETECTOR_OK;
    
    PRINT(pCamera->logParam, ADTraceFlow, 
          "%s:ADClose: closing camera %s\n", 
          driverName, pCamera->ipAddr);
    status = PSDisconnect(pCamera);
    return(status);
}

/* Note: ADSetLog, ADFindParam, ADSetInt32Callback, ADSetFloat64Callback, 
 * and ADSetImageDataCallback can usually be used with no modifications in new drivers. */
 
static int ADSetLog( DETECTOR_HDL pCamera, ADLogFunc logFunc, void * param )
{
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    if (logFunc == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    pCamera->logFunc=logFunc;
    pCamera->logParam = param;
    epicsMutexUnlock(pCamera->mutexId);
    return AREA_DETECTOR_OK;
}

static int ADFindParam( DETECTOR_HDL pCamera, const char *paramString, int *function )
{
    int i;
    int status = AREA_DETECTOR_ERROR;
    int ncommands = sizeof(DetCommands)/sizeof(DetCommands[0]);

    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    for (i=0; i < ncommands; i++) {
        if (epicsStrCaseCmp(paramString, DetCommands[i].commandString) == 0) {
            *function = DetCommands[i].command;
            status = AREA_DETECTOR_OK;
            break;
        }
    }
    if (status) 
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ADFindParam: not a valid string=%s\n", 
              driverName, paramString);
    else        
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ADFindParam: found value string=%s, function=%d\n",
              driverName, paramString, *function);
    return status;
}

static int ADSetInt32Callback(DETECTOR_HDL pCamera, ADInt32CallbackFunc callback, void * param)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    status = ADParam->setIntCallback(pCamera->params, callback, param);
    epicsMutexUnlock(pCamera->mutexId);
    return(status);
}

static int ADSetFloat64Callback(DETECTOR_HDL pCamera, ADFloat64CallbackFunc callback, void * param)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    status = ADParam->setDoubleCallback(pCamera->params, callback, param);
    epicsMutexUnlock(pCamera->mutexId);
    return(status);
}

static int ADSetStringCallback(DETECTOR_HDL pCamera, ADStringCallbackFunc callback, void * param)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    status = ADParam->setStringCallback(pCamera->params, callback, param);
    epicsMutexUnlock(pCamera->mutexId);
    return(status);
}

static int ADSetImageDataCallback(DETECTOR_HDL pCamera, ADImageDataCallbackFunc callback, void * param)
{
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    pCamera->pImageDataCallback = callback;
    pCamera->imageDataCallbackParam = param;
    epicsMutexUnlock(pCamera->mutexId);
    return AREA_DETECTOR_OK;
}

static int ADGetInteger(DETECTOR_HDL pCamera, int function, int * value)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    
    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getInteger(pCamera->params, function, value);
    if (status) 
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:ADGetInteger error, status=%d function=%d, value=%d\n", 
              driverName, status, function, *value);
    else        
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ADGetInteger: function=%d, value=%d\n", 
              driverName, function, *value);
    epicsMutexUnlock(pCamera->mutexId);
    return(status);
}

static int ADSetInteger(DETECTOR_HDL pCamera, int function, int value)
{
    int status = AREA_DETECTOR_OK;
    tPvUint32 intVal = value;

    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);

    /* Set the parameter in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= ADParam->setInteger(pCamera->params, function, value);

    switch (function) {
    case ADConnect:
        if (value) status |= PSConnect(pCamera);
        else       status |= PSDisconnect(pCamera);
        break;
    case ADBinX:
    case ADBinY:
    case ADMinX:
    case ADSizeX:
    case ADMinY:
    case ADSizeY:
        /* These commands change the chip readout geometry.  We need to cache them and apply them in the
         * correct order */
        status |= PSSetGeometry(pCamera);
        break;
    case ADNumFrames:
        status |= PvAttrUint32Set(pCamera->PvHandle, "AcquisitionFrameCount", intVal);
        break;
    case ADFrameMode:
        switch(value) {
        case ADFrameSingle:
            status |= PvAttrEnumSet(pCamera->PvHandle, "AcquisitionMode", "SingleFrame");
            break;
        case ADFrameMultiple:
            status |= PvAttrEnumSet(pCamera->PvHandle, "AcquisitionMode", "MultiFrame");
            break;
        case ADFrameContinuous:
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
            case ADFrameSingle:
                pCamera->framesRemaining = 1;
                break;
            case ADFrameMultiple:
                pCamera->framesRemaining = numFrames;
                break;
            case ADFrameContinuous:
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
    case ADTriggerMode:
        if ((value < 0) || (value > (NUM_START_TRIGGER_MODES-1))) {
            status = AREA_DETECTOR_ERROR;
            break;
        }
        status |= PvAttrEnumSet(pCamera->PvHandle, "FrameStartTriggerMode", 
                                PSTriggerStartStrings[value]);
        break;
    case PSReadStatistics:
        PSReadStats(pCamera);
        break;
    case ADWriteFile:
        status = PSWriteFile(pCamera);
        break;
    case ADDataType:
        switch (value) {
            case ADInt8:
            case ADUInt8:
                status |= PvAttrEnumSet(pCamera->PvHandle, "PixelFormat", "Mono8");
                break;
            case ADInt16:
            case ADUInt16:
                status |= PvAttrEnumSet(pCamera->PvHandle, "PixelFormat", "Mono16");
                break;
            /* We don't support other formats yet */
            default:
                PRINT(pCamera->logParam, ADTraceError, 
                    "%s:ADSetInteger error unsupported data type %d\n", 
                    driverName, value);
                status |= AREA_DETECTOR_ERROR;
                break;
        }      
    }
    
    /* Read the camera parameters and do callbacks */
    status |= PSReadParameters(pCamera);
    
    if (status) 
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:ADSetInteger error, status=%d function=%d, value=%d\n", 
              driverName, status, function, value);
    else        
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ADSetInteger: function=%d, value=%d\n", 
              driverName, function, value);
    epicsMutexUnlock(pCamera->mutexId);
    return status;
}


static int ADGetDouble(DETECTOR_HDL pCamera, int function, double * value)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getDouble(pCamera->params, function, value);
    if (status) 
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:ADGetDouble error, status=%d function=%d, value=%f\n", 
              driverName, status, function, *value);
    else        
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ADGetDouble: function=%d, value=%f\n", 
              driverName, function, *value);
    epicsMutexUnlock(pCamera->mutexId);
    return(status);
}

static int ADSetDouble(DETECTOR_HDL pCamera, int function, double value)
{
    int status = AREA_DETECTOR_OK;
    tPvUint32 intVal;
    tPvFloat32 fltVal;

    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);

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
    status |= PSReadParameters(pCamera);
    
    if (status) 
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:ADSetDouble error, status=%d function=%d, value=%f\n", 
              driverName, status, function, value);
    else        
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ADSetDouble: function=%d, value=%f\n", 
              driverName, function, value);
    epicsMutexUnlock(pCamera->mutexId);
    return status;
}

static int ADGetString(DETECTOR_HDL pCamera, int function, int maxChars, char * value)
{
    int status = AREA_DETECTOR_OK;
   
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getString(pCamera->params, function, maxChars, value);
    if (status) 
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:ADGetString error, status=%d function=%d, value=%s\n", 
              driverName, status, function, value);
    else        
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ADGetString: function=%d, value=%s\n", 
              driverName, function, value);
    epicsMutexUnlock(pCamera->mutexId);
    return(status);
}

static int ADSetString(DETECTOR_HDL pCamera, int function, const char *value)
{
    int status = AREA_DETECTOR_OK;

    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    /* Set the parameter in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= ADParam->setString(pCamera->params, function, (char *)value);
    /* Do callbacks so higher layers see any changes */
    ADParam->callCallbacks(pCamera->params);
    if (status) 
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:ADSetString error, status=%d function=%d, value=%s\n", 
              driverName, status, function, value);
    else        
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ASGetString: function=%d, value=%s\n", 
              driverName, function, value);
    epicsMutexUnlock(pCamera->mutexId);
    return status;
}

static int ADGetImage(DETECTOR_HDL pCamera, int maxBytes, void *buffer)
{
    tPvFrame *pFrame; 
    int nCopy;
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);
    if (pCamera->lastFrame == NULL) {
       status = AREA_DETECTOR_ERROR;
    } else {
        pFrame = pCamera->lastFrame;    
        nCopy = pFrame->ImageSize;
        if (nCopy > maxBytes) nCopy = maxBytes;
        memcpy(buffer, pFrame->ImageBuffer, nCopy);
    }
    
    if (status) 
        PRINT(pCamera->logParam, ADTraceError, 
              "%s:ADGetImage error, status=%d maxBytes=%d, buffer=%p\n", 
              driverName, status, maxBytes, buffer);
    else        
        PRINT(pCamera->logParam, ADTraceIODriver, 
              "%s:ADGetImage error, maxBytes=%d, buffer=%p\n", 
              driverName, maxBytes, buffer);
    epicsMutexUnlock(pCamera->mutexId);
    return status;
}

static int ADSetImage(DETECTOR_HDL pCamera, int maxBytes, void *buffer)
{
    int status = AREA_DETECTOR_OK;
    
    if (pCamera == NULL) return AREA_DETECTOR_ERROR;
    epicsMutexLock(pCamera->mutexId);

    /* The Prosilica does not allow downloading image data */    
    PRINT(pCamera->logParam, ADTraceIODriver, 
          "%s:ADSetImage not currently supported\n", driverName);
    status = AREA_DETECTOR_ERROR;
    epicsMutexUnlock(pCamera->mutexId);
    return status;
}


static int PSLogMsg(void * param, const ADLogMask_t mask, const char *pFormat, ...)
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
                    char *ipAddr)   /* IP address of this camera. */

{
    DETECTOR_HDL pCamera;
    int status = AREA_DETECTOR_OK;

    if (numCameras < 1) {
        printf("%s:PSConnect: no Prosilica cameras allocated, call prosilicaSetup first\n");
        return AREA_DETECTOR_ERROR;
    }
    if ((camera < 0) || (camera >= numCameras)) {
        printf("%s:PSConnect: camera must in range 0 to %d\n", driverName, numCameras-1);
        return AREA_DETECTOR_ERROR;
    }
    pCamera = &allCameras[camera];
    pCamera->camera = camera;
    
    /* Create the epicsMutex for locking access to data structures from other threads */
    pCamera->mutexId = epicsMutexCreate();
    if (!pCamera->mutexId) {
        printf("%s:PSConnect: epicsMutexCreate failure\n", driverName);
        return AREA_DETECTOR_ERROR;
    }
    
    /* Set the local log function, may be changed by higher layers */
    ADSetLog(pCamera, PSLogMsg, NULL);

    /* Initialize the Prosilica PvAPI library */
    status = PvInitialize();
    if (status) {
        printf("%s:PSConnect: PvInitialize failed for camera %d\n", driverName, camera);
        return AREA_DETECTOR_ERROR;
    }
    
    /* Initialize the parameter library */
    pCamera->params = ADParam->create(0, ADLastDriverParam);
    if (!pCamera->params) {
        printf("%s:PSConnect: unable to create parameter library\n", driverName);
        return AREA_DETECTOR_ERROR;
    }
    
    /* Use the utility library to set some defaults */
    status = ADUtils->setParamDefaults(pCamera->params);
    
    pCamera->ipAddr = epicsStrDup(ipAddr);
    
    /* Try to connect to the camera.  
     * It is not a fatal error if we cannot now, the camera may be off or owned by
     * someone else.  It may connect later. */
    status = PSConnect(pCamera);
    if (status) {
        printf("%s:PSConnect: cannot connect to camera %d, manually connect when available.\n", 
               driverName, camera);
        return AREA_DETECTOR_ERROR;
    }
    
    return AREA_DETECTOR_OK;
}

