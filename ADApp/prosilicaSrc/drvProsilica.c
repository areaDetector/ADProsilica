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
#include <cantProceed.h>

#include <asynStandardInterfaces.h>

#include "PvAPI.h"
#include "ImageLib.h"

/* Defining this will create the static table of standard parameters in ADInterface.h */
#define DEFINE_STANDARD_PARAM_STRINGS 1
#include "ADParamLib.h"
#include "ADUtils.h"
#include "ADInterface.h"
#include "NDArrayBuff.h"

#include "drvProsilica.h"

static char *driverName = "drvProsilica";

static int PvApiInitialized;

#define MAX_FRAMES  2  /* Number of frame buffers for PvApi */
#define MAX_PACKET_SIZE 8228

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
    PSBadFrameCounter,
    ADLastDriverParam
} PSDetParam_t;

static ADParamString_t PSDetParamString[] = {
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
    {PSStatPacketsResent,     "PS_PACKETS_RESENT"},
    {PSBadFrameCounter,       "PS_BAD_FRAME_COUNTER"}
};

#define NUM_PS_DET_PARAMS (sizeof(PSDetParamString)/sizeof(PSDetParamString[0]))

typedef struct drvADPvt {
    /* The first set of items in this structure will be needed by all drivers */
    char *portName;
    epicsMutexId mutexId;              /* A mutex to lock access to data structures. */
    PARAMS params;

    /* The asyn interfaces this driver implements */
    asynStandardInterfaces asynStdInterfaces;

    /* asynUser connected to ourselves for asynTrace */
    asynUser *pasynUser;

    /* These items are specific to the Prosilica driver */
    tPvHandle PvHandle;                /* Handle for the Prosilica PvAPI library */
    int uniqueId;
    tPvCameraInfo PvCameraInfo;
    tPvFrame PvFrames[MAX_FRAMES];
    size_t maxFrameSize;
    NDArray_t *pImage;
    int framesRemaining;
    char sensorType[20];
    char IPAddress[50];
    int sensorBits;
    int sensorWidth;
    int sensorHeight;
    int timeStampFrequency;
} drvADPvt;


static int PSWriteFile(drvADPvt *pPvt)
{
    /* Writes last image to disk as a TIFF file. */
    int status = asynSuccess, tiffStatus;
    char fullFileName[MAX_FILENAME_LEN];
    int fileFormat;
    tPvFrame PvFrame, *pFrame=&PvFrame;
    NDArrayInfo_t arrayInfo;

    if (!pPvt->pImage) return asynError;
    
    /* Set all fields in frame to 0 */
    memset(pFrame, 0, sizeof(tPvFrame));

    status |= ADUtils->createFileName(pPvt->params, MAX_FILENAME_LEN, fullFileName);
    if (status) { 
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSWriteFile error creating full file name, fullFileName=%s, status=%d\n", 
              driverName, fullFileName, status);
        return(status);
    }
    
    /* Copy the data from our last image buffer to a frame buffer structure, which is
     * required by ImageWriteTiff */
    pFrame->Width = pPvt->pImage->dims[0].size;
    pFrame->Height = pPvt->pImage->dims[1].size;
    pFrame->ImageBuffer = pPvt->pImage->pData;
    NDArrayBuff->getInfo(pPvt->pImage, &arrayInfo);
    pFrame->ImageBufferSize = arrayInfo.totalBytes;
    pFrame->ImageSize = pFrame->ImageBufferSize;
    
    /* Note, this needs work because we need to support color models */
    switch(pPvt->pImage->dataType) {
        case NDInt8:
        case NDUInt8:
            pFrame->Format = ePvFmtMono8;
            pFrame->BitDepth = 8;
            break;
        case NDInt16:
        case NDUInt16:
            pFrame->Format = ePvFmtMono16;
            pFrame->BitDepth = 16;
    }
    
    status |= ADParam->getInteger(pPvt->params, ADFileFormat, &fileFormat);
    /* We only support writing in TIFF format for now */
    tiffStatus = ImageWriteTiff(fullFileName, pFrame);
    if (tiffStatus != 1) {
        status |= asynError;
    } else {
        status |= ADParam->setString(pPvt->params, ADFullFileName, fullFileName);
    }
    return(status);
}

static void PVDECL PSFrameCallback(tPvFrame *pFrame)
{
    drvADPvt *pPvt = (drvADPvt *) pFrame->Context[0];
    int status = asynSuccess;
    NDDataType_t dataType;
    int autoSave;
    int ndims, dims[2];
    int imageCounter;
    NDArray_t *pImage;
    int badFrameCounter;
    const char *functionName = "PSFrameCallback";

    /* If this callback is coming from a shutdown operation rather than normal collection, 
     * we will not be able to take the mutex and things will hang.  Prevent this by looking at the frame
     * status and returning immediately if it is and in that case the mutex has already been taken.  Just return in
     * that case */
    if (pFrame->Status == ePvErrCancelled) return;

    epicsMutexLock(pPvt->mutexId);

    pImage = (NDArray_t *)pFrame->Context[1];

    if (pFrame->Status == ePvErrSuccess) {
        /* The frame we just received has NDArray_t* in Context[1] */ 
        /* We save the most recent good image buffer so it can be used in the PSWriteFile
         * and readADImage functions.  Now release it. */
        if (pPvt->pImage) NDArrayBuff->release(pPvt->pImage);
        pPvt->pImage = pImage;
        /* Set the properties of the image to those of the current frame */
        pImage->dims[0].size = pFrame->Width;
        pImage->dims[1].size = pFrame->Height;
        /* Convert from the PvApi data types to ADDataType */
        switch(pFrame->Format) {
            case ePvFmtMono8:
            case ePvFmtBayer8:
                dataType = NDUInt8;
                break;
            case ePvFmtMono16:
            case ePvFmtBayer16:
                dataType = NDUInt16;
                break;
            default:
                /* Note, this is wrong it does not work for ePvFmtRgb48, which is 48 bits */
                dataType = NDUInt32;
                break;
        }
        pImage->dataType = dataType;
        
        /* Set the uniqueId and time stamp */
        pImage->uniqueId = pFrame->FrameCount;
        pImage->timeStamp = ((double)pFrame->TimestampLo + 
                             (double)pFrame->TimestampHi*4294967296.)/pPvt->timeStampFrequency;
        
        /* Call the NDArray callback */
        /* Must release the lock here, or we can get into a deadlock, because we can
         * block on the plugin lock, and the plugin can be calling us */
        epicsMutexUnlock(pPvt->mutexId);
        ADUtils->handleCallback(pPvt->asynStdInterfaces.handleInterruptPvt, pImage);
        epicsMutexLock(pPvt->mutexId);

        /* See if acquisition is done */
        if (pPvt->framesRemaining > 0) pPvt->framesRemaining--;
        if (pPvt->framesRemaining == 0) {
            ADParam->setInteger(pPvt->params, ADAcquire, 0);
            ADParam->setInteger(pPvt->params, ADStatus, ADStatusIdle);
        }

        /* Update the frame counter */
        ADParam->getInteger(pPvt->params, ADImageCounter, &imageCounter);
        imageCounter++;
        ADParam->setInteger(pPvt->params, ADImageCounter, imageCounter);

        /* If autoSave is set then save the image */
        status = ADParam->getInteger(pPvt->params, ADAutoSave, &autoSave);
        if (autoSave) status = PSWriteFile(pPvt);

        asynPrintIO(pPvt->pasynUser, ASYN_TRACEIO_DRIVER, 
            pPvt->pImage->pData, pPvt->pImage->dataSize,
            "%s:%s: frameId=%d, timeStamp=%f\n",
            driverName, functionName, pImage->uniqueId, pImage->timeStamp);

        /* Allocate a new image buffer, make the size be the maximum that the frames can be */
        ndims = 2;
        dims[0] = pPvt->sensorWidth;
        dims[1] = pPvt->sensorHeight;
        pImage = NDArrayBuff->alloc(ndims, dims, NDInt8, pPvt->maxFrameSize, NULL);
        /* Put the pointer to this image buffer in the frame context[1] */
        pFrame->Context[1] = pImage;
        /* Reset the frame buffer data pointer be this image buffer data pointer */
        pFrame->ImageBuffer = pImage->pData;
    } else {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_FLOW, 
            "%s:%s: ERROR, frame has error code %d\n",
            driverName, functionName, pFrame->Status);
        ADParam->getInteger(pPvt->params, PSBadFrameCounter, &badFrameCounter);
        badFrameCounter++;
        ADParam->setInteger(pPvt->params, PSBadFrameCounter, badFrameCounter);
    }

    /* Update any changed parameters */
    ADParam->callCallbacks(pPvt->params);
    
    /* Queue this frame to run again */
    status = PvCaptureQueueFrame(pPvt->PvHandle, pFrame, PSFrameCallback); 
    epicsMutexUnlock(pPvt->mutexId);
}

static int PSSetGeometry(drvADPvt *pPvt)
{
    int status = asynSuccess;
    tPvUint32 binX, binY, minY, minX, sizeX, sizeY;
    
    /* Get all of the current geometry parameters from the parameter library */
    status |= ADParam->getInteger(pPvt->params, ADBinX, &binX);
    status |= ADParam->getInteger(pPvt->params, ADBinY, &binY);
    status |= ADParam->getInteger(pPvt->params, ADMinX, &minX);
    status |= ADParam->getInteger(pPvt->params, ADMinY, &minY);
    status |= ADParam->getInteger(pPvt->params, ADSizeX, &sizeX);
    status |= ADParam->getInteger(pPvt->params, ADSizeY, &sizeY);
    
    status |= PvAttrUint32Set(pPvt->PvHandle, "BinningX", binX);
    status |= PvAttrUint32Set(pPvt->PvHandle, "BinningY", binY);
    status |= PvAttrUint32Set(pPvt->PvHandle, "RegionX", minX/binX);
    status |= PvAttrUint32Set(pPvt->PvHandle, "RegionY", minY/binY);
    status |= PvAttrUint32Set(pPvt->PvHandle, "Width",   sizeX/binX);
    status |= PvAttrUint32Set(pPvt->PvHandle, "Height",  sizeY/binY);
    
    if (status) asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
                      "%s:PSSetGeometry error, status=%d\n", 
                      driverName, status);
    return(status);
}

static int PSGetGeometry(drvADPvt *pPvt)
{
    int status = asynSuccess;
    tPvUint32 binX, binY, minY, minX, sizeX, sizeY;

    status |= PvAttrUint32Get(pPvt->PvHandle, "BinningX", &binX);
    status |= PvAttrUint32Get(pPvt->PvHandle, "BinningY", &binY);
    status |= PvAttrUint32Get(pPvt->PvHandle, "RegionX",  &minX);
    status |= PvAttrUint32Get(pPvt->PvHandle, "RegionY",  &minY);
    status |= PvAttrUint32Get(pPvt->PvHandle, "Width",    &sizeX);
    status |= PvAttrUint32Get(pPvt->PvHandle, "Height",   &sizeY);
    
    status |= ADParam->setInteger(pPvt->params, ADBinX,  binX);
    status |= ADParam->setInteger(pPvt->params, ADBinY,  binY);
    status |= ADParam->setInteger(pPvt->params, ADMinX,  minX*binX);
    status |= ADParam->setInteger(pPvt->params, ADMinY,  minY*binY);
    status |= ADParam->setInteger(pPvt->params, ADImageSizeX, sizeX);
    status |= ADParam->setInteger(pPvt->params, ADImageSizeY, sizeY);
    
    if (status) asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
                      "%s:PSGetGeometry error, status=%d\n", 
                      driverName, status);
    return(status);
}

static PSReadStats(drvADPvt *pPvt)
{
    int status = asynSuccess;
    char buffer[50];
    int nchars;
    epicsUInt32 uval;
    float fval;
    
    status |= PvAttrEnumGet      (pPvt->PvHandle, "StatDriverType", buffer, sizeof(buffer), &nchars);
    status |= ADParam->setString (pPvt->params,  PSStatDriverType, buffer);
    status |= PvAttrStringGet    (pPvt->PvHandle, "StatFilterVersion", buffer, sizeof(buffer), &nchars);
    status |= ADParam->setString (pPvt->params,  PSStatFilterVersion, buffer);
    status |= PvAttrFloat32Get   (pPvt->PvHandle, "StatFrameRate", &fval);
    status |= ADParam->setDouble (pPvt->params,  PSStatFrameRate, fval);
    status |= PvAttrUint32Get    (pPvt->PvHandle, "StatFramesCompleted", &uval);
    status |= ADParam->setInteger(pPvt->params,  PSStatFramesCompleted, (int)uval);
    status |= PvAttrUint32Get    (pPvt->PvHandle, "StatFramesDropped", &uval);
    status |= ADParam->setInteger(pPvt->params,  PSStatFramesDropped, (int)uval);
    status |= PvAttrUint32Get    (pPvt->PvHandle, "StatPacketsErroneous", &uval);
    status |= ADParam->setInteger(pPvt->params,  PSStatPacketsErroneous, (int)uval);
    status |= PvAttrUint32Get    (pPvt->PvHandle, "StatPacketsMissed", &uval);
    status |= ADParam->setInteger(pPvt->params,  PSStatPacketsMissed, (int)uval);
    status |= PvAttrUint32Get    (pPvt->PvHandle, "StatPacketsReceived", &uval);
    status |= ADParam->setInteger(pPvt->params,  PSStatPacketsReceived, (int)uval);
    status |= PvAttrUint32Get    (pPvt->PvHandle, "StatPacketsRequested", &uval);
    status |= ADParam->setInteger(pPvt->params,  PSStatPacketsRequested, (int)uval);
    status |= PvAttrUint32Get    (pPvt->PvHandle, "StatPacketsResent", &uval);
    status |= ADParam->setInteger(pPvt->params,  PSStatPacketsResent, (int)uval);
    if (status) asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
                      "%s:PSReadStatistics error, status=%d\n", 
                      driverName, status);
    return(status);
}

static PSReadParameters(drvADPvt *pPvt)
{
    int status = asynSuccess;
    tPvUint32 intVal;
    tPvFloat32 fltVal;
    double dval;
    int nchars;
    char buffer[20];

    status |= PvAttrUint32Get(pPvt->PvHandle, "TotalBytesPerFrame", &intVal);
    ADParam->setInteger(pPvt->params, ADImageSize, intVal);

    intVal = -1;
    status |= PvAttrEnumGet(pPvt->PvHandle, "PixelFormat", buffer, sizeof(buffer), &nchars);
    if (!strcmp(buffer, "Mono8")) intVal = NDUInt8;
    else if (!strcmp(buffer, "Mono16")) intVal = NDUInt16;
    /* We don't support color modes yet */
    status |= ADParam->setInteger(pPvt->params, ADDataType, intVal);
    
    status |= PSGetGeometry(pPvt);

    status |= PvAttrUint32Get(pPvt->PvHandle, "AcquisitionFrameCount", &intVal);
    status |= ADParam->setInteger(pPvt->params, ADNumImages, intVal);

    status |= PvAttrEnumGet(pPvt->PvHandle, "AcquisitionMode", buffer, sizeof(buffer), &nchars);
    if      (!strcmp(buffer, "SingleFrame")) intVal = ADImageSingle;
    else if (!strcmp(buffer, "MultiFrame"))  intVal = ADImageMultiple;
    else if (!strcmp(buffer, "Recorder"))    intVal = ADImageMultiple;
    else if (!strcmp(buffer, "Continuous"))  intVal = ADImageContinuous;
    else {intVal=0; status |= asynError;}
    status |= ADParam->setInteger(pPvt->params, ADImageMode, intVal);

    status |= PvAttrEnumGet(pPvt->PvHandle, "FrameStartTriggerMode", buffer, sizeof(buffer), &nchars);
    for (intVal=0; intVal<NUM_START_TRIGGER_MODES; intVal++) {
        if (strcmp(buffer, PSTriggerStartStrings[intVal]) == 0) {
            status |= ADParam->setInteger(pPvt->params, ADTriggerMode, intVal);
            break;
        }
    }
    if (intVal == NUM_START_TRIGGER_MODES) {
        status |= ADParam->setInteger(pPvt->params, ADTriggerMode, 0);
        status |= asynError;
    }
    
    /* Prosilica does not support more than 1 exposure per frame */
    status |= ADParam->setInteger(pPvt->params, ADNumExposures, 1);

    /* Prosilica uses integer microseconds */
    status |= PvAttrUint32Get(pPvt->PvHandle, "ExposureValue", &intVal);
    dval = intVal / 1.e6;
    status |= ADParam->setDouble(pPvt->params, ADAcquireTime, dval);

    /* Prosilica uses a frame rate in Hz */
    status |= PvAttrFloat32Get(pPvt->PvHandle, "FrameRate", &fltVal);
    dval = 1. / fltVal;
    status |= ADParam->setDouble(pPvt->params, ADAcquirePeriod, dval);

    /* Prosilica uses an integer value */
    status |= PvAttrUint32Get(pPvt->PvHandle, "GainValue", &intVal);
    dval = intVal;
    status |= ADParam->setDouble(pPvt->params, ADGain, dval);

    /* Call the callbacks to update the values in higher layers */
    ADParam->callCallbacks(pPvt->params);
    
    if (status) asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
                      "%s:PSReadParameters error, status=%d\n", 
                      driverName, status);
    return(status);
}

static int PSDisconnect(drvADPvt *pPvt)
{
    int status = asynSuccess;
    tPvFrame *pFrame;
    NDArray_t *pImage;

    if (!pPvt->PvHandle) return(asynSuccess);
    status |= PvCaptureQueueClear(pPvt->PvHandle);
    status |= PvCaptureEnd(pPvt->PvHandle);
    status |= PvCameraClose(pPvt->PvHandle);
    asynPrint(pPvt->pasynUser, ASYN_TRACE_FLOW, 
          "%s:PSDisconnect: disconnecting camera %d\n", 
          driverName, pPvt->uniqueId);
    if (status) {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSDisonnect: unable to close camera %d\n",
              driverName, pPvt->uniqueId);
    }
    /* If we have allocated frame buffers, free them. */
    /* Must first free any image buffers they point to */
    pFrame = pPvt->PvFrames;
    while (pFrame) {
        pImage = pFrame->Context[1];
        if (pImage) NDArrayBuff->release(pImage);
        pFrame->Context[1] = 0;
        pFrame++;
    }
    pPvt->PvHandle = NULL;
    return(status);
}

static int PSConnect(drvADPvt *pPvt)
{
    int status = asynSuccess;
    int nchars;
    tPvFrame *pFrame;
    int i;
    int ndims, dims[2];
    int bytesPerPixel;
    NDArray_t *pImage;

    /* First disconnect from the camera */
    PSDisconnect(pPvt);
    
    status = PvCameraInfo(pPvt->uniqueId, &pPvt->PvCameraInfo);
    if (status) {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSConnect: Cannot find camera %d\n", 
              driverName, pPvt->uniqueId);
        return asynError;
    }

    if ((pPvt->PvCameraInfo.PermittedAccess & ePvAccessMaster) == 0) {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSConnect: Cannot get control of camera %d\n", 
               driverName, pPvt->uniqueId);
        return asynError;
    }

    status = PvCameraOpen(pPvt->uniqueId, ePvAccessMaster, &pPvt->PvHandle);
    if (status) {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSConnect: unable to open camera %d\n",
              driverName, pPvt->uniqueId);
       return asynError;
    }
    
    /* Negotiate maximum frame size */
    status = PvCaptureAdjustPacketSize(pPvt->PvHandle, MAX_PACKET_SIZE);
    if (status) {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSConnect: unable to adjust packet size %d\n",
              driverName, pPvt->uniqueId);
       return asynError;
    }
    
    /* Initialize the frame buffers and queue them */
    status = PvCaptureStart(pPvt->PvHandle);
    if (status) {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSConnect: unable to start capture on camera %d\n",
              driverName, pPvt->uniqueId);
        return asynError;
    }

    /* We allocate image buffers that are large enough for the biggest possible image.
       This is simpler than reallocating when readout parameters change.  It is also safer,
       since changing readout parameters happens instantly, but there will still be frames
       queued with the wrong size */
    /* Query the parameters of the image sensor */
    status = PvAttrEnumGet(pPvt->PvHandle, "SensorType", pPvt->sensorType, 
                             sizeof(pPvt->sensorType), &nchars);
    status |= PvAttrUint32Get(pPvt->PvHandle, "SensorBits", &pPvt->sensorBits);
    status |= PvAttrUint32Get(pPvt->PvHandle, "SensorWidth", &pPvt->sensorWidth);
    status |= PvAttrUint32Get(pPvt->PvHandle, "SensorHeight", &pPvt->sensorHeight);
    status |= PvAttrUint32Get(pPvt->PvHandle, "TimeStampFrequency", &pPvt->timeStampFrequency);
    status |= PvAttrStringGet(pPvt->PvHandle, "DeviceIPAddress", pPvt->IPAddress, 
                              sizeof(pPvt->IPAddress), &nchars);
    if (status) {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSConnect: unable to get sensor data on camera %d\n",
              driverName, pPvt->uniqueId);
        return asynError;
    }
    
    bytesPerPixel = (pPvt->sensorBits-1)/8 + 1;
    /* If the camera supports color then there can be 4 values per pixel? */
    if (strcmp(pPvt->sensorType, "Mono") != 0) bytesPerPixel *= 4;
    pPvt->maxFrameSize = pPvt->sensorWidth * pPvt->sensorHeight * bytesPerPixel;    
    for (i=0; i<MAX_FRAMES; i++) {
        pFrame = &pPvt->PvFrames[i];
        ndims = 2;
        dims[0] = pPvt->sensorWidth;
        dims[1] = pPvt->sensorHeight;
       /* Allocate a new image buffer, make the size be the maximum that the frames can be */
        pImage = NDArrayBuff->alloc(ndims, dims, NDInt8, pPvt->maxFrameSize, NULL);
        if (!pImage) {
            asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
                  "%s:PSConnect: unable to allocate image %d on camera %d\n",
                  driverName, i, pPvt->uniqueId);
            return asynError;
        }
        /* Set the frame buffer data pointer be this image buffer data pointer */
        pFrame->ImageBuffer = pImage->pData;
        pFrame->ImageBufferSize = pPvt->maxFrameSize;
        /* Put a pointer to ourselves in Context[0] */
        pFrame->Context[0] = (void *)pPvt;
        /* Put the pointer to this image buffer in the frame context[1] */
        pFrame->Context[1] = pImage;
        status = PvCaptureQueueFrame(pPvt->PvHandle, pFrame, PSFrameCallback); 
        if (status) {
            asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
                  "%s:PSConnect: unable to queue frame %d on camera %d\n",
                  driverName, i, pPvt->uniqueId);
            return asynError;
        }
    }

    /* Set some initial values for other parameters */
    status =  ADParam->setString (pPvt->params, ADManufacturer, "Prosilica");
    status |= ADParam->setString (pPvt->params, ADModel, pPvt->PvCameraInfo.DisplayName);
    status |= ADParam->setInteger(pPvt->params, ADSizeX, pPvt->sensorWidth);
    status |= ADParam->setInteger(pPvt->params, ADSizeY, pPvt->sensorHeight);
    status |= ADParam->setInteger(pPvt->params, ADMaxSizeX, pPvt->sensorWidth);
    status |= ADParam->setInteger(pPvt->params, ADMaxSizeY, pPvt->sensorHeight);
    status |= ADParam->setInteger(pPvt->params, PSBadFrameCounter, 0);
    if (status) {
        asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
              "%s:PSConnect: unable to set camera parameters on camera %d\n",
              driverName, pPvt->uniqueId);
        return asynError;
    }
    
     /* Read the current camera settings */
    status = PSReadParameters(pPvt);
    if (status) return(status);

    /* Read the current camera statistics */
    status = PSReadStats(pPvt);
    if (status) return(status);
        
    /* We found the camera and everything is OK.  Signal to asynManager that we are connected. */
    pasynManager->exceptionConnect(pPvt->pasynUser);
    return(status);
}


/* asynInt32 interface functions */
static asynStatus readInt32(void *drvPvt, asynUser *pasynUser, 
                            epicsInt32 *value)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    epicsMutexLock(pPvt->mutexId);
    
    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getInteger(pPvt->params, function, value);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                  "%s:readInt32 error, status=%d function=%d, value=%d\n", 
                  driverName, status, function, *value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:readInt32: function=%d, value=%d\n", 
              driverName, function, *value);
    epicsMutexUnlock(pPvt->mutexId);
    return(status);
}

static asynStatus writeInt32(void *drvPvt, asynUser *pasynUser, 
                             epicsInt32 value)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int reset=0;
    const char *functionName = "writeInt32";

    epicsMutexLock(pPvt->mutexId);

    /* Set the parameter in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= ADParam->setInteger(pPvt->params, function, value);

    switch (function) {
        case ADBinX:
        case ADBinY:
        case ADMinX:
        case ADSizeX:
        case ADMinY:
        case ADSizeY:
            /* These commands change the chip readout geometry.  We need to cache them and apply them in the
             * correct order */
            status |= PSSetGeometry(pPvt);
            break;
        case ADNumImages:
            status |= PvAttrUint32Set(pPvt->PvHandle, "AcquisitionFrameCount", value);
            break;
        case ADImageMode:
            switch(value) {
            case ADImageSingle:
                status |= PvAttrEnumSet(pPvt->PvHandle, "AcquisitionMode", "SingleFrame");
                break;
            case ADImageMultiple:
                status |= PvAttrEnumSet(pPvt->PvHandle, "AcquisitionMode", "MultiFrame");
                break;
            case ADImageContinuous:
                status |= PvAttrEnumSet(pPvt->PvHandle, "AcquisitionMode", "Continuous");
                break;
            }
            break;
        case ADAcquire:
            if (value) {
                /* We need to set the number of images we expect to collect, so the frame callback function
                   can know when acquisition is complete.  We need to find out what mode we are in and how
                   many frames have been requested.  If we are in continuous mode then set the number of
                   remaining frames to -1. */
                int imageMode, numImages;
                status |= ADParam->getInteger(pPvt->params, ADImageMode, &imageMode);
                status |= ADParam->getInteger(pPvt->params, ADNumImages, &numImages);
                switch(imageMode) {
                case ADImageSingle:
                    pPvt->framesRemaining = 1;
                    break;
                case ADImageMultiple:
                    pPvt->framesRemaining = numImages;
                    break;
                case ADImageContinuous:
                    pPvt->framesRemaining = -1;
                    break;
                }
                ADParam->setInteger(pPvt->params, ADStatus, ADStatusAcquire);
                status |= PvCommandRun(pPvt->PvHandle, "AcquisitionStart");
            } else {
                ADParam->setInteger(pPvt->params, ADStatus, ADStatusIdle);
                status |= PvCommandRun(pPvt->PvHandle, "AcquisitionAbort");
            }
            break;
        case ADTriggerMode:
            if ((value < 0) || (value > (NUM_START_TRIGGER_MODES-1))) {
                status = asynError;
                break;
            }
            status |= PvAttrEnumSet(pPvt->PvHandle, "FrameStartTriggerMode", 
                                    PSTriggerStartStrings[value]);
            break;
        case PSReadStatistics:
            PSReadStats(pPvt);
            break;
        case ADWriteFile:
            status = PSWriteFile(pPvt);
            break;
        case ADDataType:
            switch (value) {
                case NDInt8:
                case NDUInt8:
                    status |= PvAttrEnumSet(pPvt->PvHandle, "PixelFormat", "Mono8");
                    break;
                case NDInt16:
                case NDUInt16:
                    status |= PvAttrEnumSet(pPvt->PvHandle, "PixelFormat", "Mono16");
                    break;
                /* We don't support other formats yet */
                default:
                    asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
                        "%s:%s: error unsupported data type %d\n", 
                        driverName, functionName, value);
                    status |= asynError;
                    break;
            }      
    }
    
    /* Read the camera parameters and do callbacks */
    status |= PSReadParameters(pPvt);    
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    epicsMutexUnlock(pPvt->mutexId);
    return status;
}

static asynStatus getBounds(void *drvPvt, asynUser *pasynUser,
                            epicsInt32 *low, epicsInt32 *high)
{
    /* This is only needed for the asynInt32 interface when the device uses raw units.
       Our interface is using engineering units. */
    *low = 0;
    *high = 65535;
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s::getBounds,low=%d, high=%d\n", driverName, *low, *high);
    return(asynSuccess);
}


/* asynFloat64 interface methods */
static asynStatus readFloat64(void *drvPvt, asynUser *pasynUser,
                              epicsFloat64 *value)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    
    epicsMutexLock(pPvt->mutexId);
    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getDouble(pPvt->params, function, value);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:readFloat64 error, status=%d function=%d, value=%f\n", 
              driverName, status, function, *value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:readFloat64: function=%d, value=%f\n", 
              driverName, function, *value);
    epicsMutexUnlock(pPvt->mutexId);
    return(status);
}

static asynStatus writeFloat64(void *drvPvt, asynUser *pasynUser, 
                               epicsFloat64 value)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    tPvUint32 intVal;
    tPvFloat32 fltVal;

    epicsMutexLock(pPvt->mutexId);

    /* Set the parameter in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= ADParam->setDouble(pPvt->params, function, value);

    switch (function) {
    case ADAcquireTime:
        /* Prosilica uses integer microseconds */
        intVal = (tPvUint32) (value * 1e6);
        status |= PvAttrUint32Set(pPvt->PvHandle, "ExposureValue", intVal);
        break;
    case ADAcquirePeriod:
        /* Prosilica uses a frame rate in Hz */
        if (value == 0.) value = .01;
        fltVal = (tPvFloat32) (1. / value);
        status |= PvAttrFloat32Set(pPvt->PvHandle, "FrameRate", fltVal);
        break;
    case ADGain:
        /* Prosilica uses an integer value */
        intVal = (tPvUint32) (value);
        status |= PvAttrUint32Set(pPvt->PvHandle, "GainValue", intVal);
        break;
    default:
        break;
    }

    /* Read the camera parameters and do callbacks */
    status |= PSReadParameters(pPvt);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:writeFloat64 error, status=%d function=%d, value=%f\n", 
              driverName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:writeFloat64: function=%d, value=%f\n", 
              driverName, function, value);
    epicsMutexUnlock(pPvt->mutexId);
    return status;
}


/* asynOctet interface methods */
static asynStatus readOctet(void *drvPvt, asynUser *pasynUser,
                            char *value, size_t maxChars, size_t *nActual,
                            int *eomReason)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
   
    epicsMutexLock(pPvt->mutexId);
    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = ADParam->getString(pPvt->params, function, maxChars, value);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:readOctet error, status=%d function=%d, value=%s\n", 
              driverName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:readOctet: function=%d, value=%s\n", 
              driverName, function, value);
    *eomReason = ASYN_EOM_END;
    *nActual = strlen(value);
    epicsMutexUnlock(pPvt->mutexId);
    return(status);
}

static asynStatus writeOctet(void *drvPvt, asynUser *pasynUser,
                             const char *value, size_t nChars, size_t *nActual)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    epicsMutexLock(pPvt->mutexId);
    /* Set the parameter in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= ADParam->setString(pPvt->params, function, (char *)value);
    /* Do callbacks so higher layers see any changes */
    ADParam->callCallbacks(pPvt->params);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:writeOctet error, status=%d function=%d, value=%s\n", 
              driverName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:writeOctet: function=%d, value=%s\n", 
              driverName, function, value);
    *nActual = nChars;
    epicsMutexUnlock(pPvt->mutexId);
    return status;
}

/* asynHandle interface methods */
static asynStatus readADImage(void *drvPvt, asynUser *pasynUser, void *handle)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    NDArray_t *pImage = handle;
    NDArrayInfo_t arrayInfo;
    int dataSize=0;
    int status = asynSuccess;
    const char* functionName = "readADImage";
    
    epicsMutexLock(pPvt->mutexId);
    if (!pPvt->pImage) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, no valid image available\n", 
              driverName, functionName);
        status = asynError;
    } else {
        pImage->ndims = pPvt->pImage->ndims;
        memcpy(pImage->dims, pPvt->pImage->dims, sizeof(pImage->dims));
        pImage->dataType = pPvt->pImage->dataType;
        NDArrayBuff->getInfo(pPvt->pImage, &arrayInfo);
        dataSize = arrayInfo.totalBytes;
        if (dataSize > pImage->dataSize) dataSize = pImage->dataSize;
        memcpy(pImage->pData, pPvt->pImage->pData, dataSize);
    }
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s error, status=%d pData=%p\n", 
              driverName, functionName, status, pImage->pData);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s error, maxBytes=%d, data=%p\n", 
              driverName, functionName, dataSize, pImage->pData);
    epicsMutexUnlock(pPvt->mutexId);
    return status;
}


static asynStatus writeADImage(void *drvPvt, asynUser *pasynUser, NDArray_t *pImage)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    int status = asynSuccess;
    
    if (pPvt == NULL) return asynError;
    epicsMutexLock(pPvt->mutexId);

    /* The Prosilica does not allow downloading image data */    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
          "%s:ADSetImage not currently supported\n", driverName);
    status = asynError;
    epicsMutexUnlock(pPvt->mutexId);
    return status;
}

/* asynDrvUser routines */
static asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser,
                                const char *drvInfo, 
                                const char **pptypeName, size_t *psize)
{
    int status;
    int param;

    /* See if this is one of the standard parameters */
    status = ADUtils->findParam(ADStandardParamString, NUM_AD_STANDARD_PARAMS, 
                                drvInfo, &param);
                                
    /* If we did not find it in that table try our driver-specific table */
    if (status) status = ADUtils->findParam(PSDetParamString, NUM_PS_DET_PARAMS, 
                                            drvInfo, &param);
    
    if (status == asynSuccess) {
        pasynUser->reason = param;
        if (pptypeName) {
            *pptypeName = epicsStrDup(drvInfo);
        }
        if (psize) {
            *psize = sizeof(param);
        }
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "%s::drvUserCreate, drvInfo=%s, param=%d\n", 
                  driverName, drvInfo, param);
        return(asynSuccess);
    } else {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                     "%s::drvUserCreate, unknown drvInfo=%s", 
                     driverName, drvInfo);
        return(asynError);
    }
}
    
static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser,
                                 const char **pptypeName, size_t *psize)
{
    /* This is not currently supported, because we can't get the strings for driver-specific commands */

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "%s::drvUserGetType entered",
              driverName);
    *pptypeName = NULL;
    *psize = 0;
    return(asynError);
}

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
    /* Nothing to do because we did not allocate any resources */
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "%s::drvUserDestroy, drvPvt=%p, pasynUser=%p\n",
              driverName, drvPvt, pasynUser);
    return(asynSuccess);
}


/* asynCommon interface methods */

static asynStatus connect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionConnect(pasynUser);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
          "%s::connect, pasynUser=%p\n", 
          driverName, pasynUser);
    return(asynSuccess);
}


static asynStatus disconnect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionDisconnect(pasynUser);
    return(asynSuccess);
}

static void report(void *drvPvt, FILE *fp, int details)
{
    drvADPvt *pPvt = (drvADPvt *)drvPvt;
    tPvCameraInfo cameraInfo[20]; 
    int i;
    unsigned long numReturned, numTotal;
    
    numReturned = PvCameraList(cameraInfo, 20, &numTotal);

    fprintf(fp, "Prosilica camera %s Unique ID=%d\n", 
            pPvt->portName, pPvt->uniqueId);
    if (details > 0) {
        fprintf(fp, "  ID:                %ul\n", pPvt->PvCameraInfo.UniqueId);
        fprintf(fp, "  IP address:        %s\n",  pPvt->IPAddress);
        fprintf(fp, "  Serial number:     %s\n",  pPvt->PvCameraInfo.SerialString);
        fprintf(fp, "  Model:             %s\n",  pPvt->PvCameraInfo.DisplayName);
        fprintf(fp, "  Sensor type:       %s\n",  pPvt->sensorType);
        fprintf(fp, "  Sensor bits:       %d\n",  pPvt->sensorBits);
        fprintf(fp, "  Sensor width:      %d\n",  pPvt->sensorWidth);
        fprintf(fp, "  Sensor height:     %d\n",  pPvt->sensorHeight);
        fprintf(fp, "  Frame buffer size: %d\n",  pPvt->PvFrames[0].ImageBufferSize);
        fprintf(fp, "  Time stamp freq:   %d\n",  pPvt->timeStampFrequency);
        fprintf(fp, "\n");
        fprintf(fp, "List of all Prosilica cameras found, (total=%d):\n", numReturned);
        for (i=0; i<(int)numReturned; i++) {
            fprintf(fp, "    ID: %d\n", cameraInfo[i].UniqueId);
        }
    }
    if (details > 5) {
        fprintf(fp, "\nParameter library contents:\n");
        ADParam->dump(pPvt->params);
        NDArrayBuff->report(details);
    }
}


/* Structures with function pointers for each of the asyn interfaces */
static asynCommon ifaceCommon = {
    report,
    connect,
    disconnect
};

static asynInt32 ifaceInt32 = {
    writeInt32,
    readInt32,
    getBounds
};

static asynFloat64 ifaceFloat64 = {
    writeFloat64,
    readFloat64
};

static asynOctet ifaceOctet = {
    writeOctet,
    NULL,
    readOctet,
};

static asynDrvUser ifaceDrvUser = {
    drvUserCreate,
    drvUserGetType,
    drvUserDestroy
};

static asynHandle ifaceHandle = {
    writeADImage,
    readADImage
};



int prosilicaConfig(char *portName, /* Port name */
                    int uniqueId)   /* Unique ID # of this camera. */

{
    drvADPvt *pPvt;
    int status = asynSuccess;
    char *functionName = "prosilicaConfig";
    asynStandardInterfaces *pInterfaces;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), functionName);
    pPvt->portName = epicsStrDup(portName);
    pPvt->uniqueId = uniqueId;
 
    status = pasynManager->registerPort(portName,
                                        ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                                        1,  /*  autoconnect */
                                        0,  /* medium priority */
                                        0); /* default stack size */
    if (status != asynSuccess) {
        printf("%s ERROR: Can't register port\n", functionName);
        return(asynError);
    }

    /* Create asynUser for debugging */
    pPvt->pasynUser = pasynManager->createAsynUser(0, 0);

    pInterfaces = &pPvt->asynStdInterfaces;
    
    /* Initialize interface pointers */
    pInterfaces->common.pinterface        = (void *)&ifaceCommon;
    pInterfaces->drvUser.pinterface       = (void *)&ifaceDrvUser;
    pInterfaces->octet.pinterface         = (void *)&ifaceOctet;
    pInterfaces->int32.pinterface         = (void *)&ifaceInt32;
    pInterfaces->float64.pinterface       = (void *)&ifaceFloat64;
    pInterfaces->handle.pinterface        = (void *)&ifaceHandle;

    /* Define which interfaces can generate interrupts */
    pInterfaces->octetCanInterrupt        = 1;
    pInterfaces->int32CanInterrupt        = 1;
    pInterfaces->float64CanInterrupt      = 1;
    pInterfaces->handleCanInterrupt       = 1;

    status = pasynStandardInterfacesBase->initialize(portName, pInterfaces,
                                                     pPvt->pasynUser, pPvt);
    if (status != asynSuccess) {
        printf("%s ERROR: Can't register interfaces: %s.\n",
               functionName, pPvt->pasynUser->errorMessage);
        return(asynError);
    }
    
    /* Connect to our device for asynTrace */
    status = pasynManager->connectDevice(pPvt->pasynUser, portName, 0);
    if (status != asynSuccess) {
        printf("%s, connectDevice failed\n", functionName);
        return -1;
    }

     /* Create the epicsMutex for locking access to data structures from other threads */
    pPvt->mutexId = epicsMutexCreate();
    if (!pPvt->mutexId) {
        printf("%s: epicsMutexCreate failure\n", functionName);
        return asynError;
    }
    
   /* Initialize the Prosilica PvAPI library 
    * We get an error if we call this twice, so we need a global flag to see if 
    * it's already been done.*/
    if (!PvApiInitialized) {
        status = PvInitialize();
        if (status) {
            printf("%s:PSConnect: PvInitialize failed for camera %d, status=%d\n", 
            driverName, uniqueId, status);
            return asynError;
        }
        PvApiInitialized = 1;
    }
    
    /* It appears to be necessary to wait a little for the PvAPI library to find the cameras */
    epicsThreadSleep(0.2);
    
    /* Initialize the parameter library */
    pPvt->params = ADParam->create(0, ADLastDriverParam, &pPvt->asynStdInterfaces);
    if (!pPvt->params) {
        printf("%s: unable to create parameter library\n", functionName);
        return asynError;
    }
    
    /* Use the utility library to set some defaults */
    status = ADUtils->setParamDefaults(pPvt->params);

    /* Try to connect to the camera.  
     * It is not a fatal error if we cannot now, the camera may be off or owned by
     * someone else.  It may connect later. */
    status = PSConnect(pPvt);
    if (status) {
        printf("%s:PSConnect: cannot connect to camera %d, manually connect when available.\n", 
               driverName, uniqueId);
        return asynError;
    }
    
    return asynSuccess;
}

