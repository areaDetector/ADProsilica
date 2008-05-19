/* prosilica.cpp
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

#include "PvAPI.h"
#include "ImageLib.h"

#include "ADStdDriverParams.h"
#include "NDArray.h"
#include "ADDriverBase.h"

#include "drvProsilica.h"

static char *driverName = "prosilica";

static int PvApiInitialized;

#define MAX_FRAMES  2  /* Number of frame buffers for PvApi */
#define MAX_PACKET_SIZE 8228

class prosilica : public ADDriverBase {
public:
    prosilica(const char *portName, int uniqueId, int maxBuffers, size_t maxMemory);
                 
    /* These are the methods that we override from ADDriverBase */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, 
                                     const char **pptypeName, size_t *psize);
    void report(FILE *fp, int details);
                                        
    /* These are the methods that are new to this class */
    asynStatus writeFile();
    void frameCallback(tPvFrame *pFrame);
    asynStatus setGeometry();
    asynStatus getGeometry();
    asynStatus readStats();
    asynStatus readParameters();
    asynStatus disconnectCamera();
    asynStatus connectCamera();
    
    /* These items are specific to the Prosilica driver */
    tPvHandle PvHandle;                /* Handle for the Prosilica PvAPI library */
    unsigned long uniqueId;
    tPvCameraInfo PvCameraInfo;
    tPvFrame PvFrames[MAX_FRAMES];
    size_t maxFrameSize;
    int framesRemaining;
    char sensorType[20];
    char IPAddress[50];
    tPvUint32 sensorBits;
    tPvUint32 sensorWidth;
    tPvUint32 sensorHeight;
    tPvUint32 timeStampFrequency;
};

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
    PSReadStatistics 
        = ADFirstDriverParam,
    PSReadStatistics_RBV,
    PSStatDriverType_RBV,
    PSStatFilterVersion_RBV,
    PSStatFrameRate_RBV,
    PSStatFramesCompleted_RBV,
    PSStatFramesDropped_RBV,
    PSStatPacketsErroneous_RBV,
    PSStatPacketsMissed_RBV,
    PSStatPacketsReceived_RBV,
    PSStatPacketsRequested_RBV,
    PSStatPacketsResent_RBV,
    PSBadFrameCounter_RBV,
    ADLastDriverParam
} PSDetParam_t;

static asynParamString_t PSDetParamString[] = {
    {PSReadStatistics,            "PS_READ_STATISTICS"},
    {PSReadStatistics_RBV,        "PS_READ_STATISTICS_RBV"},
    {PSStatDriverType_RBV,        "PS_DRIVER_TYPE_RBV"},
    {PSStatFilterVersion_RBV,     "PS_FILTER_VERSION_RBV"},
    {PSStatFrameRate_RBV,         "PS_FRAME_RATE_RBV"},
    {PSStatFramesCompleted_RBV,   "PS_FRAMES_COMPLETED_RBV"},
    {PSStatFramesDropped_RBV,     "PS_FRAMES_DROPPED_RBV"},
    {PSStatPacketsErroneous_RBV,  "PS_PACKETS_ERRONEOUS_RBV"},
    {PSStatPacketsMissed_RBV,     "PS_PACKETS_MISSED_RBV"},
    {PSStatPacketsReceived_RBV,   "PS_PACKETS_RECEIVED_RBV"},
    {PSStatPacketsRequested_RBV,  "PS_PACKETS_REQUESTED_RBV"},
    {PSStatPacketsResent_RBV,     "PS_PACKETS_RESENT_RBV"},
    {PSBadFrameCounter_RBV,       "PS_BAD_FRAME_COUNTER_RBV"}
};

#define NUM_PS_DET_PARAMS (sizeof(PSDetParamString)/sizeof(PSDetParamString[0]))

asynStatus prosilica::writeFile()
{
    /* Writes last image to disk as a TIFF file. */
    int status = asynSuccess, tiffStatus;
    char fullFileName[MAX_FILENAME_LEN];
    int fileFormat;
    int addr=0;
    NDArray *pImage = this->pArrays[addr];
    tPvFrame PvFrame, *pFrame=&PvFrame;
    NDArrayInfo_t arrayInfo;
    const char *functionName = "writeFile";

    if (!pImage) return asynError;
    
    /* Set all fields in frame to 0 */
    memset(pFrame, 0, sizeof(tPvFrame));

    status |= createFileName(MAX_FILENAME_LEN, fullFileName);
    if (status) { 
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error creating full file name, fullFileName=%s, status=%d\n", 
              driverName, functionName, fullFileName, status);
        return((asynStatus)status);
    }
    
    /* Copy the data from our last image buffer to a frame buffer structure, which is
     * required by ImageWriteTiff */
    pFrame->Width = pImage->dims[0].size;
    pFrame->Height = pImage->dims[1].size;
    pFrame->ImageBuffer = pImage->pData;
    pImage->getInfo(&arrayInfo);
    pFrame->ImageBufferSize = arrayInfo.totalBytes;
    pFrame->ImageSize = pFrame->ImageBufferSize;
    
    /* Note, this needs work because we need to support color models */
    switch(pImage->dataType) {
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
    
    status |= getIntegerParam(addr, ADFileFormat, &fileFormat);
    /* We only support writing in TIFF format for now */
    tiffStatus = ImageWriteTiff(fullFileName, pFrame);
    if (tiffStatus != 1) {
        status |= asynError;
    } else {
        status |= setStringParam(addr, ADFullFileName_RBV, fullFileName);
    }
    return((asynStatus)status);
}

static void PVDECL frameCallbackC(tPvFrame *pFrame)
{
    prosilica *pPvt = (prosilica *) pFrame->Context[0];
    
    pPvt->frameCallback(pFrame);
}

void prosilica::frameCallback(tPvFrame *pFrame)
{
    int status = asynSuccess;
    NDDataType_t dataType;
    int autoSave;
    int ndims, dims[2];
    int imageCounter;
    int addr=0;
    NDArray *pImage;
    int badFrameCounter;
    const char *functionName = "frameCallback";

    /* If this callback is coming from a shutdown operation rather than normal collection, 
     * we will not be able to take the mutex and things will hang.  Prevent this by looking at the frame
     * status and returning immediately if it is and in that case the mutex has already been taken.  Just return in
     * that case */
    if (pFrame->Status == ePvErrCancelled) return;

    epicsMutexLock(this->mutexId);

    pImage = (NDArray *)pFrame->Context[1];

    if (pFrame->Status == ePvErrSuccess) {
        /* The frame we just received has NDArray* in Context[1] */ 
        /* We save the most recent good image buffer so it can be used in the PSWriteFile
         * and readADImage functions.  Now release it. */
        if (this->pArrays[addr]) this->pArrays[addr]->release();
        this->pArrays[addr] = pImage;
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
        if (this->timeStampFrequency == 0) this->timeStampFrequency = 1;
        pImage->timeStamp = ((double)pFrame->TimestampLo + 
                             (double)pFrame->TimestampHi*4294967296.)/this->timeStampFrequency;
        
        /* Call the NDArray callback */
        /* Must release the lock here, or we can get into a deadlock, because we can
         * block on the plugin lock, and the plugin can be calling us */
        epicsMutexUnlock(this->mutexId);
        doCallbacksHandle(pImage, NDArrayData, 0);
        epicsMutexLock(this->mutexId);

        /* See if acquisition is done */
        if (this->framesRemaining > 0) this->framesRemaining--;
        if (this->framesRemaining == 0) {
            setIntegerParam(addr, ADAcquire, 0);
            setIntegerParam(addr, ADAcquire_RBV, 0);
            setIntegerParam(addr, ADStatus_RBV, ADStatusIdle);
        }

        /* Update the frame counter */
        getIntegerParam(addr, ADImageCounter, &imageCounter);
        imageCounter++;
        setIntegerParam(addr, ADImageCounter, imageCounter);
        setIntegerParam(addr, ADImageCounter_RBV, imageCounter);

        /* If autoSave is set then save the image */
        status = getIntegerParam(addr, ADAutoSave, &autoSave);
        if (autoSave) status = writeFile();

        asynPrintIO(this->pasynUser, ASYN_TRACEIO_DRIVER, 
            (const char *)this->pArrays[addr]->pData, this->pArrays[addr]->dataSize,
            "%s:%s: frameId=%d, timeStamp=%f\n",
            driverName, functionName, pImage->uniqueId, pImage->timeStamp);

        /* Allocate a new image buffer, make the size be the maximum that the frames can be */
        ndims = 2;
        dims[0] = this->sensorWidth;
        dims[1] = this->sensorHeight;
        pImage = this->pNDArrayPool->alloc(ndims, dims, NDInt8, this->maxFrameSize, NULL);
        /* Put the pointer to this image buffer in the frame context[1] */
        pFrame->Context[1] = pImage;
        /* Reset the frame buffer data pointer be this image buffer data pointer */
        pFrame->ImageBuffer = pImage->pData;
    } else {
        asynPrint(this->pasynUser, ASYN_TRACE_FLOW, 
            "%s:%s: ERROR, frame has error code %d\n",
            driverName, functionName, pFrame->Status);
        getIntegerParam(addr, PSBadFrameCounter_RBV, &badFrameCounter);
        badFrameCounter++;
        setIntegerParam(addr, PSBadFrameCounter_RBV, badFrameCounter);
    }

    /* Update any changed parameters */
    callParamCallbacks(addr, addr);
    
    /* Queue this frame to run again */
    status = PvCaptureQueueFrame(this->PvHandle, pFrame, frameCallbackC); 
    epicsMutexUnlock(this->mutexId);
}

asynStatus prosilica::setGeometry()
{
    int status = asynSuccess;
    int addr=0;
    int binX, binY, minY, minX, sizeX, sizeY;
    static char *functionName = "setGeometry";
    
    /* Get all of the current geometry parameters from the parameter library */
    status |= getIntegerParam(addr, ADBinX, &binX);
    if (binX < 1) binX = 1;
    status |= getIntegerParam(addr, ADBinY, &binY);
    if (binY < 1) binY = 1;
    status |= getIntegerParam(addr, ADMinX, &minX);
    status |= getIntegerParam(addr, ADMinY, &minY);
    status |= getIntegerParam(addr, ADSizeX, &sizeX);
    status |= getIntegerParam(addr, ADSizeY, &sizeY);
    
    status |= PvAttrUint32Set(this->PvHandle, "BinningX", binX);
    status |= PvAttrUint32Set(this->PvHandle, "BinningY", binY);
    status |= PvAttrUint32Set(this->PvHandle, "RegionX", minX/binX);
    status |= PvAttrUint32Set(this->PvHandle, "RegionY", minY/binY);
    status |= PvAttrUint32Set(this->PvHandle, "Width",   sizeX/binX);
    status |= PvAttrUint32Set(this->PvHandle, "Height",  sizeY/binY);
    
    if (status) asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
                      "%s:%s: error, status=%d\n", 
                      driverName, functionName, status);
    return((asynStatus)status);
}

asynStatus prosilica::getGeometry()
{
    int status = asynSuccess;
    int addr=0;
    tPvUint32 binX, binY, minY, minX, sizeX, sizeY;
    static char *functionName = "setGeometry";

    status |= PvAttrUint32Get(this->PvHandle, "BinningX", &binX);
    status |= PvAttrUint32Get(this->PvHandle, "BinningY", &binY);
    status |= PvAttrUint32Get(this->PvHandle, "RegionX",  &minX);
    status |= PvAttrUint32Get(this->PvHandle, "RegionY",  &minY);
    status |= PvAttrUint32Get(this->PvHandle, "Width",    &sizeX);
    status |= PvAttrUint32Get(this->PvHandle, "Height",   &sizeY);
    
    status |= setIntegerParam(addr, ADBinX,  binX);
    status |= setIntegerParam(addr, ADBinY,  binY);
    status |= setIntegerParam(addr, ADMinX,  minX*binX);
    status |= setIntegerParam(addr, ADMinY,  minY*binY);
    status |= setIntegerParam(addr, ADSizeX, sizeX*binX);
    status |= setIntegerParam(addr, ADSizeY, sizeY*binY);

    status |= setIntegerParam(addr, ADBinX_RBV,  binX);
    status |= setIntegerParam(addr, ADBinY_RBV,  binY);
    status |= setIntegerParam(addr, ADMinX_RBV,  minX*binX);
    status |= setIntegerParam(addr, ADMinY_RBV,  minY*binY);
    status |= setIntegerParam(addr, ADSizeX_RBV, sizeX*binX);
    status |= setIntegerParam(addr, ADSizeY_RBV, sizeY*binY);
    status |= setIntegerParam(addr, ADImageSizeX_RBV, sizeX);
    status |= setIntegerParam(addr, ADImageSizeY_RBV, sizeY);
    
    if (status) asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
                      "%s:%s: error, status=%d\n", 
                      driverName, functionName, status);
    return((asynStatus)status);
}

asynStatus prosilica::readStats()
{
    int status = asynSuccess;
    char buffer[50];
    int addr=0;
    unsigned long nchars;
    tPvUint32 uval;
    float fval;
    static char *functionName = "readStats";
    
    status |= PvAttrEnumGet      (this->PvHandle, "StatDriverType", buffer, sizeof(buffer), &nchars);
    status |= setStringParam (addr,  PSStatDriverType_RBV, buffer);
    status |= PvAttrStringGet    (this->PvHandle, "StatFilterVersion", buffer, sizeof(buffer), &nchars);
    status |= setStringParam (addr,  PSStatFilterVersion_RBV, buffer);
    status |= PvAttrFloat32Get   (this->PvHandle, "StatFrameRate", &fval);
    status |= setDoubleParam (addr,  PSStatFrameRate_RBV, fval);
    status |= PvAttrUint32Get    (this->PvHandle, "StatFramesCompleted", &uval);
    status |= setIntegerParam(addr,  PSStatFramesCompleted_RBV, (int)uval);
    status |= PvAttrUint32Get    (this->PvHandle, "StatFramesDropped", &uval);
    status |= setIntegerParam(addr,  PSStatFramesDropped_RBV, (int)uval);
    status |= PvAttrUint32Get    (this->PvHandle, "StatPacketsErroneous", &uval);
    status |= setIntegerParam(addr,  PSStatPacketsErroneous_RBV, (int)uval);
    status |= PvAttrUint32Get    (this->PvHandle, "StatPacketsMissed", &uval);
    status |= setIntegerParam(addr,  PSStatPacketsMissed_RBV, (int)uval);
    status |= PvAttrUint32Get    (this->PvHandle, "StatPacketsReceived", &uval);
    status |= setIntegerParam(addr,  PSStatPacketsReceived_RBV, (int)uval);
    status |= PvAttrUint32Get    (this->PvHandle, "StatPacketsRequested", &uval);
    status |= setIntegerParam(addr,  PSStatPacketsRequested_RBV, (int)uval);
    status |= PvAttrUint32Get    (this->PvHandle, "StatPacketsResent", &uval);
    status |= setIntegerParam(addr,  PSStatPacketsResent_RBV, (int)uval);
    if (status) asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
                      "%s:%s: error, status=%d\n", 
                      driverName, functionName, status);
    return((asynStatus)status);
}

asynStatus prosilica::readParameters()
{
    int status = asynSuccess;
    tPvUint32 intVal;
    tPvFloat32 fltVal;
    int addr=0;
    double dval;
    unsigned long nchars;
    char buffer[20];
    static char *functionName = "setGeometry";

    status |= PvAttrUint32Get(this->PvHandle, "TotalBytesPerFrame", &intVal);
    setIntegerParam(addr, ADImageSize_RBV, intVal);

    intVal = -1;
    status |= PvAttrEnumGet(this->PvHandle, "PixelFormat", buffer, sizeof(buffer), &nchars);
    if (!strcmp(buffer, "Mono8")) intVal = NDUInt8;
    else if (!strcmp(buffer, "Mono16")) intVal = NDUInt16;
    /* We don't support color modes yet */
    status |= setIntegerParam(addr, ADDataType_RBV, intVal);
    
    status |= getGeometry();

    status |= PvAttrUint32Get(this->PvHandle, "AcquisitionFrameCount", &intVal);
    status |= setIntegerParam(addr, ADNumImages_RBV, intVal);

    status |= PvAttrEnumGet(this->PvHandle, "AcquisitionMode", buffer, sizeof(buffer), &nchars);
    if      (!strcmp(buffer, "SingleFrame")) intVal = ADImageSingle;
    else if (!strcmp(buffer, "MultiFrame"))  intVal = ADImageMultiple;
    else if (!strcmp(buffer, "Recorder"))    intVal = ADImageMultiple;
    else if (!strcmp(buffer, "Continuous"))  intVal = ADImageContinuous;
    else {intVal=0; status |= asynError;}
    status |= setIntegerParam(addr, ADImageMode_RBV, intVal);

    status |= PvAttrEnumGet(this->PvHandle, "FrameStartTriggerMode", buffer, sizeof(buffer), &nchars);
    for (intVal=0; intVal<NUM_START_TRIGGER_MODES; intVal++) {
        if (strcmp(buffer, PSTriggerStartStrings[intVal]) == 0) {
            status |= setIntegerParam(addr, ADTriggerMode_RBV, intVal);
            break;
        }
    }
    if (intVal == NUM_START_TRIGGER_MODES) {
        status |= setIntegerParam(addr, ADTriggerMode_RBV, 0);
        status |= asynError;
    }
    
    /* Prosilica does not support more than 1 exposure per frame */
    status |= setIntegerParam(addr, ADNumExposures_RBV, 1);

    /* Prosilica uses integer microseconds */
    status |= PvAttrUint32Get(this->PvHandle, "ExposureValue", &intVal);
    dval = intVal / 1.e6;
    status |= setDoubleParam(addr, ADAcquireTime_RBV, dval);

    /* Prosilica uses a frame rate in Hz */
    status |= PvAttrFloat32Get(this->PvHandle, "FrameRate", &fltVal);
    if (fltVal == 0.) fltVal = 1;
    dval = 1. / fltVal;
    status |= setDoubleParam(addr, ADAcquirePeriod_RBV, dval);

    /* Prosilica uses an integer value */
    status |= PvAttrUint32Get(this->PvHandle, "GainValue", &intVal);
    dval = intVal;
    status |= setDoubleParam(addr, ADGain_RBV, dval);

    /* Call the callbacks to update the values in higher layers */
    callParamCallbacks(addr, addr);
    
    if (status) asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
                      "%s:%s: error, status=%d\n", 
                      driverName, functionName, status);
    return((asynStatus)status);
}

asynStatus prosilica::disconnectCamera()
{
    int status = asynSuccess;
    tPvFrame *pFrame;
    NDArray *pImage;
    static char *functionName = "disconnectCamera";

    if (!this->PvHandle) return(asynSuccess);
    status |= PvCaptureQueueClear(this->PvHandle);
    status |= PvCaptureEnd(this->PvHandle);
    status |= PvCameraClose(this->PvHandle);
    asynPrint(this->pasynUser, ASYN_TRACE_FLOW, 
          "%s:%s: disconnecting camera %d\n", 
          driverName, functionName, this->uniqueId);
    if (status) {
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: unable to close camera %d\n",
              driverName, functionName, this->uniqueId);
    }
    /* If we have allocated frame buffers, free them. */
    /* Must first free any image buffers they point to */
    pFrame = this->PvFrames;
    while (pFrame) {
        pImage = (NDArray *)pFrame->Context[1];
        if (pImage) pImage->release();
        pFrame->Context[1] = 0;
        pFrame++;
    }
    this->PvHandle = NULL;
    return((asynStatus)status);
}

asynStatus prosilica::connectCamera()
{
    int status = asynSuccess;
    unsigned long nchars;
    tPvFrame *pFrame;
    int i;
    int addr=0;
    int ndims, dims[2];
    int bytesPerPixel;
    NDArray *pImage;
    static char *functionName = "connectCamera";

    /* First disconnect from the camera */
    disconnectCamera();
    
    status = ::PvCameraInfo(this->uniqueId, &this->PvCameraInfo);
    if (status) {
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: Cannot find camera %d\n", 
              driverName, functionName, this->uniqueId);
        return asynError;
    }

    if ((this->PvCameraInfo.PermittedAccess & ePvAccessMaster) == 0) {
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: Cannot get control of camera %d\n", 
               driverName, functionName, this->uniqueId);
        return asynError;
    }

    status = PvCameraOpen(this->uniqueId, ePvAccessMaster, &this->PvHandle);
    if (status) {
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: unable to open camera %d\n",
              driverName, functionName, this->uniqueId);
       return asynError;
    }
    
    /* Negotiate maximum frame size */
    status = PvCaptureAdjustPacketSize(this->PvHandle, MAX_PACKET_SIZE);
    if (status) {
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: unable to adjust packet size %d\n",
              driverName, functionName, this->uniqueId);
       return asynError;
    }
    
    /* Initialize the frame buffers and queue them */
    status = PvCaptureStart(this->PvHandle);
    if (status) {
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: unable to start capture on camera %d\n",
              driverName, functionName, this->uniqueId);
        return asynError;
    }

    /* We allocate image buffers that are large enough for the biggest possible image.
       This is simpler than reallocating when readout parameters change.  It is also safer,
       since changing readout parameters happens instantly, but there will still be frames
       queued with the wrong size */
    /* Query the parameters of the image sensor */
    status = PvAttrEnumGet(this->PvHandle, "SensorType", this->sensorType, 
                             sizeof(this->sensorType), &nchars);
    status |= PvAttrUint32Get(this->PvHandle, "SensorBits", &this->sensorBits);
    status |= PvAttrUint32Get(this->PvHandle, "SensorWidth", &this->sensorWidth);
    status |= PvAttrUint32Get(this->PvHandle, "SensorHeight", &this->sensorHeight);
    status |= PvAttrUint32Get(this->PvHandle, "TimeStampFrequency", &this->timeStampFrequency);
    status |= PvAttrStringGet(this->PvHandle, "DeviceIPAddress", this->IPAddress, 
                              sizeof(this->IPAddress), &nchars);
    if (status) {
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: unable to get sensor data on camera %d\n",
              driverName, functionName, this->uniqueId);
        return asynError;
    }
    
    bytesPerPixel = (this->sensorBits-1)/8 + 1;
    /* If the camera supports color then there can be 3 values per pixel */
    if (strcmp(this->sensorType, "Mono") != 0) bytesPerPixel *= 3;
    this->maxFrameSize = this->sensorWidth * this->sensorHeight * bytesPerPixel;    
    for (i=0; i<MAX_FRAMES; i++) {
        pFrame = &this->PvFrames[i];
        ndims = 2;
        dims[0] = this->sensorWidth;
        dims[1] = this->sensorHeight;
       /* Allocate a new image buffer, make the size be the maximum that the frames can be */
        pImage = this->pNDArrayPool->alloc(ndims, dims, NDInt8, this->maxFrameSize, NULL);
        if (!pImage) {
            asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
                  "%s:%s: unable to allocate image %d on camera %d\n",
                  driverName, functionName, i, this->uniqueId);
            return asynError;
        }
        /* Set the frame buffer data pointer be this image buffer data pointer */
        pFrame->ImageBuffer = pImage->pData;
        pFrame->ImageBufferSize = this->maxFrameSize;
        /* Put a pointer to ourselves in Context[0] */
        pFrame->Context[0] = (void *)this;
        /* Put the pointer to this image buffer in the frame context[1] */
        pFrame->Context[1] = pImage;
        status = PvCaptureQueueFrame(this->PvHandle, pFrame, frameCallbackC); 
        if (status) {
            asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
                  "%s:%s: unable to queue frame %d on camera %d\n",
                  driverName, functionName, i, this->uniqueId);
            return asynError;
        }
    }

    /* Set some initial values for other parameters */
    status =  setStringParam (addr, ADManufacturer_RBV, "Prosilica");
    status |= setStringParam (addr, ADModel_RBV, this->PvCameraInfo.DisplayName);
    status |= setIntegerParam(addr, ADSizeX_RBV, this->sensorWidth);
    status |= setIntegerParam(addr, ADSizeY_RBV, this->sensorHeight);
    status |= setIntegerParam(addr, ADMaxSizeX_RBV, this->sensorWidth);
    status |= setIntegerParam(addr, ADMaxSizeY_RBV, this->sensorHeight);
    status |= setIntegerParam(addr, PSBadFrameCounter_RBV, 0);
    if (status) {
        asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s unable to set camera parameters on camera %d\n",
              driverName, functionName, this->uniqueId);
        return asynError;
    }
    
     /* Read the current camera settings */
    status = readParameters();
    if (status) return((asynStatus)status);

    /* Read the current camera statistics */
    status = readStats();
    if (status) return((asynStatus)status);
        
    /* We found the camera and everything is OK.  Signal to asynManager that we are connected. */
    pasynManager->exceptionConnect(this->pasynUser);
    return((asynStatus)status);
}


asynStatus prosilica::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    int addr=0;
    int reset=0;
    const char *functionName = "writeInt32";

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= setIntegerParam(addr, function, value);
    status |= setIntegerParam(addr, function+1, value);

    switch (function) {
        case ADBinX:
        case ADBinY:
        case ADMinX:
        case ADSizeX:
        case ADMinY:
        case ADSizeY:
            /* These commands change the chip readout geometry.  We need to cache them and apply them in the
             * correct order */
            status |= setGeometry();
            break;
        case ADNumImages:
            status |= PvAttrUint32Set(this->PvHandle, "AcquisitionFrameCount", value);
            break;
        case ADImageMode:
            switch(value) {
            case ADImageSingle:
                status |= PvAttrEnumSet(this->PvHandle, "AcquisitionMode", "SingleFrame");
                break;
            case ADImageMultiple:
                status |= PvAttrEnumSet(this->PvHandle, "AcquisitionMode", "MultiFrame");
                break;
            case ADImageContinuous:
                status |= PvAttrEnumSet(this->PvHandle, "AcquisitionMode", "Continuous");
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
                status |= getIntegerParam(addr, ADImageMode, &imageMode);
                status |= getIntegerParam(addr, ADNumImages, &numImages);
                switch(imageMode) {
                case ADImageSingle:
                    this->framesRemaining = 1;
                    break;
                case ADImageMultiple:
                    this->framesRemaining = numImages;
                    break;
                case ADImageContinuous:
                    this->framesRemaining = -1;
                    break;
                }
                setIntegerParam(addr, ADStatus_RBV, ADStatusAcquire);
                status |= PvCommandRun(this->PvHandle, "AcquisitionStart");
            } else {
                setIntegerParam(addr, ADStatus_RBV, ADStatusIdle);
                status |= PvCommandRun(this->PvHandle, "AcquisitionAbort");
            }
            break;
        case ADTriggerMode:
            if ((value < 0) || (value > (NUM_START_TRIGGER_MODES-1))) {
                status = asynError;
                break;
            }
            status |= PvAttrEnumSet(this->PvHandle, "FrameStartTriggerMode", 
                                    PSTriggerStartStrings[value]);
            break;
        case PSReadStatistics:
            readStats();
            break;
        case ADWriteFile:
            status = writeFile();
            break;
        case ADDataType:
            switch (value) {
                case NDInt8:
                case NDUInt8:
                    status |= PvAttrEnumSet(this->PvHandle, "PixelFormat", "Mono8");
                    break;
                case NDInt16:
                case NDUInt16:
                    status |= PvAttrEnumSet(this->PvHandle, "PixelFormat", "Mono16");
                    break;
                /* We don't support other formats yet */
                default:
                    asynPrint(this->pasynUser, ASYN_TRACE_ERROR, 
                        "%s:%s: error unsupported data type %d\n", 
                        driverName, functionName, value);
                    status |= asynError;
                    break;
            }      
    }
    
    /* Read the camera parameters and do callbacks */
    status |= readParameters();    
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return((asynStatus)status);
}

asynStatus prosilica::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    int addr=0;
    int status = asynSuccess;
    tPvUint32 intVal;
    tPvFloat32 fltVal;

   /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status |= setDoubleParam(addr, function, value);
    status |= setDoubleParam(addr, function+1, value);

    switch (function) {
    case ADAcquireTime:
        /* Prosilica uses integer microseconds */
        intVal = (tPvUint32) (value * 1e6);
        status |= PvAttrUint32Set(this->PvHandle, "ExposureValue", intVal);
        break;
    case ADAcquirePeriod:
        /* Prosilica uses a frame rate in Hz */
        if (value == 0.) value = .01;
        fltVal = (tPvFloat32) (1. / value);
        status |= PvAttrFloat32Set(this->PvHandle, "FrameRate", fltVal);
        break;
    case ADGain:
        /* Prosilica uses an integer value */
        intVal = (tPvUint32) (value);
        status |= PvAttrUint32Set(this->PvHandle, "GainValue", intVal);
        break;
    default:
        break;
    }

    /* Read the camera parameters and do callbacks */
    status |= readParameters();
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:writeFloat64 error, status=%d function=%d, value=%f\n", 
              driverName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:writeFloat64: function=%d, value=%f\n", 
              driverName, function, value);
    return((asynStatus)status);
}


/* asynDrvUser routines */
asynStatus prosilica::drvUserCreate(asynUser *pasynUser,
                                    const char *drvInfo, 
                                    const char **pptypeName, size_t *psize)
{
    asynStatus status;
    int param;
    const char *functionName = "drvUserCreate";

    /* See if this is one of this drivers' parameters */
    status = findParam(PSDetParamString, NUM_PS_DET_PARAMS, 
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
                  "%s:%s, drvInfo=%s, param=%d\n", 
                  driverName, functionName, drvInfo, param);
        return(asynSuccess);
    } 
    /* This was not one of our driver parameters, call the base class method */
    status = ADDriverBase::drvUserCreate(pasynUser, drvInfo, pptypeName, psize); 
    return(status);
}
    
void prosilica::report(FILE *fp, int details)
{
    tPvCameraInfo cameraInfo[20]; 
    int i;
    unsigned long numReturned, numTotal;
    
    numReturned = PvCameraList(cameraInfo, 20, &numTotal);

    fprintf(fp, "Prosilica camera %s Unique ID=%d\n", 
            this->portName, this->uniqueId);
    if (details > 0) {
        fprintf(fp, "  ID:                %ul\n", this->PvCameraInfo.UniqueId);
        fprintf(fp, "  IP address:        %s\n",  this->IPAddress);
        fprintf(fp, "  Serial number:     %s\n",  this->PvCameraInfo.SerialString);
        fprintf(fp, "  Model:             %s\n",  this->PvCameraInfo.DisplayName);
        fprintf(fp, "  Sensor type:       %s\n",  this->sensorType);
        fprintf(fp, "  Sensor bits:       %d\n",  this->sensorBits);
        fprintf(fp, "  Sensor width:      %d\n",  this->sensorWidth);
        fprintf(fp, "  Sensor height:     %d\n",  this->sensorHeight);
        fprintf(fp, "  Frame buffer size: %d\n",  this->PvFrames[0].ImageBufferSize);
        fprintf(fp, "  Time stamp freq:   %d\n",  this->timeStampFrequency);
        fprintf(fp, "\n");
        fprintf(fp, "List of all Prosilica cameras found, (total=%d):\n", numReturned);
        for (i=0; i<(int)numReturned; i++) {
            fprintf(fp, "    ID: %d\n", cameraInfo[i].UniqueId);
        }
    }

    /* Call the base class method */
    ADDriverBase::report(fp, details);
}


extern "C" int prosilicaConfig(char *portName, /* Port name */
                               int uniqueId,   /* Unique ID # of this camera. */
                               int maxBuffers,
                               size_t maxMemory)
{
    new prosilica(portName, uniqueId, maxBuffers, maxMemory);
    return(asynSuccess);
}   


prosilica::prosilica(const char *portName, int uniqueId, int maxBuffers, size_t maxMemory)
    : ADDriverBase(portName, 1, ADLastDriverParam, maxBuffers, maxMemory, 0, 0), 
      uniqueId(uniqueId), PvHandle(NULL), framesRemaining(0)

{
    int status = asynSuccess;
    char *functionName = "prosilica";
    int addr=0;

   /* Initialize the Prosilica PvAPI library 
    * We get an error if we call this twice, so we need a global flag to see if 
    * it's already been done.*/
    if (!PvApiInitialized) {
        status = PvInitialize();
        if (status) {
            printf("%s:%s: ERROR: PvInitialize failed for camera %d, status=%d\n", 
            driverName, functionName, uniqueId, status);
            return;
        }
        PvApiInitialized = 1;
    }
    
    /* It appears to be necessary to wait a little for the PvAPI library to find the cameras */
    epicsThreadSleep(0.2);
    
    /* Try to connect to the camera.  
     * It is not a fatal error if we cannot now, the camera may be off or owned by
     * someone else.  It may connect later. */
    status = connectCamera();
    if (status) {
        printf("%s:%s: cannot connect to camera %d, manually connect when available.\n", 
               driverName, functionName, uniqueId);
        return;
    }
        
}

