/* drvProsilica.h
 *
 * This is a driver for Prosilica cameras (GigE and CameraLink).
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created:  March 20, 2008
 *
 */
#ifndef DRV_PROSILICA_H
#define DRV_PROSILICA_H

#ifdef __cplusplus
extern "C" {
#endif

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
    {PSStatPacketsResent,     "PS_PACKETS_RESENT"}
};

#define NUM_PS_DET_PARAMS (sizeof(PSDetParamString)/sizeof(PSDetParamString[0]))

int prosilicaConfig(char *portName,       /* Camera number */
                    int uniqueId);        /* Unique ID number of the camera to use */

#ifdef __cplusplus
}
#endif
#endif
