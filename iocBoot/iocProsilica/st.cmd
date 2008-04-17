< envPaths
errlogInit(20000)

dbLoadDatabase("$(AD)/dbd/prosilicaApp.dbd")
prosilicaApp_registerRecordDeviceDriver(pdbbase) 

# Initialize the buffer library
NDArrayBuffInit(50, 100000000)

prosilicaConfig("PS1", 50110)
dbLoadRecords("$(AD)/ADApp/Db/ADBase.template","P=13PS1:,D=cam1:,PORT=PS1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AD)/ADApp/Db/prosilica.template","P=13PS1:,D=cam1:,PORT=PS1,ADDR=0,TIMEOUT=1")

prosilicaConfig("PS2", 50022)
dbLoadRecords("$(AD)/ADApp/Db/ADBase.template","P=13PS1:,D=cam2:,PORT=PS2,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AD)/ADApp/Db/prosilica.template","P=13PS1:,D=cam2:,PORT=PS2,ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin, set it to get data from first simDetector driver.
drvNDStdArraysConfigure("PS1Image", 5, 0, "PS1", 0)
dbLoadRecords("$(AD)/ADApp/Db/NDStdArrays.template","P=13PS1:,A=image1:,PORT=PS1Image,ADDR=0,TIMEOUT=1,SIZE=8,FTVL=UCHAR,NELEMENTS=1392640")

# Create a file saving plugin
drvADFileConfigure("PS1File", 10, 0, "PS1", 0)
dbLoadRecords("$(AD)/ADApp/Db/ADFile.template","P=13PS1:,F=file1:,PORT=PS1File,ADDR=0,TIMEOUT=1")

#asynSetTraceMask("PS1",0,255)

set_requestfile_path("./")
set_savefile_path("./autosave")
set_requestfile_path("$(AD)/ADApp/Db")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
save_restoreSet_status_prefix("13PS1:")
dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db", "P=13PS1:")

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=13PS1:,D=cam1:")
