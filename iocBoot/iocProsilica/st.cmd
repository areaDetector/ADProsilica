errlogInit(20000)

dbLoadDatabase("$(AREA_DETECTOR)/dbd/prosilicaApp.dbd")

prosilicaApp_registerRecordDeviceDriver(pdbbase) 

# The second parameter to the prosilicaConfig command is the uniqueId of the camera.
# The simplest way to determine the uniqueId of a camera is to run the Prosilica GigEViewer application, 
# select the camera, and press the "i" icon on the bottom of the main window to show the camera information for this camera. 
# The Unique ID will be displayed on the first line in the information window.
#prosilicaConfig("PS1", 50110, 50, 200000000)
prosilicaConfig("PS1", 51031, 50, -1)
asynSetTraceIOMask("PS1",0,2)
#asynSetTraceMask("PS1",0,255)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/ADBase.template",   "P=13PS1:,R=cam1:,PORT=PS1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",   "P=13PS1:,R=cam1:,PORT=PS1,ADDR=0,TIMEOUT=1")
# Note that prosilica.template must be loaded after NDFile.template to replace the file format correctly
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/prosilica.template","P=13PS1:,R=cam1:,PORT=PS1,ADDR=0,TIMEOUT=1")

#prosilicaConfig("PS2", 50022, 10, 50000000)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/ADBase.template",   "P=13PS1:,R=cam2:,PORT=PS2,ADDR=0,TIMEOUT=1")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/prosilica.template","P=13PS1:,R=cam2:,PORT=PS2,ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin, set it to get data from first Prosilica driver.
NDStdArraysConfigure("Image1", 5, 0, "PS1", 0, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
# Use this line if you only want to use the Prosilica in 8-bit mode.  It uses an 8-bit waveform record
# NELEMENTS is set large enough for a 1360x1024x3 image size, which is the number of pixels in RGB images from the GC1380CH color camera. 
# Must be at least as big as the maximum size of your camera images
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=13PS1:,R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int8,FTVL=UCHAR,NELEMENTS=4177920")
# Use this line if you want to use the Prosilica in 8,12 or 16-bit modes.  
# It uses an 16-bit waveform record, so it uses twice the memory and bandwidth required for only 8-bit data.
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=13PS1:,R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=4177920")
# Load the database to use with Stephen Mudie's IDL code
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/EPICS_AD_Viewer.template", "P=13PS1:, R=image1:")

# Create a netCDF file saving plugin
NDFileNetCDFConfigure("FileNetCDF1", 450, 0, "PS1", 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=netCDF1:,PORT=FileNetCDF1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=13PS1:,R=netCDF1:,PORT=FileNetCDF1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=13PS1:,R=netCDF1:,PORT=FileNetCDF1,ADDR=0,TIMEOUT=1")

# Create a TIFF file saving plugin
NDFileTIFFConfigure("FileTIFF1", 20, 0, "PS1", 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=TIFF1:,PORT=FileTIFF1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=13PS1:,R=TIFF1:,PORT=FileTIFF1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileTIFF.template",  "P=13PS1:,R=TIFF1:,PORT=FileTIFF1,ADDR=0,TIMEOUT=1")

# Create a JPEG file saving plugin
NDFileJPEGConfigure("FileJPEG1", 20, 0, "PS1", 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=JPEG1:,PORT=FileJPEG1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=13PS1:,R=JPEG1:,PORT=FileJPEG1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileJPEG.template",  "P=13PS1:,R=JPEG1:,PORT=FileJPEG1,ADDR=0,TIMEOUT=1")

# Create a NeXus file saving plugin
NDFileNexusConfigure("FileNexus1", 20, 0, "PS1", 0, 0, 80000)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Nexus1:,PORT=FileNexus1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=13PS1:,R=Nexus1:,PORT=FileNexus1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNexus.template", "P=13PS1:,R=Nexus1:,PORT=FileNexus1,ADDR=0,TIMEOUT=1")

# Create 4 ROI plugins
NDROIConfigure("ROI1", 20, 0, "PS1", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=ROI1:,  PORT=ROI1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROI.template",       "P=13PS1:,R=ROI1:,  PORT=ROI1,ADDR=0,TIMEOUT=1")
NDROIConfigure("ROI2", 20, 0, "PS1", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=ROI2:,  PORT=ROI2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROI.template",       "P=13PS1:,R=ROI2:,  PORT=ROI2,ADDR=0,TIMEOUT=1")
NDROIConfigure("ROI3", 20, 0, "PS1", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=ROI3:,  PORT=ROI3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROI.template",       "P=13PS1:,R=ROI3:,  PORT=ROI3,ADDR=0,TIMEOUT=1")
NDROIConfigure("ROI4", 20, 0, "PS1", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=ROI4:,  PORT=ROI4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROI.template",       "P=13PS1:,R=ROI4:,  PORT=ROI4,ADDR=0,TIMEOUT=1")

# Create a processing plugin
NDProcessConfigure("PROC1", 20, 0, "PS1", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Proc1:,  PORT=PROC1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDProcess.template",   "P=13PS1:,R=Proc1:,  PORT=PROC1,ADDR=0,TIMEOUT=1")

# Create 5 statistics plugins
NDStatsConfigure("STATS1", 20, 0, "PS1", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Stats1:,  PORT=STATS1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=13PS1:,R=Stats1:,  PORT=STATS1,ADDR=0,TIMEOUT=1,HIST_SIZE=256")
NDStatsConfigure("STATS2", 20, 0, "ROI1", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Stats2:,  PORT=STATS2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ROI1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=13PS1:,R=Stats2:,  PORT=STATS2,ADDR=0,TIMEOUT=1,HIST_SIZE=256")
NDStatsConfigure("STATS3", 20, 0, "ROI2", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Stats3:,  PORT=STATS3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ROI2,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=13PS1:,R=Stats3:,  PORT=STATS3,ADDR=0,TIMEOUT=1,HIST_SIZE=256")
NDStatsConfigure("STATS4", 20, 0, "ROI3", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Stats4:,  PORT=STATS4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ROI3,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=13PS1:,R=Stats4:,  PORT=STATS4,ADDR=0,TIMEOUT=1,HIST_SIZE=256")
NDStatsConfigure("STATS5", 20, 0, "ROI4", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Stats5:,  PORT=STATS5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ROI4,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=13PS1:,R=Stats5:,  PORT=STATS5,ADDR=0,TIMEOUT=1,HIST_SIZE=256")

# Create a transform plugin
NDTransformConfigure("TRANS1", 20, 0, "PS1", 0, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Trans1:,  PORT=TRANS1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDTransform.template", "P=13PS1:,R=Trans1:,  PORT=TRANS1,ADDR=0,TIMEOUT=1")

# Create an overlay plugin with 8 overlays
NDOverlayConfigure("OVER1", 20, 0, "PS1", 0, 8, -1, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13PS1:,R=Over1:, PORT=OVER1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadTemplate("Overlay.substitutions")

# Create 2 color conversion plugins
NDColorConvertConfigure("CC1", 5, 0, "PS1", 0, 20, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template",   "P=13PS1:,R=CC1:,  PORT=CC1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=PS1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDColorConvert.template", "P=13PS1:,R=CC1:,  PORT=CC1,ADDR=0,TIMEOUT=1")
NDColorConvertConfigure("CC2", 5, 0, "PS1", 0, 20, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template",   "P=13PS1:,R=CC2:,  PORT=CC2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=CC1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDColorConvert.template", "P=13PS1:,R=CC2:,  PORT=CC2,ADDR=0,TIMEOUT=1")


#asynSetTraceMask("PS1",0,255)

set_requestfile_path("./")
set_savefile_path("./autosave")
set_requestfile_path("$(AREA_DETECTOR)/ADApp/Db")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
save_restoreSet_status_prefix("13PS1:")
dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db", "P=13PS1:")

iocInit()

#asynSetTraceMask("PS1",0,1)

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=13PS1:,D=cam1:")
