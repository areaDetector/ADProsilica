ADProsilica Releases
==================

The latest untagged master branch can be obtained at
https://github.com/areaDetector/ADProsilica.

Tagged source code and pre-built binary releases prior to R2-0 are included
in the areaDetector releases available via links at
https://cars.uchicago.edu/software/epics/areaDetector.html.

Tagged source code releases from R2-0 onward can be obtained at 
https://github.com/areaDetector/ADProsilica/releases.

Tagged prebuilt binaries from R2-0 onward can be obtained at
https://cars.uchicago.edu/software/pub/ADProsilica.

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.


Release Notes
=============
R2-5 (2-July-2018)
----
* Changed configure/RELEASE files for compatibility with areaDetector R3-3.
* Added support for new PVs in ADCore R3-3 in opi files (NumQueuedArrays, etc.)
* Added ADBuffers.adl to main medm screen.
* Improved op/*/autoconvert/* files with better medm files and better converters.
* Removed calls to unlock()/lock() around doCallbacksGenericPointer, not needed.
* Fixed op/Makefile for autoconvert.


R2-4 (31-January-2018)
----
* Removed calls to release lock around calls to doCallbacksGenericPointer.
* Fixed medm adl files to improve the autoconversion to other display manager files.
* Added op/Makefile to automatically convert adl files to edl, ui, and opi files.
* Updated the edl, ui, and opi autoconvert directories to contain the conversions
  from the most recent adl files.


R2-3 (05-July-2017)
----
* Fixed layout of medm screen for ADCore R3-0.


R2-2 (21-February-2017)
----
* Added PSTimestampType to prosilica_settings.req.  Thank to Stuart Wilkins for this fix.
* Added support for new parameters ADSerialNumber, ADFirmwareVersion, ADSDKVersion, NDDriverVersion 
  from ADCore R2-6. Minor change to driver and medm screen.

R2-1 (16-April-2015)
----
* Fixed bug in asynPrintIO call in frameCallback. It was printing data on previous frame, 
  not current frame, and would crash if executed on first callback.
* Changes for compatibility with ADCore R2-2.

R2-0 (20-March-2014)
----
* Moved the repository to [Github](https://github.com/areaDetector/ADProsilica).
* Re-organized the directory structure to separate the driver library from the example IOC application.
* Fixes for connection management to handle multiple cameras per IOC. Thanks to Kate Feng.
* Added support for conversion from raw Bayer to RGB1, RGB2, and RGB3 in the driver. 
  In areaDetector releases prior to R2-0 this could be done in the NDPluginColorConvert plugin.  
  That capability has been removed from the plugin to make the plugin independent of the AVT PvApi library.

R1-9-1 and earlier
------------------
Release notes are part of the
[areaDetector Release Notes](https://cars.uchicago.edu/software/epics/areaDetectorReleaseNotes.html).
