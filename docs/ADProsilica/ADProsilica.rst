ADProsilica
===========

:author: Mark Rivers, University of Chicago

.. contents:: Contents

Overview
--------

This is an :doc:`../index` driver for Gigabit Ethernet and
Firewire cameras from `Allied Vision
Technologies <https://www.alliedvision.com>`__, who purchased Prosilica.
The driver is supported under Windows, Linux and Mac OS X using the
vendor library provided for those operating systems.

This driver inherits from :doc:`../ADCore/ADDriver`.
It implements nearly all of the parameters in
`asynNDArrayDriver.h <../areaDetectorDoxygenHTML/asyn_n_d_array_driver_8h.html>`__
and in
`ADArrayDriver.h <../areaDetectorDoxygenHTML/_a_d_driver_8h.html>`__. It
also implements a number of parameters that are specific to the
Prosilica cameras. The `prosilica class
documentation <../areaDetectorDoxygenHTML/classprosilica.html>`__ describes
this class in detail.

.. _StandardNotes:

Implementation of standard driver parameters
--------------------------------------------

The driver redefines the choices for several of the parameters defined
in ``ADDriver.h``. The ADTriggerMode choices for the Prosilica are:

-  Free Run (collects images as fast as allowed by the camera readout
   and the exposure time)
-  Sync In 1 (external trigger on line 1)
-  Sync In 2 (external trigger on line 2)
-  Sync In 3 (external trigger on line 3)
-  Sync In 4 (external trigger on line 4)
-  Fixed Rate (collects images at a fixed rate)
-  Software (single software trigger)

The Prosilica supports hardware timing input and output signals that are
supported in the driver.

The NDDataType choices for the Prosilica are:

-  NDUInt8 (8-bit data)
-  NDUInt16 (12 or 16 bit data)

The NDColorMode choices for the Prosilica are:

-  NDColorModeMono (monochromatic data)
-  NDColorModeBayer (raw Bayer data)
-  NDColorModeRGB1 (RGB1 data)

The color Prosilica cameras are also capable of various YUV color
formats but these are not supported in the driver. They may be added in
a future release.

Prosilica specific parameters
-----------------------------

The Prosilica driver implements the following parameters in addition to
those in asynNDArrayDriver.h and ADDriver.h:

.. raw:: html

  <table class="table table-bordered"> 
    <tbody>
      <tr>
        <td align="center" colspan="7,">
          <b>Parameter Definitions in prosilica.cpp and EPICS Record Definitions in prosilica.template</b>
        </td>
      </tr>
      <tr>
        <th>
          Parameter index variable</th>
        <th>
          asyn interface</th>
        <th>
          Access</th>
        <th>
          Description</th>
        <th>
          drvInfo string</th>
        <th>
          EPICS record name</th>
        <th>
          EPICS record type</th>
      </tr>
      <tr>
        <td align="center" colspan="7,">
          <b>Bayer Color Conversion</b></td>
      </tr>
      <tr>
        <td>
          PSBayerConvert</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Color conversion when NDColorMode is Bayer:<br />
          None: Raw Bayer images are passed to the plugins<br />
          RGB1: Bayer images are converted to RGB1<br />
          RGB2: Bayer images are converted to RGB2<br />
          RGB3: Bayer images are converted to RGB3<br />
          Having the camera send Bayer images uses 3 times less network bandwidth than 
          sending RGB1 images.  It does place more CPU load on the host to convert
          from Bayer to RGB, but this is often an acceptable tradeoff.</td>
        <td>
          PS_BAYER_CONVERT</td>
        <td>
          $(P)$(R)BayerConvert<br />
          $(P)$(R)BayerConvert_RBV</td>
        <td>
          mbbo
          <br />
          mbbi</td>
      </tr>
      <tr>
        <td align="center" colspan="7,">
          <b>Trigger and I/O Control</b></td>
      </tr>
      <tr>
        <td>
          PSTriggerEvent</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          The edge or level for the selected trigger signal when ADTriggerMode=Sync In 1 to
          SyncIn 4. Allowed values are:<br />
          Rising edge<br />
          Falling edge<br />
          Any edge<br />
          High level<br />
          Low level </td>
        <td>
          PS_TRIGGER_EVENT</td>
        <td>
          $(P)$(R)TriggerEvent<br />
          $(P)$(R)TriggerEvent_RBV</td>
        <td>
          mbbo
          <br />
          mbbi</td>
      </tr>
      <tr>
        <td>
          PSTriggerDelay</td>
        <td>
          asynFloat64</td>
        <td>
          r/w</td>
        <td>
          The delay in seconds between the trigger signal and when the frame is actually acquired.
          Minimum value is 1 microsecond. </td>
        <td>
          PS_TRIGGER_DELAY</td>
        <td>
          $(P)$(R)TriggerDelay<br />
          $(P)$(R)TriggerDelay_RBV</td>
        <td>
          ao
          <br />
          ai</td>
      </tr>
      <tr>
        <td>
          PSTriggerOverlap</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Controls the behavior when an external trigger signal arrives before the camera
          is ready for the next trigger. Allowed values are:<br />
          Off - the external trigger is ignored<br />
          Previous frame - the external trigger is latched and triggers the next frame when
          the current frame completes<br />
        </td>
        <td>
          PS_TRIGGER_OVERLAP</td>
        <td>
          $(P)$(R)TriggerOverlap<br />
          $(P)$(R)TriggerOverlap_RBV</td>
        <td>
          mbbo
          <br />
          mbbi</td>
      </tr>
      <tr>
        <td>
          PSTriggerSoftware</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Processing this record performs a software trigger if ADTriggerMode=Software.
        </td>
        <td>
          PS_TRIGGER_SOFTWARE</td>
        <td>
          $(P)$(R)TriggerSoftware</td>
        <td>
          bo</td>
      </tr>
      <tr>
        <td>
          PSSyncIn1Level</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          The level of the Sync In 1 signal</td>
        <td>
          PS_SYNC_IN_1_LEVEL</td>
        <td>
          $(P)$(R)SyncIn1Level_RBV</td>
        <td>
          bi</td>
      </tr>
      <tr>
        <td>
          PSSyncIn2Level</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          The level of the Sync In 2 signal</td>
        <td>
          PS_SYNC_IN_2_LEVEL</td>
        <td>
          $(P)$(R)SyncIn2Level_RBV</td>
        <td>
          bi</td>
      </tr>
      <tr>
        <td>
          PSSyncOut1Mode</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          The mode of the Sync Out 1 signal. Allowed values are:
          <br />
          GPO (general purpose output)
          <br />
          AcqTrigReady
          <br />
          FrameTrigReady
          <br />
          FrameTrigger
          <br />
          Exposing
          <br />
          FrameReadout
          <br />
          Imaging
          <br />
          Acquiring
          <br />
          SyncIn1
          <br />
          SyncIn2
          <br />
          SyncIn3
          <br />
          SyncIn4
          <br />
          Strobe1
          <br />
          Strobe2
          <br />
          Strobe3
          <br />
          Strobe4
          <br />
        </td>
        <td>
          PS_SYNC_OUT_1_MODE</td>
        <td>
          $(P)$(R)SyncOut1Mode
          <br />
          $(P)$(R)SyncOut1Mode_RBV </td>
        <td>
          mbbo
          <br />
          mbbi </td>
      </tr>
      <tr>
        <td>
          PSSyncOut1Level</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          The level of the Sync Out 1 signal. This is only programmable when SyncOut1Mode=GPO.
        </td>
        <td>
          PS_SYNC_OUT_1_LEVEL</td>
        <td>
          $(P)$(R)SyncOut1Level<br />
          $(P)$(R)SyncOut1Level_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          PSSyncOut1Invert</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Flag to invert the Sync Out 1 signal.</td>
        <td>
          PS_SYNC_OUT_1_INVERT</td>
        <td>
          $(P)$(R)SyncOut1Invert<br />
          $(P)$(R)SyncOut1Invert_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          PSSyncOut2Mode</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          The mode of the Sync Out 2 signal. Allowed values are the same as for PSSyncOut1Mode.
        </td>
        <td>
          PS_SYNC_OUT_2_MODE</td>
        <td>
          $(P)$(R)SyncOut2Mode
          <br />
          $(P)$(R)SyncOut2Mode_RBV </td>
        <td>
          mbbo
          <br />
          mbbi </td>
      </tr>
      <tr>
        <td>
          PSSyncOut2Level</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          The level of the Sync Out 2 signal. This is only programmable when SyncOut2Mode=GPO.
        </td>
        <td>
          PS_SYNC_OUT_2_LEVEL</td>
        <td>
          $(P)$(R)SyncOut2Level<br />
          $(P)$(R)SyncOut1Level_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          PSSyncOut2Invert</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Flag to invert the Sync Out 2 signal.</td>
        <td>
          PS_SYNC_OUT_2_INVERT</td>
        <td>
          $(P)$(R)SyncOut2Invert<br />
          $(P)$(R)SyncOut2Invert_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          PSSyncOut3Mode</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          The mode of the Sync Out 3 signal. Allowed values are the same as for PSSyncOut1Mode.
        </td>
        <td>
          PS_SYNC_OUT_3_MODE</td>
        <td>
          $(P)$(R)SyncOut3Mode
          <br />
          $(P)$(R)SyncOut3Mode_RBV </td>
        <td>
          mbbo
          <br />
          mbbi </td>
      </tr>
      <tr>
        <td>
          PSSyncOut3Level</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          The level of the Sync Out 3 signal. This is only programmable when SyncOut3Mode=GPO.
        </td>
        <td>
          PS_SYNC_OUT_3_LEVEL</td>
        <td>
          $(P)$(R)SyncOut3Level<br />
          $(P)$(R)SyncOut3Level_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          PSSyncOut3Invert</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Flag to invert the Sync Out 3 signal.</td>
        <td>
          PS_SYNC_OUT_3_INVERT</td>
        <td>
          $(P)$(R)SyncOut3Invert<br />
          $(P)$(R)SyncOut3Invert_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          PSStrobe1Mode</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          The mode of the Strobe 1 signal. The Strobe signals are based on the following values,
          but allow for changing the delay and width relative to the underlying value. Any
          of the outputs can be set to the Stobe1 value, rather than the raw values of these
          signals. Allowed values are:
          <br />
          AcqTrigReady
          <br />
          FrameTrigReady
          <br />
          FrameTrigger
          <br />
          Exposing
          <br />
          FrameReadout
          <br />
          Acquiring
          <br />
          SyncIn1
          <br />
          SyncIn2
          <br />
          SyncIn3
          <br />
          SyncIn4
          <br />
        </td>
        <td>
          PS_STROBE_1_MODE</td>
        <td>
          $(P)$(R)Strobe1Mode
          <br />
          $(P)$(R)Strobe1Mode_RBV </td>
        <td>
          mbbo
          <br />
          mbbi </td>
      </tr>
      <tr>
        <td>
          PSStrobe1CtlDuration</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Flag to allow controlling the strobe duration.</td>
        <td>
          PS_STROBE_1_CTL_DURATION</td>
        <td>
          $(P)$(R)Strobe1CtlDuration<br />
          $(P)$(R)Strobe1CtlDuration_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          PSStrobe1Duration</td>
        <td>
          asynFloat64</td>
        <td>
          r/w</td>
        <td>
          The strobe duration if PSStrobe1CtlDuration is On.</td>
        <td>
          PS_STROBE_1_DURATION</td>
        <td>
          $(P)$(R)Strobe1Duration<br />
          $(P)$(R)Strobe1Duration_RBV</td>
        <td>
          ao<br />
          ai</td>
      </tr>
      <tr>
        <td>
          PSStrobe1Delay</td>
        <td>
          asynFloat64</td>
        <td>
          r/w</td>
        <td>
          The strobe delay relative to the underlying signal that the strobe is based on.
        </td>
        <td>
          PS_STROBE_1_DELAY</td>
        <td>
          $(P)$(R)Strobe1Delay<br />
          $(P)$(R)Strobe1Delay_RBV</td>
        <td>
          ao<br />
          ai</td>
      </tr>
      <tr>
        <td align="center" colspan="7,">
          <b>Timestamp Control</b></td>
      </tr>
      <tr>
        <td>
          PSResetTimer</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Resets the timestamp timer in the camera. If PSTimestampType is POSIX or EPICS then
          it also stores the current POSIX or EPICS time in the driver.</td>
        <td>
          PS_RESET_TIMER</td>
        <td>
          $(P)$(R)PSResetTimer</td>
        <td>
          longout</td>
      </tr>
      <tr>
        <td>
          PSTimestampType</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Controls the type of timestamp in the timeStamp field of each NDArray. Choices are:
          <ul>
            <li>NativeTicks: The number of internal camera clock ticks which have elapsed since
              the last timer reset.</li>
            <li>NativeSeconds: The number of seconds which have elapsed since the last timer reset.
              This is NativeTicks divided by the internal camera clock frequency.</li>
            <li>POSIX: The number of seconds since the POSIX Epoch (00:00:00 UTC, January 1, 1970).</li>
            <li>EPICS The number of seconds since the EPICS Epoch (January 1, 1990).</li>
          </ul>
          The POSIX and EPICS timestamps are calculated as follows: when the timer is reset
          the current POSIX or EPICS time is stored, and the internal camera timer is reset.
          The timestamps are then computed by adding the camera ticks (converted to seconds)
          to the stored POSIX or EPICS time. Thus, the relative times of each frame are accurately
          controlled by the internal camera clock. The accuracy of the absolute time is determined
          by the accuracy of the time of day clock in the IOC computer.</td>
        <td>
          PS_TIMESTAMP_TYPE</td>
        <td>
          $(P)$(R)PSTimestampType<br />
          $(P)$(R)PSTimestampType_RBV</td>
        <td>
          mbbo<br />
          mbbi</td>
      </tr>
      <tr>
        <td align="center" colspan="7,">
          <b>Statistics Information</b></td>
      </tr>
      <tr>
        <td>
          PSReadStatistics</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Read the Gigabit Ethernet statistics when 1</td>
        <td>
          PS_READ_STATISTICS</td>
        <td>
          $(P)$(R)PSReadStatistics</td>
        <td>
          longout</td>
      </tr>
      <tr>
        <td>
          PSStatDriverType</td>
        <td>
          asynOctet</td>
        <td>
          r/o</td>
        <td>
          Driver type</td>
        <td>
          PS_DRIVER_TYPE</td>
        <td>
          $(P)$(R)PSDriverType_RBV</td>
        <td>
          stringin</td>
      </tr>
      <tr>
        <td>
          PSStatFilterVersion</td>
        <td>
          asynOctet</td>
        <td>
          r/o</td>
        <td>
          Packet filter version</td>
        <td>
          PS_FILTER_VERSION</td>
        <td>
          $(P)$(R)PSFilterVersion_RBV</td>
        <td>
          stringin</td>
      </tr>
      <tr>
        <td>
          PSStatFrameRate</td>
        <td>
          asynFloat64</td>
        <td>
          r/o</td>
        <td>
          Frame rate (Hz)</td>
        <td>
          PS_FRAME_RATE</td>
        <td>
          $(P)$(R)PSFrameRate_RBV</td>
        <td>
          ai</td>
      </tr>
      <tr>
        <td>
          PSByteRate</td>
        <td>
          asynInt32</td>
        <td>
          r/w</td>
        <td>
          Stream bytes per second in the PvAPI driver. This allows limiting the bandwidth
          that a camera uses. It also allows operation of GigE cameras on non-Gigabit Ethernet
          networks by decreasing the value to maximum that the network supports. The default
          of 115000000 allows full-speed operation on GigE networks.</td>
        <td>
          PS_BYTE_RATE</td>
        <td>
          $(P)$(R)PSByteRate<br />
          $(P)$(R)PSByteRate_RBV</td>
        <td>
          longout<br />
          longin</td>
      </tr>
      <tr>
        <td>
          PSPacketSize</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Actual packet size of Ethernet packets. When connecting to the camera the driver
          always automatically negotiates the largest packet size that the camera and IOC
          computer support. </td>
        <td>
          PS_PACKET_SIZE</td>
        <td>
          $(P)$(R)PSPacketSize_RBV</td>
        <td>
          longin</td>
      </tr>
      <tr>
        <td>
          PSStatFramesCompleted</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Number of frames completed</td>
        <td>
          PS_FRAMES_COMPLETED</td>
        <td>
          $(P)$(R)PSFramesCompleted_RBV</td>
        <td>
          longin</td>
      </tr>
      <tr>
        <td>
          PSStatFramesDropped</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Number of frames dropped</td>
        <td>
          PS_FRAMES_DROPPED</td>
        <td>
          $(P)$(R)PSFramesDropped_RBV</td>
        <td>
          longin</td>
      </tr>
      <tr>
        <td>
          PSStatPacketsErroneous</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Number of erroneous packets</td>
        <td>
          PS_PACKETS_ERRONEOUS</td>
        <td>
          $(P)$(R)PSPacketsErroneous_RBV</td>
        <td>
          longin</td>
      </tr>
      <tr>
        <td>
          PSStatPacketsMissed</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Number of missed packets</td>
        <td>
          PS_PACKETS_MISSED</td>
        <td>
          $(P)$(R)PSPacketsMissed_RBV</td>
        <td>
          longin</td>
      </tr>
      <tr>
        <td>
          PSStatPacketsReceived</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Number of received packets</td>
        <td>
          PS_PACKETS_RECEIVED</td>
        <td>
          $(P)$(R)PSPacketsReceived_RBV</td>
        <td>
          longin</td>
      </tr>
      <tr>
        <td>
          PSStatPacketsRequested</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Number of packets requested</td>
        <td>
          PS_PACKETS_REQUESTED</td>
        <td>
          $(P)$(R)PSPacketsRequested_RBV</td>
        <td>
          longin</td>
      </tr>
      <tr>
        <td>
          PSStatPacketsResent</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Number of packets resent</td>
        <td>
          PS_PACKETS_RESENT</td>
        <td>
          $(P)$(R)PSPacketsResent_RBV</td>
        <td>
          longin</td>
      </tr>
      <tr>
        <td>
          PSBadFrameCounter</td>
        <td>
          asynInt32</td>
        <td>
          r/o</td>
        <td>
          Number of bad frames</td>
        <td>
          PS_BAD_FRAME_COUNTER</td>
        <td>
          $(P)$(R)PSBadFrameCounter_RBV</td>
        <td>
          longin</td>
      </tr>
    </tbody>
  </table>

Configuration
-------------

The Prosilica driver is created with the ``prosilicaConfig`` command, either
from C/C++ or from the EPICS IOC shell.

.. code-block:: c

   int prosilicaConfig(char *portName,
                       const char* cameraId,
                       int maxBuffers, size_t maxMemory,
                       int priority, int stackSize, int maxPvAPIFrames)
     

The **cameraId** string can be any of the following:

-  The camera's UniqueId, which is a number assigned by the vendor to
   each Prosilica camera, e.g. 50110.
-  The camera's IP address, e.g. 164.54.160.48.
-  The camera's IP DNS name, e.g. gse-prosilica1.cars.aps.anl.gov.

Using the UniqueId has the advantage that the cameras can be configured
to use DHCP, and hence have non-predictable TCP/IP addresses. However,
if the UniqueId is used then the areaDetector IOC must be on the same
subnet as the camera, since cameras cannot be found by UniqueID through
routers. The simplest way to determine the uniqueId of a camera is to
run the Prosilica GigEViewer application, select the camera, and press
the "i" icon on the bottom of the main window to show the camera
information for this camera. The Unique ID will be displayed on the
first line in the information window.

The IP address or IP DNS name can be used for cameras with fixed IP
addresses, and **must** be used for cameras that are not on the local
subnet.

The maxPvAPIFrames parameter controls how many frame buffers will be
used by the PvAPI library. This is the last parameter in the
prosilicaConfig command, and if it is absent the default value of 2 is
used, which is sufficient in most circumstances. However, with very high
frame rates or busy IOCs increasing this value can reduce dropped
frames.

For details on the meaning of the other parameters to this function
refer to the detailed documentation on the prosilicaConfig function in
the `prosilica.cpp
documentation <../areaDetectorDoxygenHTML/prosilica_8cpp.html>`__ and in
the documentation for the constructor for the `prosilica
class <areaDetectorDoxygenHTML/classprosilica.html>`__.


.. toctree::
   :hidden:
   
   st_cmd.rst

There is an example IOC boot directory and startup script
:ref:`st.cmd <ADProsilica_st_cmd_example>` provided with
areaDetector.

MEDM screens
------------

The following is the MEDM screen prosilica.adl.

.. figure:: prosilica.png
    :align: center

The following is the MEDM screen that provides access to the specific
parameters for the Prosilica detector.

.. figure:: prosilicaMore.png
    :align: center

The following is an IDL
`epics_ad_display <https://cars.uchicago.edu/software/idl/imaging_routines.html#epics_ad_display>`__
screen displaying the Prosilica detector images.

.. figure:: prosilica_tvscl.jpg
    :align: center

Connection management
---------------------

The Prosilica driver implements connection management. Cameras do not
need to be accessible when the IOC starts up, and can be power-cycled or
disconnected and reconnected from the Ethernet without restarting the
IOC.

Whenever a new camera is detected on the network the Prosilica library
issues a callback to the driver. If the driver is not currently
connected to a camera it will attempt to connect when receiving the
callback. This mechanism should work no matter how the camera is
identified in the startup script, i.e. by Unique ID, IP address, or IP
name. It is also possible to manually connect and disconnect the camera
by using the ``$(P)$(R)AsynIO.CNCT`` PV, which is labeled "Connect" and
"Disconnect" on the medm screen.

If the camera is not accessible when the IOC boots, or is power-cycled
then the EPICS output records may not match the actual camera settings
and readbacks. They can be made to agree by processing the output
record, e.g. by pressing Enter or Return in the medm output widget. In
the future this may be improved by sending all of the EPICS settings to
the camera when it connects, though it is not clear if this would always
be the desired behavior.


