#ifndef RDI_ALGORITHMS_H
#define RDI_ALGORITHMS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/////////////////////////////////////////////////////////////////////////////
// Master Configuration Struct

typedef struct _configParams 
{
  //////////
  // Debug Flags
  //////////

  //Enable Debug Operations in main loop
  uint8_t debug_Main;

  //Enable Debug Operations in calibration code
  uint8_t debug_Calibrate;

  //Enable Debug Operations in processing code
  uint8_t debug_Processing;

  //Enable Debug Operations in profiling code
  uint8_t debug_Timing;

  //Enable Debug Operations for removeal of clock
  uint8_t debug_Clock;

  //Enable Verbose Debug in Main
  uint8_t debug_VerboseMain;

  //Enable Verbose Debug in Calibration code
  uint8_t debug_VerboseCalibrate;

  //Enable Verbose Debug Printing of Entire Frame Buffer
  uint8_t debug_VerboseFrameDump;

  //////////
  // FFT Flags
  //////////

  //Enable the Calculation of an FFT on the Samples of a Single Pixel
  uint8_t fft_enable;


  //////////
  // Profiling Flags
  //////////

  //Enable Debug Operations in main loop
  uint8_t profile_Camera;

  //Enable Debug Operations in calibration code
  uint8_t profile_Main;


  //////////
  // Output Dump Flags
  //////////

  //Enable dump of system status to std_out
  uint8_t dump_Status;

  //Enable dump of monitored Waveform
  uint8_t dump_Waveform;

  //Enable dump of calculated FFT Data
  uint8_t dump_FFT;


  //////////
  // Raw Functional Parameters
  //////////

  //Video Frame width in Pixels (1-65535)
  uint16_t video_width;

  //Video Frame Height in Pixels (1-65535)
  uint16_t video_height;

  //Frame Per Sec the Camera should updates at (1-255)
  uint8_t video_framerate;

  //Number of Seconds of video to Store (1-255)
  uint8_t buffer_limit;

  //Number of Pixels to Track for Motion (1-64)
  uint8_t monitor_pixels;

  //Number of Second of Throwaway Video At Startup
  uint8_t throwaway_secs;

  // FOV Processing Winwdow
  uint16_t window_origin_x;
  uint16_t window_origin_y;
  uint16_t window_width;
  uint16_t window_height;

  //Number of Seconds of Video to Use in Calibration (1-255)
  uint8_t calibration_seconds;

  // Base Interval Distance between calibration frames (1-255)
  // Starting at this value and multiply by 1,2,3 we generate all the
  // width test cases till limit is readched
  uint8_t calibration_width_interval;

  // Maximum Distance between calibration frames (1-255)
  uint8_t calibration_width_limit;

  // Base distance to offset differene calculations between frames (1-255)
  // Starting at this value and multiply by 1,2,3 until limit is reached
  // this create the other set of test width parameters
  uint8_t calibration_offset_interval;

  // Maximum frame offset (1-255)
  uint8_t calibration_offset_limit;

  // Enable Flag for Adaptive Calibration System
  uint8_t calibration_enable_adaptive;

  // Time (in secs) that must be waited before the
  //// camera is ready to serve an additional frame
  double frame_process_time;

  // Value to divide raw image size by to get pixel array for
  // image processing (Must be power of 2)
  uint8_t resize_factor;

  // Default Peak Width Prior to Any Sort of Calculation (1-255)
  uint8_t default_width;

  // Threshold Scaling Factor for Standard Deviation Component
  // Used for Peak Detection
  // (.00001-10)
  double thresh_scale;

  // Motion Threshold Scaling Factor for Standard Deviation Component
  // Used for Motion Event Detection
  // (.00001-10)
  double motion_scale;

  // Lower AVG Bound Scaling Factor for Standard Deviation Component
  //// Lower limit the average can be before triggering a recalibration
  //// (.00001-10)
  double lower_bound_scale ;

  //Defaults Threshold Value Prior to Collection of First Data Set
  uint8_t default_threshold;

  //Maximum Peaks Per Calculation Session (1-255)
  uint8_t max_peaks;

  //Sets how man and how old of pixels are used in the
  //mean and variance calculations
  //Start from SAMPLE_SIZE and count back process limit (1-255)
  uint16_t process_limit;

  //Sets how many seconds of camera data is processed for peaks
  //before a calibration event is forced
  //Can be set negative for infinite time
  //(-128 - 128)
  int8_t process_secs;

  // Motion Detection Frame Distance
  // Check Value #1
  uint8_t motion_frame_1;

  // Motion Detection Frame Distance
  // Check Value #2
  uint8_t motion_frame_2;

  // Motion Detection Overall Pixel Change Threshold
  double motion_threshold;

  // Alarm Focus Pixel
  // A non-zero value corresponds to a particular monitor pixel
  // that will be used for all alarm checking
  // If equal to zero, ALL monitor pixels will be checked for
  // alarm condition
  uint8_t focus_pixel;

  //Warning Limits
  //Number of Seconds with No Peaks Till the Warning (1-255)
  uint8_t warning_secs;

  //Number of Seconds with No Peaks till alarm is triggered (1-255)
  uint8_t alarm_sec;

  //XY Cooridinates of The "Clock Region"
  //Used when the camera super imposes a clock over the image (1-65535)
  uint16_t clock_x_start;
  uint16_t clock_x_stop ;
  uint16_t clock_y_start;
  uint16_t clock_y_stop ;

  //Number of Frame to Run the System (1-4294967296)
  //Set to 0 for Unlimited Running
  uint32_t run_limit;

  //////////
  // Calculated Functional Parameters
  //////////

  //Number of camera frames to store
  uint16_t stored_frames;

  ///Inital Frames to Capture and Discard while camera adjusts
  //Std is 1 second of video
  uint16_t throwaway_frames;

  // Resized Image Frame Width in Pixels (1-65535)
  uint16_t resize_width;

  //Resized Image Frame Height in Pixels (1-65535)
  uint16_t resize_height;

  //Total number of pixels in a frame
  uint32_t pixels;

  //Number of Frames to Check during Calibration
  uint16_t sample_size;

  //Number of Frames to Check during Peak Detection
  //Before starting recalibration
  int16_t process_window;

  //Length in secs of each frame from the camera
  double frame_time_ms;

  //Number of Frames with No Peaks before warning status
  uint16_t warn_frames;

  //Number of Frames with No Peaks before alarm status
  uint16_t alarm_frames;

  //Number of pixels that must exhibite a behavior in a voting situation
  uint8_t minority;

  // Enable recalibrate and peak detection on alarm state
  uint8_t alarm_recalibrate;

  // number of samples in circ resp rate buffer
  uint16_t resp_rate_buffer_samples;

  // number of samples per average in resp rate historization
  uint16_t resp_rate_avgs_per_sample;

  // enable waveform smoothing
  uint8_t waveform_smoothing_enabled;

  // enable waveform smoothing
  uint8_t waveform_averages;

} configParams;

/////////////////////////////////////////////////////////////////////////////
// Enums

typedef enum
{
    // represents current state of system

    mode_startup,
    mode_calibration_setup,
    mode_calibration_in_progress,
    mode_running,
    mode_motion

} mode;


typedef enum
{
    // represents alarm state if the system

    NO_ALARM,           // Camera on, Movement sensor on,  movement present
    PREALARM,           // Camera on, Movement sensor on,  no movement between 15-20 seconds
    ALARM               // Camera on, Movement sensor on, no movement detected after 20 seconds

}  alarm_state;


////////////////////////////////////////////////////////////////////////////
// Callbacks

// Used to be notified when mode of system changes
// newMode: mode enum
typedef void (__stdcall * ModeChangedCallback)(mode newMode);

// Used to be notified of calibration progress
// numCalFrames: number of frames that have been processed during current calibration
typedef void (__stdcall * CalibrationProgressCallback)(uint16_t numCalFrames);

// Used to be notified when calibration is complete
// *arMonPix: array of pixels identified during calibration (configParams.monitor_pixels determines length)
// calType: 0 for initial, 1 for adaptive/final
typedef void (__stdcall * CalibrationCompleteCallback)(uint32_t *arMonPix, uint8_t calType);

// Used to request respiration rate information
// numPeaksMaxPx: number of peaks found in waveform during most recent analysis window
// respRate: respiration rate determined via dft
typedef void (__stdcall * RespirationRateUpdateCallback)(uint16_t numPeaksMaxPx, double respRate);

// Used to request respiration waveform
// numSamples: number of samples in arWfrmData array
// *arWfrmData: waveform amplitude array
typedef void (__stdcall * WaveformAvailableCallback)(uint16_t numSamples, double *arWfrmData);

// User to request spectrum of respiration waveform
// numBins: number of raw bins in spectral array (not yet divided by 2, dc included)
// *arSpecData: spectrum amplitude array
typedef void (__stdcall * SpectrumAvailableCallback)(uint16_t numBins, double *arSpecData);

// Used to request notification of alarm event
// alarmType: alarm_state enum
// originalMaxPx: max pixel identified prior to alarm detection
// relocatedMaxPx: pixel relocated after alarm detected
typedef void (__stdcall * AlarmEventCallback)(alarm_state alarmType, uint32_t originalMaxPx, uint32_t relocatedMaxPx);

////////////////////////////////////////////////////////////////////////////
// public functions

// 
// usercfg: 
// modeChangedCallback: invoked whenever run mode changes
__declspec(dllexport) void SetupBuffers(configParams usercfg, ModeChangedCallback modeChangedCallback);

// 
// *frame_bytes: array of 8-byte image data, buffer length = usercfg.pixels, image width = usercfg.window_width, image height = userCfg.window_height
// usercfg: massive config struct defined above, could be passed in just once as part of setup if needed 
// modeChanged: invoked whenever run mode changes
// calProgress: discussed above, checked once per frame during calibration
// calComplete: discussed above, invoked when calibration complete (either initial or adaptive)
// respRateUpdate: discussed above, checked once per frame when mode_running
// wfrmAvailable: used to request waveform, this callback is checked once for every frame processed
// specAvailable: used to request spectrum, this callback is checked once for every frame processed
// respWfrmUpdate: used to request respiration waveform, this callback is invoked max of once per minute
// alarmEvent: used to request notification of alarm event
__declspec(dllexport) uint32_t ProcessNewImage(
                                                uint8_t *frame_bytes,                               
                                                configParams usercfg, 
                                                ModeChangedCallback modeChanged, 
                                                CalibrationProgressCallback calProgress, 
                                                CalibrationCompleteCallback calComplete,
                                                RespirationRateUpdateCallback respRateUpdate,
                                                WaveformAvailableCallback wfrmAvailable,
                                                SpectrumAvailableCallback specAvailable,
                                                WaveformAvailableCallback respWfrmUpdate,
                                                AlarmEventCallback alarmEvent
                                              );

__declspec(dllexport) void CleanupBuffers();

#endif
