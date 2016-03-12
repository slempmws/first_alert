#include "string.h"
#include "rdi_alg.h"
#include "fft.h"

///////////////////////////////////////////////////////////////////////////////
// static / private functions

static uint8_t calc_DC_offset (uint8_t *frame,configParams usercfg);

static uint8_t calc_std_dev(uint8_t *buffer, uint8_t mean,configParams usercfg);

static uint8_t process_peaks(uint8_t *camera_frames,uint32_t *motion_pixels,uint32_t *lastpeakframe,uint32_t frame_total,configParams usercfg);

static void process_stream(uint8_t stream,configParams usercfg);

static uint16_t calc_FIFO_address(uint16_t offset, uint16_t pivot, uint16_t queue_pos,configParams usercfg);

static void inital_setup(uint8_t *camera_frames,uint32_t *motion_pixels,uint32_t frameaddr,configParams usercfg);

static void intial_sample_load(uint8_t *camera_frames,uint8_t stream,uint32_t stream_pixel,uint32_t frameaddr,configParams usercfg);

static void calc_stream_params(configParams usercfg);

static void update_sample_buffer(uint32_t *motion_pixels,uint8_t *camera_frames,uint32_t new_frame,configParams usercfg);

static void update_queue_position(configParams usercfg);

static uint16_t GetPeakCount();

// Alarming Function
static uint8_t event_checking(uint32_t *lastpeakframe,uint32_t frame_total, configParams usercfg);

// Calibration

static uint8_t init_calibration(uint8_t width_interval, uint8_t width_limit, uint8_t offset_interval, uint8_t offset_limit, uint32_t pixels, uint32_t start_frame);

static uint8_t update_calibration(uint8_t *camera_frames, uint16_t current_buffer_position, uint16_t calibration_sample_number, configParams usercfg);

static void difference_frame(uint8_t *camera_frames,uint16_t newFrame,uint16_t oldFrame, uint8_t *diff_frame, configParams usercfg);

static void add_frame(uint8_t *diffFrame, uint32_t *sumFrame, configParams usercfg);

static uint8_t finalize_calibration(configParams usercfg, uint32_t *max_pixel_pos);

static uint8_t cleanup_calibration(void);

static void PerformAdaptiveCalibration(configParams usercfg);

static void RerunCalibration(configParams usercfg);

// fft

static uint8_t calc_fft_and_resp_rate(configParams usercfg, uint8_t num);

// resp rate

static void calc_resp_rate(double* real, double* imag, configParams usercfg);

static uint8_t GetRespWaveform(double* wfrm);

static double CalcPhase(double real, double imag);

// waveform

static uint8_t GetWaveform(double* wfrm, configParams usercfg);


/////////////////////////////////////////////////////////////////////////////
// Structs

// Define Pixel Data Storage Stucture
// struct stores the a value, posistion and the address
// of the next smallest pixel (basically a linked list)
struct list_el 
{
    uint32_t pixel_pos;
    uint16_t pixel_val;
    struct list_el * next;
};

typedef struct list_el pixel_data;

// Instantiate Detection Parameter Struct
typedef struct _DetectionParams{
  //Calculated DC Offset of All Pixels in A Frame
  uint8_t dc_offset[64];													// mws - thought it was top 5? monitoring_pixels configured to be 5?  is 64 assumed worst case?

  //Calculated Standard Deviation of All Pixels in A Frame
  uint8_t std_dev[64];

  //Minimum Pixel Value to Start and End a Peak
  uint8_t threshold[64];

  // Upper Limit for Stream Average to Indicate a Motion Event
  uint8_t motion_high_thresh[64];

  // Lower Limit for Stream Average to Indicate a Motion Event
  uint8_t motion_low_thresh[64];

  //Mininum Number of Samples above the Threshold to be considered a peak
  uint16_t peak_width;

  //Current Position in the Sample Buffer Cyclical Address Range
  uint16_t queue_pos;

  //2D Array that acts as cyclical buffer for pixel sample data
  //New Data is inserted in the oldest sample (starting from 0)
  uint8_t sample_buffer[64][2048];											// mws - is 2048 assumed largest circ buffer size?

  //Number of Peaks Found in Last Processing Step
  uint16_t peaks[64];

  //Number of Samples in the Array that were greater than the threshold
  uint16_t overthresh[64];

  //Pixel Positions of the last
  uint32_t peak_positions[64][64];											// mws - assuming this is worst case, max_configured = 64

  //Lower limit of stream average to count as an excursion
  uint8_t lower_bound[64];

  // Storage for frame where last peak was detected
  uint32_t latest_peak;

  // Respiration rate circ buffer (12 hours @ one sample per minute)
  double resp_rate[720];

  // Respiration rate sample count
  uint8_t resp_rate_count;

} DetectionParams;



// Debug Enables
// #define DEBUG_PROCESSING

// #define DUMP_SUMMATION_BIN
// #define PEAK_PROCESSING

#define CameraFrames(r, c) (camera_frames[((r)*usercfg.pixels) + (c)])
#define FrameBufferAddr(x,y) ((x) % (y))
#define Add_Ints(x,y) ((x) +(y))

// Choose Alarming Mode
// #define PEAK_DIFF_ALARM // Pre-alarm/Full Alarm based on distance to last peak detected
#define NO_PEAKS_ALARM  // Instant alarming based on no peaks present

#define PI 3.14159265

const double FFT_THRESHOLD = 0.001;

DetectionParams PeakDetect;		

// Global Array Pointers
uint32_t **madd_array;
uint16_t **param_array;

//Global Variables for Calibration
uint32_t cal_start_frame;
uint8_t  test_cases;

uint16_t resp_rate_samples;
double resp_rate_avg;

// global variables moved from python
uint32_t totalFrames;
uint16_t newFrameCount;
uint16_t samples;

uint8_t *frame_buffer;
uint32_t *arMonitorPix;
uint32_t *arPeakCountPix;

double *arMag;
double *arReal;
double *arImag;

mode runMode;
alarm_state alarmMode;
uint8_t currentAlarmResult;

uint8_t optimizedWidth;
uint8_t optimizedOffset;
double optimalSamplingRate;
double respRate;
double phase;

void SetupBuffers(configParams usercfg, ModeChangedCallback modeChanged)
{
    totalFrames = 0;
    newFrameCount = 0;
    samples = 0;

    frame_buffer = (uint8_t *)malloc(usercfg.video_framerate * usercfg.buffer_limit * usercfg.pixels * sizeof(uint8_t *));
    arMonitorPix = (uint32_t *)malloc(usercfg.monitor_pixels * sizeof(uint32_t *));
    arPeakCountPix = (uint32_t *)malloc(usercfg.monitor_pixels * sizeof(uint32_t *));

    // fft buffers
    arMag = (double *)malloc(usercfg.sample_size * sizeof(double));
    arReal = (double *)malloc(usercfg.sample_size * sizeof(double));
    arImag = (double *)malloc(usercfg.sample_size * sizeof(double));

    runMode = mode_startup;

    if (modeChanged)
    {
        // send progress update
        modeChanged(runMode);
    }

}


void CleanupBuffers()
{
    free(frame_buffer);
    free(arMonitorPix);
    free(arPeakCountPix);
    
    free(arMag);
    free(arReal);
    free(arImag);
}


uint32_t ProcessNewImage(
                            uint8_t *frame_bytes, 
                            configParams usercfg, 
                            ModeChangedCallback modeChanged, 
                            CalibrationProgressCallback calProgress, 
                            CalibrationCompleteCallback calComplete,
                            RespirationRateUpdateCallback respUpdate,
                            WaveformAvailableCallback wfrmAvailable,
                            SpectrumAvailableCallback specAvailable,
                            WaveformAvailableCallback respWfrmUpdate,
                            AlarmEventCallback alarmEvent
                        )
{
    uint32_t i;
    uint8_t ret;
    uint16_t start_frame;
    uint32_t prevMaxPix;

    int iLen;
    double* wfrm;
    double* respWfrm;

    totalFrames++;

    newFrameCount = (uint16_t)((totalFrames) % usercfg.stored_frames);

    for (i = 0; i < usercfg.pixels; i++)
    {
        // calc offset

        uint32_t index = newFrameCount * (usercfg.pixels) + i;

        frame_buffer[index] = frame_bytes[i];
    }

    if (runMode == mode_startup)
    {
        samples++;
                
        if (samples == usercfg.throwaway_secs * usercfg.video_framerate)
        {
            runMode = mode_calibration_setup;

            if (modeChanged)
                modeChanged(runMode);
        }
    }
    else if (runMode == mode_calibration_setup)
    {
        init_calibration(
                            usercfg.calibration_width_interval,
                            usercfg.calibration_width_limit, 
                            usercfg.calibration_offset_interval, 
                            usercfg.calibration_offset_limit,
                            usercfg.pixels,
                            totalFrames
                        );

        runMode = mode_calibration_in_progress;

        if (modeChanged)
            modeChanged(runMode);

        samples = 0;

        update_calibration(frame_buffer, newFrameCount, samples, usercfg);

        samples++;
    }
    else if (runMode == mode_calibration_in_progress)
    {
        // need to test to get out of calibration

        if (samples == usercfg.calibration_seconds * usercfg.video_framerate)
        {
            // finish calibration
            ret = finalize_calibration(usercfg, arMonitorPix);

            // callback indicating initial calibration complete
            if (calComplete)
                calComplete(arMonitorPix, 0);
                                        
            // initialize frame processor
            start_frame = newFrameCount - usercfg.sample_size;

            inital_setup(frame_buffer, arMonitorPix, start_frame, usercfg);

            cleanup_calibration();
                    
            // perform adaptive adaptive cal
            
            ret = calc_fft_and_resp_rate(usercfg, (uint8_t)usercfg.sample_size);


            PerformAdaptiveCalibration(usercfg);

            // callback indicating adaptive calibration complete
            if (calComplete)
                calComplete(arMonitorPix, 1);
                    
            runMode = mode_running;

            // callback 
            if (modeChanged)
                modeChanged(runMode);

            samples = 0;
        }
        else
        {
            update_calibration(frame_buffer, newFrameCount, samples, usercfg);

            samples++;

            // callback 
            if (calProgress)
                calProgress(samples);
        }
    }
    else if (runMode == mode_running)
    {
        samples++;

        update_sample_buffer(arMonitorPix, frame_buffer, newFrameCount, usercfg);
                
        // process
        process_peaks(frame_buffer, arMonitorPix, arPeakCountPix, totalFrames, usercfg);

        currentAlarmResult = event_checking(arPeakCountPix, totalFrames, usercfg);

        // waveform available callback
        if (wfrmAvailable)
        {
            iLen = usercfg.sample_size;
            if (usercfg.waveform_smoothing_enabled != 0)
                iLen -= (usercfg.waveform_averages - 1);
        
            wfrm = (double *)malloc(iLen * sizeof(double));

            GetWaveform(wfrm, usercfg);

            wfrmAvailable(iLen, wfrm);

            free(wfrm);
        }

        ret = calc_fft_and_resp_rate(usercfg, (uint8_t)usercfg.sample_size);

        // callback to make peak count & resp rate available in UI
        if (respUpdate)
            respUpdate(PeakDetect.peaks[0], respRate);

        // callback to update spectrum
        if (specAvailable)
            specAvailable(usercfg.sample_size, arMag);

        // callback to update resp wfrm
        if (respWfrmUpdate)
        {
            iLen = PeakDetect.resp_rate_count;

            respWfrm = (double *)malloc(iLen * sizeof(double));

            ret = GetRespWaveform(respWfrm);

            if (ret != 0)
                respWfrmUpdate(iLen, respWfrm);
        }

        if (currentAlarmResult != 0 && alarmMode == NO_ALARM)
        {
            // perform adaptive calibration

            prevMaxPix = arMonitorPix[0];

            // mws - Problem...
            //  1) phase may be off from previous adaptive cal
            //  2) signal is crap w/o motion so can't ftt again
            //  May need to either fall back to 16 width or buffer width/phase and pull that from before alarm event
            
            //PerformAdaptiveCalibration();
            RerunCalibration(usercfg);

            // if max pix changed, perform analysis again before alarming
            if (prevMaxPix != arMonitorPix[0])
            {
                // need to perform analysis on new max pixel
                process_peaks(frame_buffer, arMonitorPix, arPeakCountPix, totalFrames, usercfg);

                currentAlarmResult = event_checking(arPeakCountPix, totalFrames, usercfg);
            }

            if (currentAlarmResult != 0)
            {
                alarmMode = ALARM;
                
                if (alarmEvent)
                {
                    alarmEvent(alarmMode, prevMaxPix, arMonitorPix[0]);
                }
            }
        }
    }

    return totalFrames;
}



void update_sample_buffer(uint32_t *motion_pixels,uint8_t *camera_frames,uint32_t new_frame,configParams usercfg)
{
    // Add Next Pixel Value to Waveform Sample Queue
    // Only used in running mode / after calibration
    
    uint8_t byte_value = 0;
    uint8_t i;

    //Update Data Buffers
    for (i = 0; i < usercfg.monitor_pixels; ++i) 
    {

        //Get Byte From Master Frame Buffer
        byte_value = CameraFrames(new_frame,motion_pixels[i]);

        //Write value in to stream buffer (queue_pos is position in cir buffer)
        PeakDetect.sample_buffer[i][PeakDetect.queue_pos] = byte_value;

        //Calculate pixel DC Offset (Average value of every buffer sample for a given pixel position)
        PeakDetect.dc_offset[i] = calc_DC_offset(PeakDetect.sample_buffer[i], usercfg);

        //Calculate the Standard Deviation of pixel stream
        PeakDetect.std_dev[i] = calc_std_dev(PeakDetect.sample_buffer[i], PeakDetect.dc_offset[i], usercfg);

        #ifdef DEBUG_PROCESSING
          fprintf(stderr,"Updating sample buffer %d \n",i);
        fprintf(stderr,"Queue Position: %d\n",PeakDetect.queue_pos);
        fprintf(stderr,"Value to Insert: %d\n",byte_value);
        #endif
    }

    update_queue_position(usercfg);
}


void update_queue_position(configParams usercfg)
{
  //Update Queue Position to Next Place
  //Wrap around to zero if reach sample size
  if (PeakDetect.queue_pos < usercfg.sample_size)
    PeakDetect.queue_pos++;
  else
    PeakDetect.queue_pos = 0;

}



uint8_t process_peaks(uint8_t *camera_frames,uint32_t *motion_pixels,uint32_t *lastpeakframe,uint32_t frame_total,configParams usercfg)
{
    //   *camera_frames	- pointer to frame buffer
    //   *motion_pixels - array of monitored locations
    //   *lastpeakframe - array of monitored pixels that contains last frame where pk detected
    //   frame_total	- count of available frames

    uint8_t i,j;

    //Process Thru Each pixel stream checking for excursion from the mean and then
    // executing the main peak detection routine
    for (i = 0; i < usercfg.monitor_pixels; ++i)
    {

        #ifdef DUMP_BIN_WAVEFORM
          printSampleBuffer(i);
        #endif
        #ifdef PEAK_PROCESSING
        printf("Processing frames %d to %d for motion pixel %d (Pixel Address: %d) \n",frame_total-usercfg.sample_size,frame_total,i,motion_pixels[i]);	// mws - sample size = cal sec (10) * frame rate
        printf("Using threshold value: %d \n",PeakDetect.threshold[i]);
        #endif

        /////////
        // Process Data to Get Peak Number and Positions
        ////////
        process_stream(i,usercfg);
        ////////

        #ifdef PEAK_PROCESSING
          printf("Number of Peaks Found %d \n",PeakDetect.peaks[i]);
        printf("Number of Samples over threshold %d \n",PeakDetect.overthresh[i]);
        #endif

        ///Get Number of Peak Detected by checking for the any non-zero values in peak_positions array
        j=0;
        #ifdef PEAK_PROCESSING
        printf("Peaks at [");
        #endif

        while (PeakDetect.peak_positions[i][j])
        {
            #ifdef PEAK_PROCESSING
            printf("%d,",PeakDetect.peak_positions[i][j]);
            #endif
            j++;
        }
        #ifdef PEAK_PROCESSING
            printf("] \n");
        #endif

        /////Calculate and Print the Absolute Frame Number for each peak location
        //if a peak was found on this run...
        if (j > 0) 
        {
            //..Go back to last peak found,
            j--;
            //Get the absolute position on the total number of frames processed
            //taking found peak with highest index and putting in lastpeakframe
            lastpeakframe[i] = (frame_total - usercfg.sample_size) + PeakDetect.peak_positions[i][j];

            #ifdef PEAK_PROCESSING
                printf("Real Frame Number for Last Peak: %d \n",lastpeakframe[i]);
            #endif
        }
        #ifdef PEAK_PROCESSING
        printf("\n");
        #endif
  }

//  printf("Peaks: ");
//  for (i = 0; i < usercfg.monitor_pixels; ++i){
//    printf("%d ",PeakDetect.peaks[i]);
//  }
//  printf("\n");

  return(0);
}


void process_stream(uint8_t stream, configParams usercfg)
{
	uint16_t i;

	//Flag for if number samples indicate a peak
	uint8_t peak_active = 0;

	//Stores the Number of samples over threshold to that make up a peak
	uint8_t threshold_count = 0;

	//Starting sample for the pixel
	uint16_t peak_start =0;

	//Address Buffer for samples in cyclical buffer
	uint16_t array_pos = 0;


	//Precalculate the Pivot of the cyclical array
	const uint16_t array_pivet = (usercfg.sample_size - PeakDetect.queue_pos);

	////////////////////////////////
	//Clear Over-Threshold Counter
	PeakDetect.overthresh[stream] = 0;

	//Clear Peak Count
	PeakDetect.peaks[stream] = 0;		

	//Clear Peak Position Data
	for (i = 0; i < usercfg.max_peaks; ++i) 
	{
		PeakDetect.peak_positions[stream][i] = 0;
	}

	//Process each sample
	for (i = 0; i < usercfg.sample_size; ++i)
	{																							// mws - using sample size which is based on calib duration
		array_pos  = calc_FIFO_address(i,array_pivet,PeakDetect.queue_pos,usercfg);

		//////////////////////// Sample Process Logic////////////////////////
		//If the sample is over the threshold
		if (PeakDetect.sample_buffer[stream][array_pos] > PeakDetect.threshold[stream])
		{
			//The first sample over the threshold
			if (!threshold_count) peak_start = i;

			//Increment over master and local threshold counters
			PeakDetect.overthresh[stream] = PeakDetect.overthresh[stream] + 1;
			threshold_count++;

			//Check if enough samples are over threshold to count as a peak
			if (threshold_count == PeakDetect.peak_width)
			{
				peak_active = 1;
			}
		}

		//If the sample is below the threshold
		else 
		{
			//Clear threshold counter
			threshold_count = 0;

			//Peak was active last cycle, now sample is below threshold indicating end of peak
			if (peak_active)
			{

				// Increment Peak Count
				PeakDetect.peaks[stream] = PeakDetect.peaks[stream] + 1;

				//Clear Peak Flag
				peak_active = 0;
				//Save Center Point of Last Peak
				PeakDetect.peak_positions[stream][(PeakDetect.peaks[stream]-1)] = (i-1) - (((i-1) - peak_start) >> 1);	// mws - finds center of (end - start)
			}
		}
	}

	//If sample data indicate a peak was still in progress,
	//calculate the best guess for center positon
	if (peak_active)
	{
		//Save Center Point of Last Peak
		PeakDetect.peak_positions[stream][PeakDetect.peaks[stream] - 1] = ((usercfg.sample_size - peak_start) >> 1) + peak_start;
	}
}


uint8_t event_checking(uint32_t *lastpeakframe, uint32_t frame_total, configParams usercfg)
{
    // Evaluate the relative position of the newest frame the system has detected
    // versus the current frame and set alarm state

    uint32_t i;

    // Alarming mode where the newest peak is checked for how
    #ifdef PEAK_DIFF_ALARM																// mws - commented out / not defined / not active
    uint32_t frame_diff = 0;

    // If focus pixel is set to a valid monitor pixel, then ONLY use
    // the peaks for that pixel  for alarming											// mws - appears as if focus_pixel is ON / 1
    if (usercfg.focus_pixel && (usercfg.focus_pixel <= usercfg.monitor_pixels)) {
        if (lastpeakframe[usercfg.focus_pixel - 1] > PeakDetect.latest_peak) {			// mws - latest peak set to start frame on initialization, updated in peak finding
            PeakDetect.latest_peak = lastpeakframe[usercfg.focus_pixel - 1];
        }
    }
    // Otherwise, check all pixels for new peak Value
    else {
        for (i = 0; i < usercfg.monitor_pixels; ++i) {
            // Find newest peak from all monitor pixels
            if (lastpeakframe[i] > PeakDetect.latest_peak) {
                PeakDetect.latest_peak = lastpeakframe[i];
            }
        }
    }

    // Find the distance to that frame from the current frame
    frame_diff = frame_total - PeakDetect.latest_peak;
    // printf(" %d\n",frame_diff);

    //Check if distance is greater than warning and alarm limits
    if (frame_diff > usercfg.warn_frames) {												// mws - warning secs (15) * frame rate
        if (frame_diff > usercfg.alarm_frames) {										// mws - alarm secs (20) * frame rate
            return ALARM;
        }
        return PREALARM;
    }

    return NO_ALARM;
    #endif


    // Alarming Mode where anytime no peaks are found, the alarm is instantly activated
    #ifdef NO_PEAKS_ALARM
    // If focus pixel is set to a valid monitor pixel, then ONLY use
    // the peaks for that pixel  for alarming
    if (usercfg.focus_pixel && (usercfg.focus_pixel <= usercfg.monitor_pixels)) {		// mws - currently focus_pixel is ON / 1, only using 1 px for alarming, other px are output as wfrm stream
        if (PeakDetect.peaks[usercfg.focus_pixel - 1]) {
            return NO_ALARM;
        }
    }
    // Otherwise, check all pixels for new peaks
    else {
        for (i = 0; i < usercfg.monitor_pixels; ++i) {
            // See if any peaks exist for any pixel
            if (PeakDetect.peaks[i]) {
                return NO_ALARM;
            }
        }
    }

    return ALARM;
    #endif
}


uint16_t GetPeakCount()
{
	return PeakDetect.peaks[0];
}


uint16_t calc_FIFO_address(uint16_t offset, uint16_t pivot, uint16_t queue_pos,configParams usercfg)
{
    //Sample Arrays are cyclical FIFOs (First In First Out)
    //Start form an offset and calculate around a pivot
    //in the buffer where it wraps back around to zero

    uint16_t array_pos = 0;

    if (queue_pos + offset < usercfg.sample_size)
    {
        array_pos = queue_pos + offset;
    }
    else
    {
        array_pos = offset - pivot;
    }
    
    return array_pos;
}

// Calibration Functions

uint8_t init_calibration(uint8_t width_interval, uint8_t width_limit, uint8_t offset_interval, uint8_t offset_limit, uint32_t pixels, uint32_t start_frame) {

    // width_interval, width_limit = set to 16 in cfg file
    // offset_interval & offset_limit = set to 0 in cfg file, couldn't find them set anywhere
    // pixels = "resized"(?) pixels in each frame, subset of entire image?
    // start_frame = total frames in buffer - sample size (cal sec (10) * frame rate)
    
    uint32_t i;
    uint32_t width_temp;
    uint32_t offset_temp;
    uint32_t count;

    uint8_t width_values,offset_values;

    // Determine Number of Wdith Modes
    if (width_interval > 0) {
        width_values = width_limit / width_interval;
    }
    else {
        width_values = 1;
    }

    // Determine Number of Offset Modes
    if (offset_interval > 0) {
        offset_values = (offset_limit / offset_interval) + 1;
    }
    else {
         offset_values = 1;
    }

    test_cases = width_values * offset_values;

    if (!test_cases){
        return 1;
    }

    // Build madd array

	madd_array = (uint32_t **)malloc(test_cases * sizeof(uint32_t *));
    for (i=0; i < test_cases; i++){
      madd_array[i] = (uint32_t *)calloc(pixels , sizeof(uint32_t));
    }

    // Build parameter array	// mws - what is parameter array

    param_array = (uint16_t **)malloc(test_cases * sizeof(uint16_t *));
    for (i=0; i < test_cases; i++){
        param_array[i] = (uint16_t *)calloc(3 , sizeof(uint16_t));
    }

    // Load Parameters
    width_temp = width_interval;
    offset_temp = 0;
    count = 0;

    for (i = 0; i < test_cases; ++i){
        param_array[i][0] = width_temp;
        param_array[i][1] = offset_temp;
        count += offset_interval;

        if (count > offset_limit || offset_interval == 0) {
            width_temp += width_interval;
            offset_temp = 0;
            count = 0;
        }
        else {
            offset_temp += offset_interval;
        }
    }

    // (Re)set global flags
    cal_start_frame = start_frame;
    return 0;
}



uint8_t update_calibration(uint8_t *camera_frames, uint16_t current_buffer_position, uint16_t calibration_sample_number, configParams usercfg) 
{

    uint32_t i;

    //Working Frame Number
    int16_t temp_frame_num;

    //Difference of the pixel intensity from one calibration frame to the next
	
    uint8_t *diffarray = (uint8_t *)malloc(usercfg.pixels * sizeof(uint8_t));

    //Check if update is needed
    for (i = 0; i < test_cases; ++i)
    {
        // Calculate Relative Frame Position
        temp_frame_num = calibration_sample_number - param_array[i][1];

        // Check  if relative frame position is withing calculation limits
        if ((temp_frame_num > 0) && (temp_frame_num <= usercfg.sample_size)){

            // Check if Current Frame is eligible for calibration
            if (((temp_frame_num) % param_array[i][0]) == 0) {
																									
                // Calculate Position in Buffer of Old Comparison Frame
                uint16_t last_frame = (((cal_start_frame + calibration_sample_number) - param_array[i][0]) % usercfg.stored_frames);

                //Get absolute difference of each pixel in each calibration frame (the most current frame and frame as determined above by offset)
                difference_frame(camera_frames, current_buffer_position, last_frame, diffarray, usercfg);

                //Add frame difference to total difference summation
                add_frame(diffarray, madd_array[i], usercfg);

                //Update Frame Count
                param_array[i][2] += 1;
            }
        }
    }

	free(diffarray);

    return 0;
}



void difference_frame(uint8_t *camera_frames,uint16_t newFrame,uint16_t oldFrame, uint8_t *diff_frame, configParams usercfg)
{
    // Return Difference Matrix of New and Old Frame
    // Difference Matrix is written in place of old frame

    uint32_t i;
    int16_t  diff_signed;
    uint8_t  diff_abs;

    for (i = 0; i < usercfg.pixels; ++i)
    {
        //Get Signed Difference
        diff_signed =  CameraFrames(oldFrame,i) - CameraFrames(newFrame,i);

        //Get Absolute value (may need to replace for embedded)												// mws - Calvin says should be ok / disregard comment
        diff_abs = abs(diff_signed);

		if (diff_abs > 0)
		{
			int z = 0;
		}

        //Assign Diff Value in place in oldFrame
        diff_frame[i] = diff_abs;
    }
} 



void add_frame(uint8_t *diffFrame,uint32_t *sumFrame, configParams usercfg)
{
    //Add New Diff Matrix to Sum Matrix
  
    uint32_t i;
  
    for (i = 0; i < usercfg.pixels; ++i)
    {
        sumFrame[i] = diffFrame[i] + sumFrame[i];
    }
}



uint8_t finalize_calibration(configParams usercfg, uint32_t *max_pixel_pos) 
{
    uint32_t i,j,k;

    // Sorting Storage Variables
    uint32_t new_pos;
    uint16_t new_val;
    pixel_data *new_pixel;
    pixel_data *old_pointer = 0;
    pixel_data *eval_pixel_1;
    pixel_data *eval_pixel_2;
    pixel_data *eval_pixel_3;  // Debug Only Delete me Later

    // Sorting Flags
    uint8_t unique_position = 1;
    uint8_t order_disturbed = 1;
    uint8_t reassign_order = 1;
    uint8_t reassign_pointer = 1;
    uint8_t sorted = 1;

	uint8_t order_match;
	uint8_t new_pixel_present;

    pixel_data *max_pixels;
	pixel_data *largest_pixel;
	pixel_data *smallest_pixel;

    /*
    *** Setup Maximum Pixel Search System ***
    */

    // Create Array of pixel data structs
    max_pixels = (pixel_data *)malloc(usercfg.monitor_pixels * sizeof(pixel_data));

    //// Fill in the Linked List with data and address links
    // Fill in Values/Positons
    for (i = 0; i < usercfg.monitor_pixels; ++i)
    {
        max_pixels[i].pixel_val = i;
        max_pixels[i].pixel_pos = i;
    }

    // Fill in links
    for (i = usercfg.monitor_pixels - 1; i > 0; --i)
    {
        max_pixels[i].next = &max_pixels[i - 1];
    }

    //Smallest Pixel has a NULL address
    max_pixels[0].next = 0;

    // Create pointers to largest and smallest val/pos pairs
    largest_pixel = &max_pixels[usercfg.monitor_pixels - 1];
    smallest_pixel = &max_pixels[0];

    #ifdef DUMP_SUMMATION_BIN
      char str[20];
      FILE * fp;
      for (i = 0; i < test_cases; ++i) {
        sprintf(str, "sum_bin_%03d.bin", i);
        fp = fopen (str, "w");
        fwrite(madd_array[i], 4, usercfg.pixels, fp);
        fclose(fp);
      }
    #endif

    //Normalize all data
    for (i = 0; i < test_cases; ++i)
    {
        for (j = 0; j < usercfg.pixels; ++j)
        {
            madd_array[i][j] /= param_array[i][2];
        }
    }
    // Process thru all the different test case summation frames
    for (i = 0; i < test_cases; ++i)
    {
        //Process thru the entire array of pixels for each Summation Frame
        for (j = 0; j < usercfg.pixels; ++j)
        {

            new_val = madd_array[i][j];
            new_pos = j;
            eval_pixel_1 = largest_pixel;
            eval_pixel_2 = largest_pixel->next;

            unique_position = 1;  // Flag that Pixel Position Does Not Repeat One Already in the Max Pixels Array
            order_disturbed = 1;  // Flag wether or not a new value/position combo requires sorting the array

            reassign_order = 1;  // Flag to check if the array need sorting to accomdate the new pixel
            reassign_pointer = 1;
            sorted = 1;

            //extra_tree_walk = 0;

            // Begin sorting process if value needs to be inserted

            if (new_val > smallest_pixel->pixel_val) {
                // Check if Position Value is Already In Used
                for (k = 0; k < usercfg.monitor_pixels; ++k)
                {
                    // If pixel position is already in palce and pixel diff value is greater
                    if (new_pos == max_pixels[k].pixel_pos){
                        unique_position = 0;  // Clear flag so new value isn't stored in the current smallest pixel position
                        old_pointer = max_pixels[k].next;  // Store old pointer for reordering later

                        // Check if new pixel value is larger current for a particular pixel
                        if (new_val > max_pixels[k].pixel_val){
                            new_pixel = &max_pixels[k];

                            //Check if already in largest pixel positon
                            if (&max_pixels[k] == largest_pixel) {
                                max_pixels[k].pixel_val = new_val;
                                order_disturbed = 0;
                            }
                        }
                        // If pixel value is smaller than exisiting value, discard and skip further sorting
                        else {
                            order_disturbed = 0;
                            sorted = 0;
                        }
                        break;
                    }
                }

                // If the new pixel did not replace the largest pixel with the
                if (order_disturbed) {

                    // New Pixel Value does not exactly replace another position
                    // new pixel value and position will be inserted into the smallest pixel pos
                    if (unique_position) {
                        reassign_pointer = 0;
                        new_pixel = smallest_pixel;
                    }

                    // Check if bigger than largest pixel
                    if (new_val > largest_pixel->pixel_val) {
                        reassign_order = 0;
                        new_pixel->next = largest_pixel;
                        largest_pixel = new_pixel;
                    }
                    // Check if new pixels value is equal to
                    // the largest pixels mag and then sort based on pos
                    else if (new_val == largest_pixel->pixel_val) {

                        // New Positon is greater
                        if (new_pos > largest_pixel->pixel_pos){
                            new_pixel->next = largest_pixel;
                            largest_pixel = new_pixel;
                            reassign_order = 0;
                        }
                        // New Position is lower
                        else {
                            if (largest_pixel->next != new_pixel) {
                                eval_pixel_2 = largest_pixel->next;
                                if ((eval_pixel_2->pixel_val !=new_val) || (eval_pixel_2->pixel_pos < new_pos)){
                                    new_pixel->next = largest_pixel->next;
                                    largest_pixel->next = new_pixel;
                                }
                            }
                        }
                    }

                    new_pixel->pixel_val = new_val;
                    new_pixel->pixel_pos = new_pos;
                    eval_pixel_1 = largest_pixel;
                    eval_pixel_2 = largest_pixel->next;

                    // Traverse the Pixel List In Order to Check If Re-order need to Be Done
                    for (k = 0; k < usercfg.monitor_pixels - 1; ++k) {

                        // Check if new pixels old pointer needs to be reassigned due to a pixel that now incorrectly
                        // points to the new pixel when it should point to the old target of new pixel
                        if (reassign_pointer) {
                            if (eval_pixel_2 != 0) {
                                if ((eval_pixel_2->next == new_pixel) && ((eval_pixel_2->pixel_val < new_val) || ((eval_pixel_2->pixel_val == new_val)&& (eval_pixel_2->pixel_pos < new_pos)))) {
                                    eval_pixel_2->next = old_pointer;

                                    // Clear flag
                                    reassign_pointer = 0;
                                }
                            }
                        }

                        // Check for new pixel inserted to correct place
                        if ((reassign_order) && (eval_pixel_2 != 0)) {
                            //Test if Values are Equal and Need to Fall Back to Position for Sorting
                            if (new_pixel != eval_pixel_2) {
                                if (new_val == eval_pixel_2->pixel_val ) {
                                    if (new_pos > eval_pixel_2->pixel_pos) {
                                        eval_pixel_1->next = new_pixel;
                                        new_pixel->next = eval_pixel_2;
                                        reassign_order = 0;
                                    }
                                }

                                else if (new_val > eval_pixel_2->pixel_val) {

                                    // Insert new pixel in between current evaluation pixels
                                    eval_pixel_1->next = new_pixel;
                                    new_pixel->next = eval_pixel_2;

                                    // Clear flag to check if new value is larger than existing values
                                    reassign_order = 0;
                                }
                            }
                        }

                        // Complete Loop by either assigning smallest pixel value
                        if (k == usercfg.monitor_pixels - 2 ) {
                            eval_pixel_2->next = 0;
                            smallest_pixel = eval_pixel_2;
                        }
                        // or Moving Pointers forward
                        else{
                            eval_pixel_1 = eval_pixel_1->next;
                            eval_pixel_2 = eval_pixel_1->next;
                        }
                    }
                }

                /*
                // Test For Proper Sorting
                */
                if (sorted) {
                    eval_pixel_1 = largest_pixel;
                    eval_pixel_2 = largest_pixel-> next;
                    order_match = 1;
                    new_pixel_present = 0;
	                    for (k = 0; k < usercfg.monitor_pixels - 1; ++k) {

                        // Check on General List Order
                        if (eval_pixel_1->pixel_val >= eval_pixel_2->pixel_val) {
                            if (eval_pixel_1->pixel_val == eval_pixel_2->pixel_val) {
                                if (eval_pixel_1->pixel_pos <= eval_pixel_2->pixel_pos) {
                                    order_match = 0;
                                }
                            }
                        }
                        else {
                            order_match = 0;
                        }
                        // Check that the new valu is present in the list somewhere
                        if ((eval_pixel_1->pixel_val == new_val) && (eval_pixel_1->pixel_pos == new_pos)){
                            new_pixel_present = 1;
                        }

                        if ((eval_pixel_2->pixel_val == new_val) && (eval_pixel_2->pixel_pos == new_pos)){
                            new_pixel_present = 1;
                        }
                        eval_pixel_2 = eval_pixel_2->next;
                        eval_pixel_1 = eval_pixel_1->next;
                    }
                    // Check that both flags are set and not cleared
                    if (order_match && new_pixel_present) {}
                    else {
                        printf("Bad Sort\n");
                        // Print Results
                        for (k = 0; k < usercfg.monitor_pixels; ++k) {
                            printf("Slot: %d Value: %02d Position: %02d Next: %07d Address:%d\n",k, max_pixels[k].pixel_val, max_pixels[k].pixel_pos, max_pixels[k].next,&max_pixels[k]);
                        }

                        eval_pixel_3 = largest_pixel;
                        for (k = 0; k < usercfg.monitor_pixels; ++k) {
                            printf("Pixel %02d: Value %02d Position %02d  Address %d Next Pixel %d\n", k, eval_pixel_3->pixel_val, eval_pixel_3->pixel_pos, eval_pixel_3, eval_pixel_3->next);
                            eval_pixel_3 = eval_pixel_3->next;
                        }
                        printf("Largest Pos %d\n",largest_pixel);
                        printf("Smallest Pos %d\n",smallest_pixel);
                        while(1){}
                    }
                }
                /*
                // End Test For Proper Sorting
                */
            }
        }
    }

    // At this point max_pixels contains the information for the pixels we're interested in
    // need to extract the pixel positions and pass it back the processing engine
    eval_pixel_1 = largest_pixel;  // Start with the largest pixel

    // Iteratively work your way through the values found
    for (i = 0; i < usercfg.monitor_pixels; ++i) {												// mws - update array passed in with pixels to be monitored
            max_pixel_pos[i] = eval_pixel_1->pixel_pos;
            printf("%d %d %d\n",i,max_pixel_pos[i],eval_pixel_1->pixel_val);
            eval_pixel_1 = eval_pixel_1->next;
    }
    // Remember to Free that malloc memory!!
    free(max_pixels);

    return 0;
}


void PerformAdaptiveCalibration(configParams usercfg)
{
    double respFreq;
    double offset;    
    
    respFreq = respRate / 60;

    // find mutiple of 15 fps that most closely matches resp freq
    optimalSamplingRate = usercfg.video_framerate / respFreq;

    if (phase > 0)
        //offset = optimalSamplingRate - phase / (2d * Math.PI) * optimalSamplingRate;
        offset = phase / (2 * PI) * optimalSamplingRate;
    else
        //offset = optimalSamplingRate - (Math.PI + (Math.PI + _phase)) / (2d * Math.PI) * optimalSamplingRate;
        offset = (PI + (PI + phase)) / (2 * PI) * optimalSamplingRate;

    optimizedWidth = (uint8_t)floor(optimalSamplingRate / 2 + 0.5);
    optimizedOffset = (uint8_t)floor(offset + 0.5);

    RerunCalibration(usercfg);
}


void RerunCalibration(configParams usercfg)
{
    uint16_t startFrame;
    uint16_t idx;
    uint16_t relIdx;
    uint8_t ret;

    if ((uint32_t)usercfg.calibration_seconds * usercfg.video_framerate + optimizedOffset <= totalFrames)
        startFrame = (uint16_t)(totalFrames - usercfg.calibration_seconds * usercfg.video_framerate - optimizedOffset);
    else
        startFrame = (uint16_t)(totalFrames - usercfg.calibration_seconds * usercfg.video_framerate + floor(optimalSamplingRate - optimizedOffset + 0.5));

    ret = init_calibration(
                                optimizedWidth,
                                optimizedWidth,
                                0,                      // optimizedOffset,
                                0,                      // optimizedOffset,
                                usercfg.pixels,
                                startFrame
                             );

    samples = 0;

    for (idx = startFrame; idx < totalFrames; idx++)
    {
        // translate absolute index into relative pos in fifo buffer
        relIdx = (uint16_t)(idx % usercfg.stored_frames);             // index into frame buffer
                
        update_calibration(frame_buffer, relIdx, samples, usercfg);

        samples++;
    }

    ret = finalize_calibration(usercfg, arMonitorPix);

    cleanup_calibration();
}



uint8_t cleanup_calibration(void) 
{
    uint32_t i;
    for (i = 0; i < test_cases; i++)
    {
        uint32_t* currentIntPtr = madd_array[i];
        free(currentIntPtr);
    }

    free(madd_array);

    for (i = 0; i < test_cases; i++)
    {
        uint16_t* currentIntPtr = param_array[i];
        free(currentIntPtr);
    }

    free(param_array);
    return 0;
}


void inital_setup(uint8_t *camera_frames, uint32_t *motion_pixels, uint32_t start_frame, configParams usercfg)
{
    //   camera_frames - 2 dim array [stored frames = framerate (15) * buffer_length (45), pixels]
    //   motion_pixels - array of pixels being monitored
    //   start_frame - total sample size - cal size

    
    uint8_t i= 0;
    PeakDetect.peak_width = usercfg.default_width;

    //fprintf(stderr,"starting from %d\n",start_frame); //
    ///Intial Fill of Sample Buffers
    for (i = 0; i < usercfg.monitor_pixels; ++i)
    {
        intial_sample_load(camera_frames,i,motion_pixels[i],start_frame, usercfg);
        //fprintf(stderr, "%d \n",i); //REMOVE ME LATER
    }
    //Get Threshold to detect peaks and excursions by
    calc_stream_params(usercfg);

    // Set the latest peak to the very beginning of calibration period
    // No actual peak  may exist here, but it is a safe reset value
    PeakDetect.latest_peak = start_frame;
    
    resp_rate_samples = 0;
    resp_rate_avg = 0;
}


/* Local Functions */

void intial_sample_load(uint8_t *camera_frames, uint8_t stream, uint32_t stream_pixel, uint32_t start_frame, configParams usercfg)
{
    //Gather Sample Data From the Calibration Frames to create the initial data set
    //Remove DC offset and add to sample buffer of specific pixel

    //   *camera frames - all available frames
    //   stream - pixel of interest
    //   stream_pixel - array of pixels of interest for this stream
    //   start_frame - total frames - cal time * camera framerate

    
    uint32_t i;
    const uint32_t frame_buffer_size = usercfg.stored_frames;		// mws - stored_frames = framerate * buffer_limit

    //printf("Loading pixel %d\n",stream_pixel);
    for (i = 0; i < usercfg.sample_size; ++i)		// mws - sample size = cal sec (10) * frame rate (15) -> how many frames are we looking at
    {
		PeakDetect.sample_buffer[stream][i] = (camera_frames[((((i+start_frame) % (frame_buffer_size)))*usercfg.pixels) + (stream_pixel)]); //- PeakDetect.dc_offset;
    }

    //Reset Queue Position to Zero
    PeakDetect.queue_pos = 0;


    for (i = 0; i < usercfg.resp_rate_buffer_samples; ++i)
    {
	    PeakDetect.resp_rate[i] = 0;
    }

    PeakDetect.resp_rate_count = 0;

}



void calc_stream_params(configParams usercfg)
{
    //Prior to Processing additional frames,
    //find a baseline value of the mean pixel values and
    //determine the std_deviation and peak detect threshold

    // mws - only called during initial setup, setting up params for different pixel streams being monitored, after calibration complete

    
    uint32_t i;
      double std_dev_component;
    int16_t motion_limit = 0;

      //Calculate Threshold Using Standard Deviation and DC OFfset
    for (i = 0; i < usercfg.monitor_pixels; ++i) 
    {

        //Calculate pixel DC Offset (Average value of every buffer sample for a given pixel position)
        PeakDetect.dc_offset[i] = calc_DC_offset(PeakDetect.sample_buffer[i], usercfg);
        #ifdef DEBUG_PROCESSING
          fprintf(stderr,"DC OffSet for stream %d: %d\n",i,PeakDetect.dc_offset[i]);
        #endif

        //Calculate the Standard Deviation of pixel stream
        PeakDetect.std_dev[i] = calc_std_dev(PeakDetect.sample_buffer[i],PeakDetect.dc_offset[i],usercfg);
        #ifdef DEBUG_PROCESSING
        fprintf(stderr,"Standard Deviation for stream %d: %d\n",i,PeakDetect.std_dev[i]);
        #endif

        //Theshold is avg + (user_scaling * std deviation)
        std_dev_component = usercfg.thresh_scale * PeakDetect.std_dev[i];									// mws - thresh_scale = 0.2
        PeakDetect.threshold[i] = PeakDetect.dc_offset[i] + (uint8_t)std_dev_component;
        #ifdef DEBUG_PROCESSING
          fprintf(stderr,"User Scaling for stream %d: %f\n",i,std_dev_component);
        fprintf(stderr,"Peak Threshold for stream %d: %d\n",i,PeakDetect.threshold[i]);
        #endif

        //Motion Thresholds is avg +/- (user_scaling * std deviation)
        std_dev_component = usercfg.motion_scale * PeakDetect.std_dev[i];									// mws - motion scale = 2

        //Notice limiting function to prevent rollover
        motion_limit = PeakDetect.dc_offset[i] + (int16_t)std_dev_component;
        if (motion_limit > 255) 
    		PeakDetect.motion_high_thresh[i] = 255;										// mws - not afraid of rollover for peak detect above ?
        else 
	    	PeakDetect.motion_high_thresh[i] = (uint8_t)motion_limit;

        motion_limit = PeakDetect.dc_offset[i] - (int16_t)std_dev_component;											// mws - motion low threshold
        if (motion_limit < 0) 
    		PeakDetect.motion_low_thresh[i] = 0;
        else 
    		PeakDetect.motion_low_thresh[i] = (uint8_t)motion_limit;

        #ifdef DEBUG_PROCESSING
          fprintf(stderr,"Std_dev %d: %d\n",i,PeakDetect.std_dev[i]);
          fprintf(stderr,"Motion Scale %d: %f\n",i,usercfg.motion_scale);
          fprintf(stderr,"User Scaling for Motion Event Detection for stream %d: %f\n",i,std_dev_component);
          fprintf(stderr,"Current Stream Average %d: %d\n",i,PeakDetect.dc_offset[i]);
          fprintf(stderr,"Upper Motion Threshold %d: %d\n",i,PeakDetect.motion_high_thresh[i]);
          fprintf(stderr,"Lower Motion Threshold %d: %d\n",i,PeakDetect.motion_low_thresh[i]);
        #endif

    }
}



uint8_t calc_DC_offset (uint8_t *frame,configParams usercfg)
{
    //Finds Average Pixel Value for the newest pixels as set by PROCESS_LIMIT
    // *frame = PeakDetect.SampleBuffer[pixel of interest] - waveform buffer

    uint32_t i,pix_sum;
    uint8_t average;

    pix_sum = 0;

    for (i = 0; i < usercfg.process_limit; ++i)															// mws - process_limit = 80 - frames to consider
    {
        pix_sum = pix_sum + frame[(usercfg.sample_size-usercfg.process_limit) + i];							// mws - (cal seconds * framerate = 150) - frames to consider (80) + i
    }

    average = pix_sum/usercfg.process_limit;

    return average;

}


uint8_t calc_std_dev(uint8_t *buffer, uint8_t mean,configParams usercfg)
{
    //Given the mean of a data set, calculate the Standard Deviation

    uint32_t i =0;
    uint32_t running_sum = 0;
    int16_t diff_val = 0;
    uint32_t square_val = 0;
    uint16_t variance = 0;
    uint8_t std_dev = 0;


    for (i = 0; i < usercfg.process_limit; ++i)	// mws - process_limit = 80
    {
        diff_val = buffer[(usercfg.sample_size-usercfg.process_limit)+i] - mean;							// mws - sample size = cal time (10) * frame rate (15) - frame rate (80)
        square_val = diff_val*diff_val;
        running_sum = square_val + running_sum;
    }

    variance = running_sum/usercfg.process_limit;
    std_dev = (uint8_t)sqrt((float)variance);

    // Std Deviation must be a minimum of 1 for rest of system to work
    if (std_dev == 0)
    {
        std_dev = 1;
    }

    return std_dev;
}


uint8_t calc_fft_and_resp_rate(configParams usercfg, uint8_t num)
{
    uint8_t i;
    uint8_t idxResp;
    int ret;

    uint16_t array_idx;

    const uint16_t array_pivot = (usercfg.sample_size - PeakDetect.queue_pos);	// intermediate value

    // perform DFT

    for (i = 0; i < usercfg.sample_size; ++i)		// mws - sample size = cal sec (10) * frame rate (15) -> how many frames are we looking at
    {
	    // queue pos is position in circ buffer
	    array_idx = calc_FIFO_address(i, array_pivot, PeakDetect.queue_pos, usercfg);					// mws - calc index in circ buffer for this sample i

	    arReal[i] = PeakDetect.sample_buffer[0][array_idx];		// 0 index into sample buffer is pixel w/ most movement
        arImag[i] = 0;
    }

    ret = DFT(1, num, arReal, arImag);

    // calculate magnitude

    for (i = 0; i < usercfg.sample_size; i++)
    {
	    arMag[i] = sqrt(arReal[i] * arReal[i] + arImag[i] * arImag[i]) * 2;
    }

    // calc respiration rate

    calc_resp_rate(arReal, arImag, usercfg);

    // take care of resp rate historization

    resp_rate_samples++;
    if (resp_rate_samples == usercfg.resp_rate_avgs_per_sample)
    {
 	    // update avg
	    resp_rate_avg = ((resp_rate_avg * (resp_rate_samples - 1)) + respRate) / resp_rate_samples;

	    if (PeakDetect.resp_rate_count < usercfg.resp_rate_buffer_samples)
	    {
    	    idxResp = PeakDetect.resp_rate_count;
	      	PeakDetect.resp_rate_count++;
	    }
	    else
	    {
		    // shift array values 1-719 down to 0-718
		    memcpy(&PeakDetect.resp_rate[0], &PeakDetect.resp_rate[1], (usercfg.resp_rate_buffer_samples - 1) * sizeof(PeakDetect.resp_rate[0]));
	    }

	    // fill in resp value
	    PeakDetect.resp_rate[idxResp] = resp_rate_avg;

	    // reset avg / samples
	    resp_rate_samples = 0;
	    resp_rate_avg = 0;
	  
    }
    else
    {
	    // update avg
	    resp_rate_avg = ((resp_rate_avg * (resp_rate_samples - 1)) + respRate) / resp_rate_samples;
    }

    return 1;
}


void calc_resp_rate(double* real, double* imag, configParams usercfg)
{
	uint16_t i;

	double max = FFT_THRESHOLD;
	uint16_t firstLine = 1;		// skip line 0 / DC
	uint16_t idx = 0;

	double y1, y2, y3, d;
	double origFreq, finalFreq;
	double deltaFreq;

	// find largest peak
	
	for (i = firstLine; i < usercfg.sample_size / 2; i++)
    {
		if (arMag[i] > max)
        {
			max = arMag[i];
            idx = i;
        }
	}

	deltaFreq = 1 / (double)usercfg.calibration_seconds;
	origFreq = ((float)idx) * deltaFreq;

	if (idx == 0)
    {
        respRate = 0;
        phase = 0;
    }
    else
    {
        if (idx > 1 && idx < usercfg.sample_size / 2)
	    {
    		y1 = fabs(arMag[idx - 1]);
		    y2 = fabs(arMag[idx]);
	    	y3 = fabs(arMag[idx + 1]);
		
    		//d = (y3 - y1) / (2 * (2 * y2 - y1 - y3));

	    	d = (y3 - y1) / (y1 + y2 + y3);

    		finalFreq = origFreq + d * deltaFreq;
	    }
    	else
	    {
    		finalFreq = origFreq;
        }

	    respRate = finalFreq * 60;

    	phase = CalcPhase(real[idx], imag[idx]);
    }

	return;
}


double CalcPhase(double real, double imag)
{
	double phase, deg;
	
	if (real > 0)			// quadrant 1 or 2
	{
		phase = atan(imag / real);
	}
	else if (imag > 0)		// quadrant 2
	{
		phase = atan(imag / real) + PI;
	}
	else					// quadrant 3
	{
		phase = atan(imag / real) - PI;
	}

	deg = phase * 180 / PI;

	return phase;
}


uint8_t GetWaveform(double* wfrm, configParams usercfg)
{
  uint8_t i, j, idxRel;
  uint8_t iStart, iAvgs;
  uint16_t array_idx;
  uint16_t sum;

  const uint16_t array_pivot = (usercfg.sample_size - PeakDetect.queue_pos);	// intermediate value

  if (usercfg.waveform_smoothing_enabled == 0)
  {
      iStart = 0;
      iAvgs = 1;
  }
  else
  {
      iStart = usercfg.waveform_averages - 1;
      iAvgs = usercfg.waveform_averages;
  }

  for (i = iStart; i < usercfg.sample_size; ++i)		// mws - sample size = cal sec (10) * frame rate (15) -> how many frames are we looking at
  {
      sum = 0;

      for (j = 0; j < iAvgs; j++)
      {
          idxRel = i - j;
          
          // queue pos is position in circ buffer
	      array_idx = calc_FIFO_address(idxRel, array_pivot, PeakDetect.queue_pos, usercfg);					// mws - calc index in circ buffer for this sample i

	      sum += PeakDetect.sample_buffer[0][array_idx];
      }

      wfrm[i - iStart] = sum / iAvgs;
  }

  return 1;
}


uint8_t GetRespWaveform(double* wfrm)
{
	if (resp_rate_samples == 0)
	{
		memcpy(wfrm, &PeakDetect.resp_rate[0], PeakDetect.resp_rate_count * sizeof(PeakDetect.resp_rate[0]));

		return 1;
	}
    
	return 0;
}
