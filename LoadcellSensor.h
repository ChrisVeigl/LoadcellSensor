/**************************************************************************/
/**
  @file   LoadcellSensor.h
  Author: Chris Veigl (Asterics Foundation)
  License: GPL (see license.txt)

  This is a library for processing raw values from a load cell or similar sensor
  for application in HCI applications (e.g. for mouse cursor control).

  The values are noise-filtered and compared to a baseline which adapts to slow drifting.
  Automatic calibration is supported (either by resetting the baseline or by apating movement thresholds).
  Note: the values should be fed into the LoadcellSensor::process() method periodically

  Thanks to Jim Peters for the marvellous fiview filter tool and the fidlib filter library:
  http://uazu.net/fiview/

  The AsTeRICS Foundation promotes Open Source Assistive Technologies, visit:
  https://www.asterics-foundation.org
  
*/
/**************************************************************************/

extern "C" {
#include "fidlib.h"
}

#define AUTOCALIBRATION_DISABLED        0 
#define AUTOCALIBRATION_RESET_BASELINE  1 
#define AUTOCALIBRATION_ADAPT_THRESHOLD 2 


// default frequency settings
#define SAMPLE_RATE 25       // sampling rate: 25Hz
#define LP_BASELINE 0.1      // low pass cutoff frequency for baseline calculation (Hz)
#define LP_NOISE    3        // low pass cutoff frequency for denoising the signal (Hz)
#define LP_ACTIVITY 2        // low pass cutoff frequency for activity/idle detection (Hz)

// default signal conditioning parameters
#define GAIN                        1.00   // gain for incoming values
#define MOVEMENT_THRESHOLD          1000   // deflection from baseline which indicates a valid movement (gain normalized)
#define THRESHOLD_DECAY             0.97   // threshold correction decay time (close to 1 -> slower decay)
#define IDLE_DETECTION_THRESHOLD    3000   // noise theshold value for auto calibration (gain normalized)
#define IDLE_DETECTION_PERIOD       1000   // in milliseconds
#define BYPASS_BASELINE             10     // bypass baseline calculation n times after a movement (avoid drift)

// other signal processing settings (fixed)
#define THRESHOLD_CORRECTION_VALUE  400   // if autocalibration mode is "adapt threshold", this update value is used

// filter identification bitmasks
#define FILTER_BASELINE   (1<<0)
#define FILTER_NOISE      (1<<1)
#define FILTER_ACTIVITY   (1<<2)
#define FILTERS_ALL       (FILTER_BASELINE|FILTER_NOISE|FILTER_ACTIVITY)

/**************************************************************************/
/*!
    @brief  LoadcellSensor class
*/
/**************************************************************************/

class LoadcellSensor {
public:
  LoadcellSensor();
  ~LoadcellSensor();
  void     calib(void);
  int32_t  process(int32_t value);
  void     clearActivity(void);
  int32_t  getActivity(void);
  void     setMovementThreshold(int32_t movementThreshold);
  void     setIdleDetectionThreshold(int32_t idleDetectionThreshold);
  void     setIdleDetectionPeriod(int32_t idleDetectionPeriod);
  void     setThresholdDecay(double thresholdDecay);
  void     setGain(double gain);
  void     setSampleRate(double sampleRate);
  void     setBaselineLowpass(double lpBaseline);
  void     setNoiseLowpass(double lpNoise);
  void     setActivityLowpass(double lpActivity);
  void     setAutoCalibrationMode(uint8_t mode);
  bool     isMoving(void);
  void     lockBaseline(bool b);
  void     printValues(uint8_t mask, int32_t limit);


private:

  int32_t  movementThreshold;
  int32_t  idleDetectionThreshold,idleDetectionPeriod;
  double   thresholdDecay;
  double   gain;
  double   sampleRate,lpBaseline,lpNoise,lpActivity;
  
  int32_t  raw,filtered,activity,baseline,offset;
  int32_t  lastFilteredValue,lastActivityValue,thresholdCorrection;
  bool     moving,baselineLocked;
  uint8_t  autoCalibrationMode;
  uint32_t activityTimestamp=0;

  FidFilter * filt_baseline;
  FidRun *run_baseline;
  FidFunc *func_baseline;
  void *fbuf_baseline;  

  FidFilter * filt_noise;
  FidRun *run_noise;
  FidFunc *func_noise;
  void *fbuf_noise;  

  FidFilter * filt_activity;
  FidRun *run_activity;
  FidFunc *func_activity;
  void *fbuf_activity;  

  int      sgn(int x);
  void     initFilters(uint8_t filterMask);
  void     freeFilters(uint8_t filterMask);
  
};
