/**************************************************************************/
/**
  @file   LoadcellSensor.h
  Author: Chris Veigl (Asterics Foundation)
  License: GPL (see license.txt)

  This is a library for processing raw values from a load cell or similar sensor
  for application in HCI applications (e.g. for mouse cursor control).

  The values are noise-filtered and compared to a baseline which adapts to slow drifting.
  Overshoot compensation and automatic calibration are supported.
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

// default frequency settings
#define SAMPLE_RATE 25       // sampling rate: 25Hz
#define LP_BASELINE 0.1      // low pass cutoff frequency for baseline calculation (Hz)
#define LP_NOISE    3        // low pass cutoff frequency for denoising the signal (Hz)
#define LP_ACTIVITY 2        // low pass cutoff frequency for activity/idle detection (Hz)

// default signal conditioning parameters
#define GAIN                        1.00   // gain for incoming values
#define MOVEMENT_THRESHOLD          1000   // deflection from baseline which indicates a valid movement (gain normalized)
#define COMPENSATION_DECAY          0.95   // overshoot compensation time (close to 1 -> slower decay)
#define COMPENSATION_FACTOR         0.05   // overshoot compensation amplitude (* max amplitude)
#define IDLE_DETECTION_THRESHOLD    3000   // noise theshold value for auto calibration (gain normalized)
#define IDLE_DETECTION_PERIOD       1000   // in milliseconds
#define BYPASS_BASELINE             10     // bypass baseline calculation n times after a movement (avoid drift)
#define STARTUP_TIME                200    // return 0 for signal result until startup time passed, to let signal settle

// other signal processing settings (fixed)
#define MAXIMUM_GRADIENT_NOMOVEMENT 500   // the maximum signal gradient for updating the baseline when not moving (gain normalized)
#define MINIMUM_COMPENSATION_VALUE  400   // the minimum compensation of movement threshold after a movement (gain normalized)
#define BASELINE_ADAPTIVE_FEEDRATE    5   // baseline lowpass filter adaption (higher feed rate after a movement)
#define BYPASS_BASELINE_AFTER_MOVEMENT 1  // bypass baseline adjustment for n samples after a movement)

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
  void     setCompensationFactor(double compensationFactor);
  void     setCompensationDecay(double compensationDecay);
  void     setGain(double gain);
  void     setStartupTime(uint32_t ms);
  void     setSampleRate(double sampleRate);
  void     setBaselineLowpass(double lpBaseline);
  void     setNoiseLowpass(double lpNoise);
  void     setActivityLowpass(double lpActivity);
  void     enableOvershootCompensation(bool b);
  void     enableAutoCalibration(bool b);
  bool     isMoving(void);
  void     lockBaseline(bool b);
  void     printValues(uint8_t mask, int32_t limit);


private:

  int32_t  movementThreshold;
  int32_t  idleDetectionThreshold,idleDetectionPeriod;
  double   compensationDecay,compensationFactor;
  double   gain;
  double   sampleRate,lpBaseline,lpNoise,lpActivity;
  
  int32_t  raw,filtered,activity,baseline,offset,bypassBaseline,gradient;
  int32_t  lastFilteredValue,lastActivityValue,maxForce,compensationValue;
  bool     moving,starting,overshootCompensationEnabled,autoCalibrationEnabled,baselineLocked;
  uint32_t activityTimestamp=0,startupTimestamp,startupTime,feedRate;

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
