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

#define MOVEMENT_THRESHOLD          1000   // deflection from baseline which indicates a valid movement
#define COMPENSATION_DECAY          0.95   // overshoot compensation time (close to 1 -> slower decay)
#define COMPENSATION_FACTOR         0.05   // overshoot compensation amplitude (* max amplitude)

#define GAIN                        1.00   // gain for incoming values

#define IDLE_DETECTION_THRESHOLD    3000   // noise theshold value for auto calibration
#define IDLE_DETECTION_PERIOD       1000   // in milliseconds

#define BYPASS_BASELINE             10     // bypass baseline calculation n times after a movement (avoid drift)

#define SAMPLE_RATE 25
#define LP_BASELINE 0.1
#define LP_NOISE    3
#define LP_ACTIVITY 2


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
  void     setSampleRate(double sampleRate);
  void     setBaselineLowpass(double lpBaseline);
  void     setNoiseLowpass(double lpNoise);
  void     setActivityLowpass(double lpActivity);
  void     enableOvershootCompensation(bool b);
  void     enableAutoCalibration(bool b);
  bool     isMoving(void);
  void     printValues(uint8_t mask, int32_t limit);


private:

  int32_t  movementThreshold;
  int32_t  idleDetectionThreshold,idleDetectionPeriod;
  double   compensationDecay,compensationFactor;
  double   gain;
  double   sampleRate,lpBaseline,lpNoise,lpActivity;
  
  int32_t  raw,filtered,baseline,offset,bypassBaseline;
  int32_t  activity,lastFilteredValue,maxForce,compensationValue;
  bool     moving,overshootCompensationEnabled,autoCalibrationEnabled;
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
  void     initFilters();
  void     freeFilters();
  
};
