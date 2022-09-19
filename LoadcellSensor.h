/**************************************************************************/
/**
  @file   LoadcellSensor.h
  Author: Chris Veigl (Asterics Foundation)
  License: GPL (see license.txt)

  This is a library for processing raw values from a load cell or similar sensor
  for application in HCI applications (e.g. for mouse cursor control).

  The values are noise-filtered and compared to a baseline which adapts to slow drifting.
  Overshoot compensation and automatic calibration are supported.
  Note: currently the values should be fed into the LoadcellSensor::process() method periodically (ca. 25Hz)
  TBD: flexible update periods, flexible filter paramters

  Thanks to Jim Peters for the marvellous fiview filter tool and the fidlib filter library:
  http://uazu.net/fiview/

  The AsTeRICS Foundation promotes Open Source Assistive Technologies, visit:
  https://www.asterics-foundation.org
  
*/
/**************************************************************************/

#define MOVEMENT_THRESHOLD          1000   // deflection from baseline which indicates a valid movement
#define COMPENSATION_DECAY           0.9   // overshoot compensation time (close to 1 -> slower decay)
#define COMPENSATION_FACTOR         0.05   // overshoot compensation amplitude (* max amplitude)

#define IDLE_DETECTION_THRESHOLD    3000   // noise theshold value for auto calibration
#define IDLE_DETECTION_PERIOD       1000   // in milliseconds


/**************************************************************************/
/*!
    @brief  LoadcellSensor class
*/
/**************************************************************************/

class LoadcellSensor {
public:
  LoadcellSensor();
  void     calib(void);
  int32_t  process(int32_t value);
  void     clearActivity(void);
  int32_t  getActivity(void);
  void     setMovementThreshold(int32_t movementThreshold);
  void     setIdleDetectionThreshold(int32_t idleDetectionThreshold);
  void     setIdleDetectionPeriod(int32_t idleDetectionPeriod);
  void     setCompensationFactor(double compensationFactor);
  void     setCompensationDecay(double compensationDecay);
  void     enableOvershootCompensation(bool b);
  void     enableAutoCalibration(bool b);
  bool     isMoving(void);
  void     printValues(uint8_t mask, int32_t limit);


private:

  int32_t  movementThreshold;
  int32_t  idleDetectionThreshold,idleDetectionPeriod;
  double   compensationDecay,compensationFactor;
  
  int32_t  raw,filtered,baseline,offset;
  int32_t  activity,lastFilteredValue,maxForce,compensationValue;
  bool     moving,overshootCompensationEnabled,autoCalibrationEnabled;
  double   accu,afBuf[2],nfBuf[2],blBuf[2];
  uint32_t activityTimestamp=0;

  int      sgn(int x);
  double   baselineFilter(register double val, double * buf);
  double   noiseFilter(register double val, double * buf);
  double   activityFilter(register double val, double * buf);
  
};
