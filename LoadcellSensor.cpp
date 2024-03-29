/**************************************************************************/
/*!
  @file    LoadcellSensor.cpp
  @mainpage LoadcellSensor data processing class
  @section intro Introduction
  
  This is a library for processing raw values from a load cell or similar sensor
  for application in HCI applications (e.g. for mouse cursor control).

  The values are noise-filtered and compared to a baseline which adapts to slow drifting.
  Automatic calibration is supported (either by resetting the baseline or by apating movement thresholds).
  Note: the values should be fed into the LoadcellSensor::process() method periodically

  Thanks to Jim Peters for the marvellous fiview filter tool and the fidlib filter library:
  http://uazu.net/fiview/

  The AsTeRICS Foundation promotes Open Source Assistive Technologies, visit:
  https://www.asterics-foundation.org  

  @section author Author
  Chris Veigl (AsTeRICS Foundation)
  @section license License
  GPL (see license.txt)
*/
/**************************************************************************/

#include <Arduino.h>
#include "LoadcellSensor.h"


/**************************************************************************/
/*!
    @brief  Instantiates a new LoadcellSensor class, sets default values
*/
/**************************************************************************/
LoadcellSensor::LoadcellSensor () {
  offset=activity=thresholdCorrection=0;
  lastFilteredValue=lastActivityValue=0;
  autoCalibrationMode=AUTOCALIBRATION_RESET_BASELINE;
  activityTimestamp=0;
  moving=false;baselineLocked=false;
  movementThreshold=MOVEMENT_THRESHOLD;
  idleDetectionThreshold=IDLE_DETECTION_THRESHOLD;
  idleDetectionPeriod=IDLE_DETECTION_PERIOD;
  thresholdDecay=THRESHOLD_DECAY;
  gain=GAIN;
  sampleRate=SAMPLE_RATE;
  lpBaseline=LP_BASELINE;
  lpNoise=LP_NOISE;
  lpActivity=LP_ACTIVITY;
  
  initFilters(FILTERS_ALL); 
  calib();
  
}

LoadcellSensor::~LoadcellSensor () {
  freeFilters(FILTERS_ALL);
}

/**************************************************************************/
/*!
    @brief  processes the next raw data value, must be called periodically
    @param value Raw data value.
    @return processed movement estimation value.
*/
/**************************************************************************/
int32_t LoadcellSensor::process (int32_t value) {
  int32_t result=0;

  value*=gain;
  raw=value;
  int32_t a=func_activity(fbuf_activity,raw);
  activity += abs(a-lastActivityValue);  
  lastActivityValue=a;

  // remove offset from adjust raw values 
  raw-=offset;

  // calculate filtered channel value
  filtered= func_noise(fbuf_noise,raw);
  lastFilteredValue=filtered;

  // autocalibration / idle detection
  if (autoCalibrationMode && moving) { 
	if (millis()-activityTimestamp > idleDetectionPeriod) {
		// check if threshold or baseline should be adapted
		if (activity < idleDetectionThreshold) {
			switch (autoCalibrationMode) {
				case AUTOCALIBRATION_RESET_BASELINE:
						calib();
					break;					
				case AUTOCALIBRATION_ADAPT_THRESHOLD:
						thresholdCorrection += movementThreshold/2;  // increase movement threshold 
					break;
			}
		}
		clearActivity();
		activityTimestamp=millis();
	}
  }

  // handle baseline and movement
  int actThreshold = movementThreshold + abs(thresholdCorrection);
  if ((abs(filtered-baseline) <= actThreshold )) {
    moving=false; 

    if (!baselineLocked) 
        baseline= func_baseline(fbuf_baseline,raw);

    if (thresholdCorrection>0) 
	    thresholdCorrection*=thresholdDecay;
  }  

  if (abs(filtered-baseline) > actThreshold ) {  // moving! leave baseline as it is!
    if (!moving) {
		activityTimestamp=millis();
		activity+=idleDetectionThreshold;
		moving=true;
	}
    result=filtered-baseline; 
	if (result < 0) result += actThreshold; 
	else result -= actThreshold;
  }

  return(result);
}

/**************************************************************************/
/*!
    @brief  calculates the sign (signum) of an integer variable
    @param x the integer value
    @return the sign value of x (-1/0/1)
*/
/**************************************************************************/
int LoadcellSensor::sgn(int x) {
  return (x > 0) - (x < 0);
}

/**************************************************************************/
/*!
    @brief  initiates a calibration (offset is set to current filtered value)
*/
/**************************************************************************/
void LoadcellSensor::calib(void) {  // perform offset calibration
  fid_run_zapbuf(fbuf_baseline);
  fid_run_zapbuf(fbuf_noise);
  fid_run_zapbuf(fbuf_activity);
  baseline= func_baseline(fbuf_baseline, 0);

  offset+=filtered;
}

/**************************************************************************/
/*!
    @brief  returns the current accumulated activity value
    @return activity value
*/
/**************************************************************************/
int32_t LoadcellSensor::getActivity(void) {
  return (activity);
}

/**************************************************************************/
/*!
    @brief  sets the current accumulated activity value to 0
*/
/**************************************************************************/
void LoadcellSensor::clearActivity(void) {
  activity=0;
}


/**************************************************************************/
/*!
    @brief  sets movement threshold value
    @param  movementThreshold  the threshold value
*/
/**************************************************************************/
void LoadcellSensor::setMovementThreshold(int32_t movementThreshold) {
  this->movementThreshold=movementThreshold;
}

/**************************************************************************/
/*!
    @brief  sets idle detection threshold value
    @param  idleDetectionThreshold  the threshold value
*/
/**************************************************************************/
void LoadcellSensor::setIdleDetectionThreshold(int32_t idleDetectionThreshold) {
  this->idleDetectionThreshold=idleDetectionThreshold;  
}

/**************************************************************************/
/*!
    @brief  sets idle detection period
    @param  idleDetectionPeriod  the measurement period (in milliseconds)
*/
/**************************************************************************/
void LoadcellSensor::setIdleDetectionPeriod(int32_t idleDetectionPeriod) {
  this->idleDetectionPeriod=idleDetectionPeriod;  
}

/**************************************************************************/
/*!
    @brief  sets overshoot threshold decay
    @param  thresholdDecay the threshold decay (0-1, close to 1 -> long decay)
*/
/**************************************************************************/
void LoadcellSensor::setThresholdDecay(double thresholdDecay) {
  this->thresholdDecay=thresholdDecay;   
}


/**************************************************************************/
/*!
    @brief  sets gain for incoming values
    @param  gain factor
*/
/**************************************************************************/
void LoadcellSensor::setGain(double gain) {
  this->gain=gain;   
}


/**************************************************************************/
/*!
    @brief  sets sampling rate for signal processing / filtering
    @param  sampleRate: sampling rate (Hz)
*/
/**************************************************************************/
void LoadcellSensor::setSampleRate(double sampleRate) {
  this->sampleRate=sampleRate;
  freeFilters(FILTERS_ALL);
  initFilters(FILTERS_ALL);
}

/**************************************************************************/
/*!
    @brief  sets low pass filter for baseline adjustment
    @param  lpBaseline: low pass cutoff frequency
*/
/**************************************************************************/
void LoadcellSensor::setBaselineLowpass(double lpBaseline) {
  if (this->lpBaseline!=lpBaseline) {
    this->lpBaseline=lpBaseline;   
    freeFilters(FILTER_BASELINE);
    initFilters(FILTER_BASELINE);
  }
}

/**************************************************************************/
/*!
    @brief  sets low pass filter for noise removal from signal
    @param  lpNoise: low pass cutoff frequency
*/
/**************************************************************************/
void LoadcellSensor::setNoiseLowpass(double lpNoise) {
  if (this->lpNoise!=lpNoise) {
    this->lpNoise=lpNoise;   
    freeFilters(FILTER_NOISE);
    initFilters(FILTER_NOISE);
  }
}

/**************************************************************************/
/*!
    @brief  sets low pass filter for activity measurement
    @param  lpActivity: low pass cutoff frequency
*/
/**************************************************************************/
void LoadcellSensor::setActivityLowpass(double lpActivity) {
  if (this->lpActivity!=lpActivity) {
    this->lpActivity=lpActivity;   
    freeFilters(FILTER_ACTIVITY);
    initFilters(FILTER_ACTIVITY);
  }
}


/**************************************************************************/
/*!
    @brief  set autocalibration mode
    @param  m: mode
*/
/**************************************************************************/
void LoadcellSensor::setAutoCalibrationMode(uint8_t m) {
  autoCalibrationMode=m;
}

/**************************************************************************/
/*!
    @brief  returns if movement is currently active
    @return true if movement active, false otherwise    
*/
/**************************************************************************/
bool LoadcellSensor::isMoving(void) {
  return (moving);
}

/**************************************************************************/
/*!
    @brief  sets the lockmode for baseline updates
    @return none
*/
/**************************************************************************/
void LoadcellSensor::lockBaseline(bool b) {
  baselineLocked=b;
}


/**************************************************************************/
/*!
    @brief  prints current signal values to Serial window (plotter)
    @param  mask mask for signal selection, bit0: raw signal, bit1:filtered signal
            bit2: baseline signal
    @param  limit constrain for signal plotter (e.g 1000 limits signal to -1000/1000)
*/
/**************************************************************************/
void LoadcellSensor::printValues(uint8_t mask, int32_t limit) {
  if (mask&1) Serial.print(constrain(raw,-limit,limit)); else Serial.print("0"); 
  Serial.print(" ");
  if (mask&2) Serial.print(constrain(filtered,-limit,limit)); else Serial.print("0");
  Serial.print(" ");
  if (mask&4) { 
    Serial.print(constrain(baseline,-limit,limit));
    Serial.print(" ");
	Serial.print(constrain(baseline-movementThreshold-thresholdCorrection,-limit,limit));
	Serial.print(" ");
	Serial.print(constrain(baseline+movementThreshold+thresholdCorrection,-limit,limit));
  }  else Serial.print("0 0 0");
  Serial.print(" ");
  // if (autoCalibrationEnabled) Serial.print(constrain(activity,-limit,limit)); else Serial.print("0");
  // Serial.print(" ");
}

/**************************************************************************/
/*!
    @brief  initialise filters for actual sampling rate and cutoff frequencies
    @param  filterMask: a binary mask byte, identifying the filter(s)
*/
/**************************************************************************/

void LoadcellSensor::initFilters(uint8_t filterMask) 
{
  if (filterMask & FILTER_BASELINE) {
	  filt_baseline=fid_design((char *)"LpBe2", sampleRate, lpBaseline, 0, 0, 0);
	  run_baseline= fid_run_new(filt_baseline, &func_baseline);
	  fbuf_baseline= fid_run_newbuf(run_baseline);
  }
  if (filterMask & FILTER_NOISE) {
	  filt_noise=fid_design((char *)"LpBe2", sampleRate, lpNoise, 0, 0, 0);
	  run_noise= fid_run_new(filt_noise, &func_noise);
	  fbuf_noise= fid_run_newbuf(run_noise);
  }
  if (filterMask & FILTER_ACTIVITY) {
	  filt_activity=fid_design((char *)"LpBe2", sampleRate, lpActivity, 0, 0, 0);
	  run_activity= fid_run_new(filt_activity, &func_activity);
	  fbuf_activity= fid_run_newbuf(run_activity);
  }
}

/**************************************************************************/
/*!
    @brief  free filters and allocated buffers
    @param  filterMask: a binary mask byte, identifying the filter(s)
*/
/**************************************************************************/
void LoadcellSensor::freeFilters(uint8_t filterMask)
{
  if (filterMask & FILTER_BASELINE) {
	  fid_run_freebuf(run_baseline);
	  fid_run_free(filt_baseline);
  }
  if (filterMask & FILTER_NOISE) {
	  fid_run_freebuf(run_noise);
	  fid_run_free(filt_noise);
  }
  if (filterMask & FILTER_ACTIVITY) {
	  fid_run_freebuf(run_activity);
	  fid_run_free(filt_activity);
  }
}

