/**************************************************************************/
/*!
  @file    LoadcellSensor.cpp
  @mainpage LoadcellSensor data processing class
  @section intro Introduction
  
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
  offset=activity=lastFilteredValue=maxForce=compensationValue=0;
  overshootCompensationEnabled=autoCalibrationEnabled=true;
  activityTimestamp=0;
  bypassBaseline=0;
  moving=false;
  movementThreshold=MOVEMENT_THRESHOLD;
  idleDetectionThreshold=IDLE_DETECTION_THRESHOLD;
  idleDetectionPeriod=IDLE_DETECTION_PERIOD;
  compensationFactor=COMPENSATION_FACTOR;
  compensationDecay=COMPENSATION_DECAY;
  gain=GAIN;
  accu=0;
  calib();
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
  int32_t f=activityFilter(raw,afBuf);
  activity += abs(f-lastFilteredValue);  
  lastFilteredValue=f;

  // adjust raw values: offset and overshoot compensation
  raw-=offset;

  if (overshootCompensationEnabled && (compensationValue!=0)) {
    raw+=compensationValue;
    compensationValue*=compensationDecay;
  }

  // calculate filtered channel value
  filtered= noiseFilter(raw,nfBuf);

  // autocalibration / idle detection
  if (autoCalibrationEnabled) {
    if (millis()-activityTimestamp > idleDetectionPeriod) { 
      if (activity < idleDetectionThreshold) { 
        if (moving) calib(); 
      }
      clearActivity();
      activityTimestamp=millis();
    }
  }

  // handle baseline and movement
  if ((abs(filtered-baseline) <= movementThreshold + abs(compensationValue)) ||
     ((maxForce!=0) && (sgn(maxForce) != sgn(filtered-baseline)))) {
    moving=false; 
    if (maxForce!=0) {
      compensationValue+=maxForce*compensationFactor;
      maxForce=0;
    }
	if (!bypassBaseline)
      baseline=baselineFilter(raw,blBuf);  // feed new values for baseline
    else bypassBaseline--;
  }  

  if (abs(filtered-baseline) > movementThreshold + abs(compensationValue) ) {  // moving! hlastFilteredValue baselineX as it is!
    if (!moving) activity+=idleDetectionThreshold;
    moving=true;
	bypassBaseline=BYPASS_BASELINE;
    result=filtered-baseline; 
    if (abs(result) > abs(maxForce)) {
      maxForce=result;
    }
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
  memset(afBuf,0,sizeof(afBuf));
  memset(nfBuf,0,sizeof(nfBuf));
  memset(blBuf,0,sizeof(blBuf));
  baseline=baselineFilter(0,blBuf);
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
    @brief  sets overshoot compensation factor
    @param  compensationFactor  the compensation factor, multiplied with last max amplitude
*/
/**************************************************************************/
void LoadcellSensor::setCompensationFactor(double compensationFactor) {
  this->compensationFactor=compensationFactor;  
}

/**************************************************************************/
/*!
    @brief  sets overshoot compensation decay
    @param  compensationDecay  the compensation decay (0-1, close to 1 -> long decay)
*/
/**************************************************************************/
void LoadcellSensor::setCompensationDecay(double compensationDecay) {
  this->compensationDecay=compensationDecay;   
}


/**************************************************************************/
/*!
    @brief  enable/disable overshoot compensation
    @param  b true:enable, false:disable
*/
/**************************************************************************/
void LoadcellSensor::enableOvershootCompensation(bool b) {
  overshootCompensationEnabled=b;
}

/**************************************************************************/
/*!
    @brief  sets gain for incopming values
    @param  gain factor as float value
*/
/**************************************************************************/
void LoadcellSensor::setGain(double gain) {
  this->gain=gain;   
}


/**************************************************************************/
/*!
    @brief  enable/disable autocalibration
    @param  b true:enable, false:disable
*/
/**************************************************************************/
void LoadcellSensor::enableAutoCalibration(bool b) {
  autoCalibrationEnabled=b;
}

/**************************************************************************/
/*!
    @brief  returns if movemetn is currently active
    @return true if movement active, false otherwise    
*/
/**************************************************************************/
bool LoadcellSensor::isMoving(void) {
  return (moving);
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
  if (mask&1) Serial.print(constrain(raw,-limit,limit)); else Serial.print("0"); Serial.print(" ");
  if (mask&2) Serial.print(constrain(filtered,-limit,limit)); else Serial.print("0"); Serial.print(" ");
  if (mask&4) Serial.print(constrain(baseline,-limit,limit)); else Serial.print("0"); Serial.print(" ");
  if (overshootCompensationEnabled) Serial.print(constrain(compensationValue,-limit,limit));else Serial.print("0"); Serial.print(" ");
  if (autoCalibrationEnabled) Serial.print(constrain(activity,-limit,limit));else Serial.print("0"); Serial.print(" ");
}



/**************************************************************************/
/*!
    @brief  3Hz Lowpass Bessel, 2nd Order (./fiview 25 -i LpBe2/3)
    @param  val the next incoming signal value
    @param  buf a pointer to a buffer for working data (2 double values needed)
    @return  the filtered signal
*/
/**************************************************************************/
double LoadcellSensor::noiseFilter(double val, double * buf) {
   double tmp, fir, iir;
   tmp= buf[0]; memmove(buf, buf+1, 1*sizeof(double));
   val *= 0.1193072508536958;
   iir= val+0.7021409471770795*buf[0]-0.1793699505918626*tmp;
   fir= iir+buf[0]+buf[0]+tmp;
   buf[1]= iir; val= fir;
   return val;
}


/**************************************************************************/
/*!
    @brief  0.1Hz Lowpass Bessel, 2nd Order (./fiview 25 -i LpBe2/0.1)
    @param  val the next incoming signal value
    @param  buf a pointer to a buffer for working data (2 double values needed)
    @return  the filtered signal
*/
/**************************************************************************/
double LoadcellSensor::baselineFilter(double val, double * buf) {
   double tmp, fir, iir;
   tmp= buf[0]; memmove(buf, buf+1, 1*sizeof(double));
   val *= 0.0002485901688812353;
   iir= val+1.945135508892326*buf[0]-0.9461298695678511*tmp;
   fir= iir+buf[0]+buf[0]+tmp;
   buf[1]= iir; val= fir;
   return val;
}


/**************************************************************************/
/*!
    @brief  5Hz Lowpass Bessel, 2nd Order (./fiview 25 -i LpBe2/5)
    @param  val the next incoming signal value
    @param  buf a pointer to a buffer for working data (2 double values needed)
    @return  the filtered signal
*/
/**************************************************************************/
double LoadcellSensor::activityFilter(double val, double * buf) {
   double tmp, fir, iir;
   tmp= buf[0]; memmove(buf, buf+1, 1*sizeof(double));
   val *= 0.06378257264840968;
   iir= val+1.068354407019735*buf[0]-0.3234846976133736*tmp;
   fir= iir+buf[0]+buf[0]+tmp;
   buf[1]= iir; val= fir;
   return val;
}
