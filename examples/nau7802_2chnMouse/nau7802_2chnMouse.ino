/*
 * NAU7802_2chnMouse.ino
 * 
 * Testbench for LoadcellSensor class using a NAU7802 2-channel bridge sensor amplifier
 * HW-Setup: NAU7802 connected to i2C-interface of ArduinoNano2040Connect
 * Active movement is indicated via internal LED
 * 
 * Following serial commands are supported:
 *   'm': enable/disable mouse cursor movement
 *   'a': set autocalibration mode (0: disabled, 1: reset baseline, 2: adapt thresholds)
 *   'c': calibrate, reset 0 position (also triggered by connecting Pin 17 to GND)
 *   'l': enable/disable limit of values for serial plotter
 *   '1'-'3': show/hide channel1 (X-) signals
 *   '4'-'6': show/hide channel2 (Y-) signals
 */


#include <Adafruit_NAU7802.h>
#include <Mouse.h>
#include "LoadcellSensor.h"

// uncomment Wire or Wire1 as needed:
// #define myWire Wire
#define myWire Wire1   

#define FLUSH_ADC_VALUES 0          // fortunately even the first value after a channel change is valid!
#define BUTTON_PIN      17
#define LED_PIN         LED_BUILTIN

#define SLOWDOWN_FACTOR 800         // for cursor movment (bigger value -> slower movement)
#define PLOT_LIMIT 60000            // limit for serial plotter values (to avoid display re-scale)

Adafruit_NAU7802 nau;
LoadcellSensor X,Y;

int32_t  mx=0,my=0;
double   accuX,accuY;
int32_t  plotLimit=PLOT_LIMIT;
uint8_t  autoCalibrationMode=AUTOCALIBRATION_RESET_BASELINE;
uint8_t  mouseEnabled=0;
uint8_t  chnMask=0b111111;

void setup() {

  pinMode (BUTTON_PIN, INPUT_PULLUP);
  pinMode (LED_PIN, OUTPUT);
  delay(10);
  
  Serial.begin(115200);
  while (! nau.begin(&myWire)) {
    Serial.println("Failed to find NAU7802");
    delay(1000);
  }

  Serial.println("rawX filteredX BaselineX compX autocalibX rawY filteredY BaselineY compY autocalibY");   
  configureNAU();
  nau.setChannel(NAU7802_CHANNEL2);
  configureNAU();

  /* 1K SMD thick film resistor sensorboard settings */
  /*
      X.setGain(-1.0);                   Y.setGain(-1.5);
      X.setMovementThreshold(4000);      Y.setMovementThreshold(6000);
      X.setIdleDetectionPeriod(200);     Y.setIdleDetectionPeriod(200);
      X.setIdleDetectionThreshold(500);  Y.setIdleDetectionThreshold(750);
      X.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      Y.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
   */

  /* strain gauge sensorboard settings */
   X.setGain(0.1); Y.setGain(0.1);
   X.setThresholdDecay(0.97); Y.setThresholdDecay(0.97);
   X.setMovementThreshold(900); Y.setMovementThreshold(900);
   X.setIdleDetectionPeriod(200); Y.setIdleDetectionPeriod(200);
   X.setIdleDetectionThreshold(120); Y.setIdleDetectionThreshold(120);
   X.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
   Y.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE);
}


void loop() {
  
  // button pressed -> trigger calibration
  if (digitalRead(BUTTON_PIN) == LOW) { 
    X.calib(); Y.calib();
  }

  // process serial commands 
  if (Serial.available()) {
    int c=Serial.read();
    if ((c>='1') && (c<='6')) chnMask ^= (1 << (c-'1'));  // enable/disable signal display)
    if (c=='m') mouseEnabled=!mouseEnabled;
    if (c=='l') plotLimit^=(1<<24);   
    if (c=='c') { X.calib(); Y.calib(); }
    if (c=='a') { autoCalibrationMode++; 
                  if (autoCalibrationMode>=3) autoCalibrationMode=0;
                  X.setAutoCalibrationMode(autoCalibrationMode); 
                  Y.setAutoCalibrationMode(autoCalibrationMode); }
  }

  // read raw ADC values 
  nau.setChannel(NAU7802_CHANNEL1);
  for (int i=0;i<=FLUSH_ADC_VALUES;i++) {
    while (! nau.available()) delayMicroseconds(200);
    mx=X.process(nau.read());
  }
  nau.setChannel(NAU7802_CHANNEL2);
  for (int i=0;i<=FLUSH_ADC_VALUES;i++) {
    while (! nau.available()) delayMicroseconds(200);
    my=Y.process(nau.read());
  }
  
  // perform mouse cursor movement
  accuX += (double)mx / SLOWDOWN_FACTOR;
  mx = constrain((int)accuX,-128,127);
  accuX -= mx;

  accuY += (double)my / SLOWDOWN_FACTOR;
  my = constrain((int)accuY,-128,127);
  accuY -= my;

  if (((mx!=0) || (my!=0)) && mouseEnabled )  Mouse.move (mx,my);
 
  // indicate activity
  if (X.isMoving() || Y.isMoving()) digitalWrite(LED_PIN, HIGH);
  else digitalWrite(LED_PIN, LOW);

  // output signals to serial plotter
  X.printValues(chnMask, plotLimit);
  Y.printValues(chnMask>>3, plotLimit);
  Serial.println(" ");
}

void configureNAU() {
  nau.setLDO(NAU7802_3V0);
  /*
  Serial.print("LDO voltage set to ");
  switch (nau.getLDO()) {
    case NAU7802_4V5:  Serial.println("4.5V"); break;
    case NAU7802_4V2:  Serial.println("4.2V"); break;
    case NAU7802_3V9:  Serial.println("3.9V"); break;
    case NAU7802_3V6:  Serial.println("3.6V"); break;
    case NAU7802_3V3:  Serial.println("3.3V"); break;
    case NAU7802_3V0:  Serial.println("3.0V"); break;
    case NAU7802_2V7:  Serial.println("2.7V"); break;
    case NAU7802_2V4:  Serial.println("2.4V"); break;
    case NAU7802_EXTERNAL:  Serial.println("External"); break;
  }*/

  nau.setGain(NAU7802_GAIN_128);
  /*
  Serial.print("Gain set to ");
  switch (nau.getGain()) {
    case NAU7802_GAIN_1:  Serial.println("1x"); break;
    case NAU7802_GAIN_2:  Serial.println("2x"); break;
    case NAU7802_GAIN_4:  Serial.println("4x"); break;
    case NAU7802_GAIN_8:  Serial.println("8x"); break;
    case NAU7802_GAIN_16:  Serial.println("16x"); break;
    case NAU7802_GAIN_32:  Serial.println("32x"); break;
    case NAU7802_GAIN_64:  Serial.println("64x"); break;
    case NAU7802_GAIN_128:  Serial.println("128x"); break;
  }*/

  nau.setRate(NAU7802_RATE_320SPS);
  /*
  Serial.print("Conversion rate set to ");
  switch (nau.getRate()) {
    case NAU7802_RATE_10SPS:  Serial.println("10 SPS"); break;
    case NAU7802_RATE_20SPS:  Serial.println("20 SPS"); break;
    case NAU7802_RATE_40SPS:  Serial.println("40 SPS"); break;
    case NAU7802_RATE_80SPS:  Serial.println("80 SPS"); break;
    case NAU7802_RATE_320SPS:  Serial.println("320 SPS"); break;
  }
  */

  nau.setPGACap(NAU7802_CAP_OFF); //disable PGA capacitor on channel 2
  
  // Take 10 readings to flush out readings
  for (uint8_t i=0; i<40; i++) {
    while (! nau.available()) delay(1);
    nau.read();
  }  

  while (! nau.calibrate(NAU7802_CALMOD_INTERNAL)) {
    Serial.println("Failed to calibrate internal offset, retrying!");
    delay(1000);
  }
  // Serial.println("Calibrated internal offset");

  while (! nau.calibrate(NAU7802_CALMOD_OFFSET)) {
    Serial.println("Failed to calibrate system offset, retrying!");
    delay(1000);
  }
  // Serial.println("Calibrated system offset");

  // Take readings to flush out readings
  for (uint8_t i=0; i<40; i++) {
    while (! nau.available()) delay(1);
    nau.read();
  }  
}