/*
 * NAU7802_2chnMouse.ino
 * 
 * NAU+MPRLS readout benchmark test
 * using LoadcellSensor class for signal conditioning
 * 
 * HW-Setup: NAU7802 and MPRLS sensors connected to i2C/i2C1-interface of ArduinoNano2040Connect
 * 
 * Following serial commands are supported:
 *   'c': calibrate, reset 0 position (also triggered by connecting Pin 17 to GND)
 *   '1'-'4': show/hide timestamp,x,y,pressure
 */


#include <Adafruit_NAU7802.h>
#include "LoadcellSensor.h"

// uncomment Wire or Wire1 as needed:
// #define myWire Wire
#define myWire Wire1       

#define BUTTON_PIN      17
#define DRDY_PIN        28

#define MPRLS_ADDR 0x18
#define MPRLS_READ_TIMEOUT (20)     ///< millis
#define MPRLS_STATUS_POWERED (0x40) ///< Status SPI powered bit
#define MPRLS_STATUS_BUSY (0x20)    ///< Status busy bit
#define MPRLS_STATUS_FAILED (0x04)  ///< Status bit for integrity fail
#define MPRLS_STATUS_MATHSAT (0x01) ///< Status bit for math saturation
#define MPRLS_STATUS_MASK (0b01100101) ///< Sensor status mask: only these bits are set

#define MPRLS_DIVIDER 1
#define MPRLS_MID 0


Adafruit_NAU7802 nau;

LoadcellSensor X,Y,P;


uint8_t  chnMask=0b1111, channel, newData=0;
uint8_t calibNow=1;
uint32_t actTs,ts;
int32_t x=0,y=0,pressure=0;
uint32_t mprls_rawval=0;

void setup() {

  pinMode (BUTTON_PIN, INPUT_PULLUP);
  pinMode (DRDY_PIN, INPUT);
  Serial.begin(115200);

  while (!configureMPRLS()){
    Serial.println("Failed to find MPRS");
    delay(1000);
  }
  
  while (! nau.begin(&myWire)) {
    Serial.println("Failed to find NAU7802");
    delay(1000);
  }

  nau.setChannel(NAU7802_CHANNEL1);
  configureNAU();
  nau.setChannel(NAU7802_CHANNEL2);
  configureNAU();
  nau.setChannel(NAU7802_CHANNEL1);
  channel=1;

  attachInterrupt(digitalPinToInterrupt(DRDY_PIN), getValueISR, RISING);

  P.setGain(0.1);  // adjust gain for pressure sensor

  // take initial timestamp
  ts=micros();

}

void getValueISR() {
  static int32_t xChange=0,yChange=0;
  
  if (channel==1) {
      xChange=(X.process(nau.read())-x)/2;
      nau.setChannel(NAU7802_CHANNEL2);
      x+=xChange; y+=yChange;
      channel=2;
      getValueMPRLS();
      newData=1; 
  }
  else {
      yChange=(Y.process(nau.read())-y)/2;
      nau.setChannel(NAU7802_CHANNEL1);
      x+=xChange; y+=yChange;
      channel=1;
      getValueMPRLS();
      newData=1;
  }
}

void loop() {
  
  // button pressed -> trigger channel offset calibration
  if (digitalRead(BUTTON_PIN) == LOW) calibNow=1;
  
  // process serial commands 
  if (Serial.available()) {
    int c=Serial.read();
    if ((c>='1') && (c<='4')) chnMask ^= (1 << (c-'1'));  // enable/disable signal display)
    if (c=='c') { X.calib(); Y.calib(); P.calib(); }
  }

  if (newData) {
    newData=0;
    actTs=micros()-ts;
    ts=micros();      
    
    if (calibNow) {
      X.calib(); Y.calib(); P.calib();
      calibNow=0;
    }

    pressure = P.process(mprls_rawval) / MPRLS_DIVIDER + MPRLS_MID;
 
    // print values if desired
    if (chnMask) {
      if (chnMask&1) Serial.print (actTs); else  Serial.print("0"); Serial.print(" ");
      if (chnMask&2) Serial.print (constrain(x,-1000000,1000000)); else  Serial.print("0"); Serial.print(" ");
      if (chnMask&4) Serial.print (constrain(y,-1000000,1000000)); else  Serial.print("0"); Serial.print(" ");
      if (chnMask&8) Serial.print (pressure); else  Serial.print("0"); Serial.print(" ");      
      Serial.println(" ");
    }
  }  
}

void configureNAU() {
  nau.setLDO(NAU7802_3V0);   // NAU7802_2V7, NAU7802_2V4 
  nau.setGain(NAU7802_GAIN_128);  // NAU7802_GAIN_64, NAU7802_GAIN_32
  nau.setRate(NAU7802_RATE_320SPS);  // NAU7802_RATE_80SPS
  nau.setPGACap(NAU7802_CAP_OFF); //disable PGA capacitor on channel 2
  // trigger internal calibration 
  while (! nau.calibrate(NAU7802_CALMOD_INTERNAL)) {
    Serial.println("Failed to set internal calibration, retrying!");
    delay(1000);
  }
  
  // flush ADC
  for (uint8_t i=0; i<10; i++) {
    while (! nau.available()) delay(1);
    nau.read();
  }  
}

bool configureMPRLS() {
  myWire.begin();
  myWire.setClock(400000);  // use 400kHz I2C clock
  //detect if there is an MPRLS sensor connected to I2C (Wire)
  myWire.beginTransmission(MPRLS_ADDR);
  uint8_t result = myWire.endTransmission();
  //we found the MPRLS sensor, start the initialization
  if(result == 0) return true;
  return false;
}


void getValueMPRLS() {
  uint8_t buffer[4]  = {0};
  myWire.requestFrom(MPRLS_ADDR,1);
  buffer[0] = myWire.read();
  //any errors? set pressure value to 512, convert otherwise...
  if(buffer[0] & MPRLS_STATUS_BUSY)
  {
    //sensor is busy, cannot read data
    Serial.println("MPRLS: busy");
    return;
  }
  if((buffer[0] & MPRLS_STATUS_MATHSAT) || (buffer[0] & MPRLS_STATUS_FAILED))
  {
    //sensor failed or saturated
    Serial.println("MPRLS:failed or saturated");
    return;
  } else {
    //request all 4 bytes
    myWire.requestFrom(MPRLS_ADDR,4);
    for(uint8_t i = 0; i<4; i++) buffer[i] = myWire.read();
    mprls_rawval = (uint32_t(buffer[1]) << 16) | (uint32_t(buffer[2]) << 8) | (uint32_t(buffer[3]));
  }
  //trigger new conversion
  myWire.beginTransmission(MPRLS_ADDR);
  myWire.write(0xAA);
  myWire.write(0);
  myWire.write(0);
  myWire.endTransmission();
}  
  
