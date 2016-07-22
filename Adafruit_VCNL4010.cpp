#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif

#include "Adafruit_VCNL4010.h"

/**************************************************************************/
/*! 
    @brief  Instantiates a new VCNL4010 class
*/
/**************************************************************************/
Adafruit_VCNL4010::Adafruit_VCNL4010() {
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
boolean Adafruit_VCNL4010::begin(uint8_t addr) {
  _i2caddr = addr;
  Wire.begin();

  uint8_t rev = read8(VCNL4010_PRODUCTID);
  //Serial.println(rev, HEX);
  if ((rev & 0xF0) != 0x20) {
    return false;
  }
  
  setLEDcurrent(20);
  setFrequency(VCNL4010_390K625);

  write8(VCNL4010_INTCONTROL, 0x08);
  return true;
}

boolean Adafruit_VCNL4010::dataReadyInterruptEnabled(void) {
  uint8_t result = read8(VCNL4010_INTCONTROL);
  return result & VCNL4010_INT_PROX_READY;
}


void Adafruit_VCNL4010::enableDataReadyInterrupt(boolean enable) {
  uint8_t flag = read8(VCNL4010_INTCONTROL);
  if (enable) {
    write8(VCNL4010_INTCONTROL, flag | VCNL4010_INT_PROX_READY);
  } else {
    write8(VCNL4010_INTCONTROL, flag & 0xF0);
  }
  Serial.printf("DRDY en: %d\n", read8(VCNL4010_INTCONTROL));
}

boolean Adafruit_VCNL4010::continuousMeasurementsEnabled(void) {
  uint8_t result = read8(VCNL4010_COMMAND);
  return result & VCNL4010_MEASUREPROXIMITYPERIODIC;
}

void Adafruit_VCNL4010::enableContinuousMeasurements(boolean enable) {
  if (enable) {
    write8(VCNL4010_COMMAND, VCNL4010_MEASUREPROXIMITYPERIODIC);
  } else {
    write8(VCNL4010_COMMAND, 0x00);
  }
  Serial.printf("Cont Meas. enabled: %d\n", read8(VCNL4010_COMMAND) & VCNL4010_MEASUREPROXIMITYPERIODIC);
}

uint8_t Adafruit_VCNL4010::getCmdReg(void) {
  return read8(VCNL4010_COMMAND);
}

uint8_t Adafruit_VCNL4010::getIntReg(void) {
  return read8(VCNL4010_INTCONTROL);
}

void Adafruit_VCNL4010::readAllRegs(void) {
  uint8_t result[15];
  Serial.println(read8(VCNL4010_COMMAND), HEX);
  Serial.println(read8(VCNL4010_PRODUCTID), HEX);
  Serial.println(read8(VCNL4010_PROXRATE), HEX);
  Serial.println(read8(VCNL4010_IRLED), HEX);
  Serial.println(read8(VCNL4010_AMBIENTPARAMETER), HEX);
  Serial.println(read8(VCNL4010_AMBIENTDATA), HEX);
  Serial.println(read8(VCNL4010_AMBIENTDATA+1), HEX);
  Serial.println(read8(VCNL4010_PROXIMITYDATA), HEX);
  Serial.println(read8(VCNL4010_PROXIMITYDATA+1), HEX);
  Serial.println(read8(VCNL4010_INTCONTROL), HEX);
  Serial.println(read8(VCNL4010_PROXINITYADJUST), HEX);
  Serial.println(read8(VCNL4010_PROXINITYADJUST+1), HEX);
  Serial.println(read8(VCNL4010_HIGHTHRESHOLD_HB), HEX);
  Serial.println(read8(VCNL4010_HIGHTHRESHOLD_LB), HEX);
  Serial.println(read8(VCNL4010_INTSTAT), HEX);
  Serial.println(read8(VCNL4010_MODTIMING), HEX);
}

void Adafruit_VCNL4010::setProxRate(uint8_t rate) {
  if (rate > VCNL4010_PROXRATE256)
    rate = VCNL4010_PROXRATE256;
  Serial.println(rate);
  write8(VCNL4010_PROXRATE, rate);
  Serial.printf("Proxrate: %d\n", read8(VCNL4010_PROXRATE));
}

/**************************************************************************/
/*! 
    @brief  Get and set the LED current draw
*/
/**************************************************************************/

void Adafruit_VCNL4010::setLEDcurrent(uint8_t c) {
  if (c > 20) c = 20;
  write8(VCNL4010_IRLED, c);
}

uint8_t Adafruit_VCNL4010::getLEDcurrent(void) {
  return read8(VCNL4010_IRLED);
}

/**************************************************************************/
/*! 
    @brief  Get and set the measurement signal frequency
*/
/**************************************************************************/

void Adafruit_VCNL4010::setFrequency(vcnl4010_freq f) {
  uint8_t r =  read8(VCNL4010_MODTIMING);
  r &= ~(0b00011000);
  r |= f << 3;
  write8(VCNL4010_MODTIMING, r);
}


/**************************************************************************/
/*! 
    @brief  Get proximity measurement
*/
/**************************************************************************/

uint16_t  Adafruit_VCNL4010::readProximity(void) {
  uint8_t i = read8(VCNL4010_INTSTAT);
  i &= ~0x80;
  write8(VCNL4010_INTSTAT, i);

  write8(VCNL4010_COMMAND, VCNL4010_MEASUREPROXIMITY);
  while (1) {
    //Serial.println(read8(VCNL4010_INTSTAT), HEX);
    uint8_t result = read8(VCNL4010_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & VCNL4010_PROXIMITYREADY) {
      return read16(VCNL4010_PROXIMITYDATA);
    }
    delay(1);
  }
}

uint16_t  Adafruit_VCNL4010::readAmbient(void) {
  uint8_t i = read8(VCNL4010_INTSTAT);
  i &= ~0x40;
  write8(VCNL4010_INTSTAT, i);


  write8(VCNL4010_COMMAND, VCNL4010_MEASUREAMBIENT);
  while (1) {
    //Serial.println(read8(VCNL4010_INTSTAT), HEX);
    uint8_t result = read8(VCNL4010_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & VCNL4010_AMBIENTREADY) {
      return read16(VCNL4010_AMBIENTDATA);
    }
    delay(1);
  }
}

/**************************************************************************/
/*! 
    @brief  I2C low level interfacing
*/
/**************************************************************************/


// Read 1 byte from the VCNL4000 at 'address'
uint8_t Adafruit_VCNL4010::read8(uint8_t address)
{
  uint8_t data;

  Wire.beginTransmission(_i2caddr);
#if ARDUINO >= 100
  Wire.write(address);
#else
  Wire.send(address);
#endif
  Wire.endTransmission();

  delayMicroseconds(170);  // delay required

  Wire.requestFrom(_i2caddr, (uint8_t)1);
  while(!Wire.available());

#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}


// Read 2 byte from the VCNL4000 at 'address'
uint16_t Adafruit_VCNL4010::read16(uint8_t address)
{
  uint16_t data;

  Wire.beginTransmission(_i2caddr);
#if ARDUINO >= 100
  Wire.write(address);
#else
  Wire.send(address);
#endif
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, (uint8_t)2);
  while(!Wire.available());
#if ARDUINO >= 100
  data = Wire.read();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.read();
#else
  data = Wire.receive();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.receive();
#endif
  
  return data;
}

// write 1 byte
void Adafruit_VCNL4010::write8(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(_i2caddr);
#if ARDUINO >= 100
  Wire.write(address);
  Wire.write(data);  
#else
  Wire.send(address);
  Wire.send(data);  
#endif
  Wire.endTransmission();
}
