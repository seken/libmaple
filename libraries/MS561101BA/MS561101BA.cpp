/*
MS5611-01BA.cpp - Interfaces a Measurement Specialities MS5611-01BA with Arduino
See http://www.meas-spec.com/downloads/MS5611-01BA01.pdf for the device datasheet

Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "MS561101BA.h"
#define EXTRA_PRECISION 5 // trick to add more precision to the pressure and temp readings
#define CONVERSION_TIME 10000l // conversion time in microseconds

#include <i2c.h>
MS561101BA::MS561101BA() {
  ;
}

void MS561101BA::init(uint8_t address) {  
  _addr =  address;
  
  reset(); // reset the device to populate its internal PROM registers
  delay(1000); // some safety time 
  readPROM(); // reads the PROM into object variables for later use
}

float MS561101BA::getPressure(uint8_t OSR) {
  // see datasheet page 7 for formulas
  int64_t dT   = getDeltaTemp(OSR);
  int32_t rawPress = rawPressure(OSR);
  if(dT == NULL) {
    return NULL;
  }
  int64_t off  = (((int64_t)_C[1]) << 16) + ((_C[3] * dT) >> 7);
  int64_t sens = (((int64_t)_C[0]) << 15) + ((_C[2] * dT) >> 8);
  if(rawPress != NULL) {
    return ((((rawPress * sens) >> 21) - off) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION) * 100.0);
  }
  else {
    return NULL;
  }
}

float MS561101BA::getTemperature(uint8_t OSR) {
  // see datasheet page 7 for formulas
  int64_t dT = getDeltaTemp(OSR);
  
  if(dT != NULL) {
    return ((1<<EXTRA_PRECISION)*2000l + ((dT * _C[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION) * 100.0);
  }
  else {
    return NULL;
  }
}

int64_t MS561101BA::getDeltaTemp(uint8_t OSR) {
  int32_t rawTemp = rawTemperature(OSR);
  if(rawTemp != NULL) {
    return rawTemp - (((int32_t)_C[4]) << 8);
  }
  else {
    return NULL;
  }
}

int32_t MS561101BA::rawPressure(uint8_t OSR) {
  unsigned long now = micros();
  if(lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME) {
    lastPresConv = 0;
    return getConversion(MS561101BA_D1 + OSR);
  }
  else {
    if(lastPresConv == 0 && lastTempConv == 0) {
      startConversion(MS561101BA_D1 + OSR);
      lastPresConv = now;
    }
    return NULL;
  }
}

int32_t MS561101BA::rawTemperature(uint8_t OSR) {
  unsigned long now = micros();
  if(lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME) {
    lastTempConv = 0;
    tempCache = getConversion(MS561101BA_D2 + OSR);
    return tempCache;
  }
  else {
    if(lastTempConv == 0 && lastPresConv == 0) {
      startConversion(MS561101BA_D2 + OSR);
      lastTempConv = now;
    }
    else if(lastPresConv != 0) { // there is a Pressure reading in process
      return tempCache;
    }
	
	return NULL;
  }
}


// see page 11 of the datasheet
void MS561101BA::startConversion(uint8_t command) {
  // initialize pressure conversion
  i2c_msg tmp;
  tmp.addr = _addr;
  tmp.flags = 0;
  tmp.length = 1;
  tmp.data = &command;
  i2c_master_xfer(I2C1, &tmp, 1, 0);
  while (I2C1->state == I2C_STATE_BUSY) {}
}

unsigned long MS561101BA::getConversion(uint8_t command) {
  unsigned long conversion = 0;
  
  i2c_msg tmp[2];
  tmp[0].addr = _addr;
  tmp[0].flags = 0;
  tmp[0].length = 1;
  uint8_t cmd = 0;
  tmp[0].data = &cmd;

  tmp[1].addr = _addr;
  tmp[1].flags = I2C_MSG_READ;
  tmp[1].length = 3;
  uint8_t data[3] = {0, 0, 0};
  tmp[1].data = &data[0];

  i2c_master_xfer(I2C1, &tmp[0], 2, 0);
  
  while (I2C1->state == I2C_STATE_BUSY) {}

  conversion = data[0] * 65536 + data[1] * 256 + data[2];

  return conversion;
}


/**
 * Reads factory calibration and store it into object variables.
*/
int MS561101BA::readPROM() {
  for (int i=0;i<MS561101BA_PROM_REG_COUNT;i++) {
	i2c_msg tmp[2];
	tmp[0].addr = _addr;
	tmp[0].flags = 0;
	tmp[0].length = 1;
	uint8_t cmd = MS561101BA_PROM_BASE_ADDR + (i*MS561101BA_PROM_REG_SIZE);
	tmp[0].data = &cmd;

	tmp[1].addr = _addr;
	tmp[1].flags = I2C_MSG_READ;
	tmp[1].length = MS561101BA_PROM_REG_SIZE;
	uint8_t data[2] = {0, 0};
	tmp[1].data = &data[0];

	i2c_master_xfer(I2C1, &tmp[0], 2, 0);
	while (I2C1->state == I2C_STATE_BUSY) {}
	_C[i] = data[0] << 8 | data[1];
  }

  return 0;
	  /*
    Wire.beginTransmission(_addr);
    Wire.send(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    Wire.endTransmission();
    
    Wire.beginTransmission(_addr);
    Wire.requestFrom(_addr, (uint8_t) MS561101BA_PROM_REG_SIZE);
    if(Wire.available()) {
      _C[i] = Wire.receive() << 8 | Wire.receive();
      
      //DEBUG_PRINT(_C[i]);
    }
    else {
      return -1; // error reading the PROM or communicating with the device
    }
  }*/
  return 0;
}


/**
 * Send a reset command to the device. With the reset command the device
 * populates its internal registers with the values read from the PROM.
*/
void MS561101BA::reset() {
  i2c_msg tmp;
  tmp.addr = _addr;
  tmp.flags = 0;
  tmp.length = 1;
  uint8_t cmd = MS561101BA_RESET;
  tmp.data = &cmd;
  i2c_master_xfer(I2C1, &tmp, 1, 0);
  while (I2C1->state == I2C_STATE_BUSY) {}
}
