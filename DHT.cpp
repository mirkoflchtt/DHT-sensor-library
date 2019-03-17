/* DHT library

MIT license
written by Adafruit Industries
*/

#include "DHT.h"
#include <Wire.h>

#define MIN_INTERVAL  (2000)

#define AM2320_ADDR         (0x5C)
#define AM2320_ADDR_START   (0x0)

#define ONE_WIRE_CHECKSUM(a, b, c, d) \
  ( ((uint32_t)(a)+(b)+(c)+(d)) & 0xFF )

#define DEBUG_DATA \
  DEBUG_PRINTLN(F("Received:")); \
  DEBUG_PRINT(data[0], HEX); DEBUG_PRINT(F(", ")); \
  DEBUG_PRINT(data[1], HEX); DEBUG_PRINT(F(", ")); \
  DEBUG_PRINT(data[2], HEX); DEBUG_PRINT(F(", ")); \
  DEBUG_PRINT(data[3], HEX); DEBUG_PRINT(F(", ")); \
  DEBUG_PRINT(data[4], HEX); DEBUG_PRINT(F(" =? ")); \
  DEBUG_PRINTLN(ONE_WIRE_CHECKSUM(data[0], data[1], data[2], data[3]), HEX);

static
uint16_t _crc16(const uint8_t* byte, uint32_t numBytes)
{
  uint16_t crc = 0xFFFF;          // 16-bit crc register

  for ( ; numBytes>0; numBytes--) {
    crc ^= *byte++;                   // exclusive-or crc with first byte

    for (int i = 0; i < 8; i++) {     // perform 8 shifts
      const uint16_t lsb = crc & 0x01;  // extract LSB from crc
      crc >>= 1;                      // shift be one position to the right

      if (lsb == 0) {                 // LSB is 0
        continue;                     // repete the process
      } else {                        // LSB is 1
        crc ^= 0xA001;                // exclusive-or with 1010 0000 0000 0001
      }
    }
  }

  return crc;
}


DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  (void)(count);
  _pin     = pin;
  _type    = type;
  _address = 0x0;
  #ifdef __AVR
    _bit = digitalPinToBitMask(pin);
    _port = digitalPinToPort(pin);
  #endif
  _maxcycles = microsecondsToClockCycles(1000);  // 1 millisecond timeout for
                                                 // reading pulses from DHT sensor.
  // Note that count is now ignored as the DHT reading algorithm adjusts itself
  // based on the speed of the processor.
}

void DHT::begin(bool oneWire) {
  if ( !oneWire ) {
    begin(0,0);
    return;
  }

  _address = 0x0;

  // set up the pins!
  pinMode(_pin, INPUT_PULLUP);
  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  _lastreadtime = -MIN_INTERVAL;
  DEBUG_PRINT("[OneWire] Max clock cycles: "); DEBUG_PRINTLN(_maxcycles, DEC);
}

void DHT::begin(uint8_t sda, uint8_t scl) {
  switch (_type) {
    case AM2320:
      _address = AM2320_ADDR;
      break;

    default:
      begin(true);
      return;
  }
  
  if ( sda==scl ) {
    Wire.begin();
  } else {
    Wire.begin(sda, scl);
  }
  _lastreadtime = -MIN_INTERVAL;
  DEBUG_PRINTLN("[TwoWire] Begin");
}

//boolean isFahrenheit: True == Fahrenheit; False == Celcius
float DHT::readTemperature(bool isFahrenheit, bool force) {
  float f = NAN;

  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if(isFahrenheit) {
        f = convertCtoF(f);
      }
      break;
    case DHT22:
    case DHT21:
    case AM2301:
    case AM2320:
      f = data[2] & 0x7F;
      f *= 256;
      f += data[3];
      f *= (data[2] & 0x80) ? -0.1 : 0.1;
      if(isFahrenheit) {
        f = convertCtoF(f);
      }
      break;
    }
  }
  return f;
}

float DHT::convertCtoF(float c) {
  return c * 1.8 + 32;
}

float DHT::convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

float DHT::readHumidity(bool force) {
  float f = NAN;

  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[0];
      break;
    case DHT22:
    case DHT21:
    case AM2301:
    case AM2320:
      f = data[0];
      f *= 256;
      f += data[1];
      f *= 0.1;
      break;
    }
  }
  return f;
}

//boolean isFahrenheit: True == Fahrenheit; False == Celcius
float DHT::computeHeatIndex(float temp, float percentHumidity, bool isFahrenheit) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (!isFahrenheit) {
    temp = convertCtoF(temp);
  }

  hi = 0.5 * (temp + 61.0 + ((temp - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temp +
            10.14333127 * percentHumidity +
            -0.22475541 * temp*percentHumidity +
            -0.00683783 * pow(temp, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temp, 2) * percentHumidity +
             0.00085282 * temp*pow(percentHumidity, 2) +
            -0.00000199 * pow(temp, 2) * pow(percentHumidity, 2);

    if ((percentHumidity < 13) && (temp >= 80.0) && (temp <= 112.0)) {
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temp - 95.0)) * 0.05882);
    }
    else if ((percentHumidity > 85.0) && (temp >= 80.0) && (temp <= 87.0)) {
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temp) * 0.2);
    }
  }

  return (isFahrenheit) ? hi : convertFtoC(hi);
}

bool DHT::read(bool force) {
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  const uint32_t currenttime = millis();
  if ( !force && (currenttime < (_lastreadtime+MIN_INTERVAL)) ) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  return (_address>0x0) ? readTwoWire() : readOneWire();
}

bool DHT::readOneWire(void) {
  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  digitalWrite(_pin, HIGH);
  delay(250);

  // First set data line low for 20 milliseconds.
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);

  uint16_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);

    // Now start reading the data line to get the value from the DHT sensor.
    pinMode(_pin, INPUT_PULLUP);
    delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectOneWirePulse(LOW) == 0) {
      DEBUG_PRINTLN(F("Timeout waiting for start signal low pulse."));
      _lastresult = false;
      return _lastresult;
    }
    if (expectOneWirePulse(HIGH) == 0) {
      DEBUG_PRINTLN(F("Timeout waiting for start signal high pulse."));
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) {
      cycles[i+0] = expectOneWirePulse(LOW);
      cycles[i+1] = expectOneWirePulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    const uint32_t lowCycles  = cycles[2*i+0];
    const uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      DEBUG_PRINTLN(F("Timeout waiting for pulse."));
      _lastresult = false;
      return _lastresult;
    }
    data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    // High cycles are greater than 50us low cycle count, must be a 1.
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
    data[i/8] |= (highCycles > lowCycles) ? 0x1 : 0x0;
  }

  DEBUG_DATA

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ONE_WIRE_CHECKSUM(data[0], data[1], data[2], data[3])) {
    _lastresult = true;
  }
  else {
    DEBUG_PRINTLN(F("Checksum failure!"));
    _lastresult = false;
  }
  return _lastresult;
}

bool DHT::readTwoWireRegisters(uint8_t startAddress, uint8_t numBytes) {
  // wake up sensor
  Wire.beginTransmission(_address);
  Wire.endTransmission();

  Wire.beginTransmission(_address);
  Wire.write(0x03);           // function code: 0x03 - read register data
  Wire.write(startAddress);   // begin address
  Wire.write(numBytes);       // number of bytes to read

  // send and check result if not success, return error code
  if (Wire.endTransmission(true) != 0) {        
    return false;                           // return sensor not ready code
  }
  delayMicroseconds(1500);                  // as specified in datasheet
  Wire.requestFrom(_address, (uint8_t)(numBytes+4));   // request bytes from sensor
                                            // see function code description in datasheet    
  for ( uint8_t i=0; i<numBytes+4; i++) {   // read
    data[i] = Wire.read();
  }

  return true;
}

bool DHT::readTwoWire(void) {
  if ( readTwoWireRegisters(AM2320_ADDR_START, 4) ) {
    const uint16_t receivedCrc = (((uint16_t)data[7]<<8) | data[6]);

    if ( receivedCrc == _crc16(data, 6) ) {
      // Remap humidity
      data[0] = data[2];
      data[1] = data[3];

      // Remap temperature
      data[2] = data[4];
      data[3] = data[5];

      // Compute OneWire protocol checksum
      data[4] = ONE_WIRE_CHECKSUM(data[0], data[1], data[2], data[3]);

      DEBUG_DATA

      _lastresult = true;
      return _lastresult;
    }
  }
  
  DEBUG_PRINTLN(F("Checksum failure!"));
  _lastresult = false;
  return _lastresult;
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t DHT::expectOneWirePulse(const uint8_t level) {
  uint32_t count = 0;
  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
  #ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ((*portInputRegister(_port) & _bit) == portState) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
  // right now, perhaps bugs in direct port access functions?).
  #else
    while (digitalRead(_pin) == level) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  #endif

  return count;
}

bool DHT::loop(void)
{
  return true;
}

#undef MIN_INTERVAL
#undef AM2320_ADDR
#undef AM2320_ADDR_START
#undef ONE_WIRE_CHECKSUM
#undef DEBUG_DATA
