/* DHT library

MIT license
written by Adafruit Industries
*/
#ifndef DHT_H
#define DHT_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif


// Uncomment to enable printing out nice debug messages.
//#define DHT_DEBUG

// Define where debug output will be printed.
#define DEBUG_PRINTER Serial

// Setup debug printing macros.
#ifdef DHT_DEBUG
  #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {}
  #define DEBUG_PRINTLN(...) {}
#endif

// Define types of sensors.
#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 31
#define AM2320 32

#define MAX_BYTES_READ 8

class DHT {
public:
  DHT(uint8_t pin, uint8_t type, uint8_t count=6);

  void begin(bool oneWire=true);
  void begin(uint8_t sda, uint8_t scl);

  float readTemperature(bool isFahrenheit=false, bool force=false);
  float convertCtoF(float);
  float convertFtoC(float);
  float computeHeatIndex(float temp, float percentHumidity, bool isFahrenheit=true);
  float readHumidity(bool force=false);

  bool  loop(void);

private:
  uint8_t data[MAX_BYTES_READ];
  uint8_t _pin;
  uint8_t _type;
  uint8_t _address;

  #ifdef __AVR
    // Use direct GPIO access on an 8-bit AVR so keep track of the port and bitmask
    // for the digital pin connected to the DHT.  Other platforms will use digitalRead.
    uint8_t _bit, _port;
  #endif
  uint32_t _lastreadtime, _maxcycles;
  bool _lastresult;

  bool  read(bool force=false);
  bool  readOneWire(void);
  bool  readTwoWire(void);

  bool  readTwoWireRegisters(uint8_t startAddress, uint8_t numBytes);

  uint32_t expectPulse(const bool level);
};

class InterruptLock {
  public:
   InterruptLock() {
    noInterrupts();
   }
   ~InterruptLock() {
    interrupts();
   }

};

#endif
