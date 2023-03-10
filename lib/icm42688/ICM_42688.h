/*

A C++ interface to the ICM-42688

*/

#ifndef _ICM_42688_H_
#define _ICM_42688_H_

#include "ICM_42688_C.h" // The C backbone. ICM_42688_USE_DMP is defined in here.
#include "Arduino.h" // Arduino support
#include "Wire.h"
#include "SPI.h"

#define ICM_42688_ARD_UNUSED_PIN 0xFF

// Base
class ICM_42688
{
private:
  Stream *_debugSerial;     //The stream to send debug messages to if enabled
  bool _printDebug = true; //Flag to print the serial commands we are sending to the Serial port for debug

  const uint8_t MAX_MAGNETOMETER_STARTS = 10; // This replaces maxTries

protected:
  ICM_42688_Device_t _device;

  float getTempC(int16_t val);
  float getGyrDPS(int16_t axis_val);
  float getAccMG(int16_t axis_val);
  float getMagUT(int16_t axis_val);

public:
  ICM_42688(); // Constructor

// Enable debug messages using the chosen Serial port (Stream)
// Boards like the RedBoard Turbo use SerialUSB (not Serial).
// But other boards like the SAMD51 Thing Plus use Serial (not SerialUSB).
// These lines let the code compile cleanly on as many SAMD boards as possible.
#if defined(ARDUINO_ARCH_SAMD) // Is this a SAMD board?
#if defined(USB_VID) // Is the USB Vendor ID defined?
#if (USB_VID == 0x1B4F) // Is this a SparkFun board?
#if !defined(ARDUINO_SAMD51_THING_PLUS) & !defined(ARDUINO_SAMD51_MICROMOD) // If it is not a SAMD51 Thing Plus or SAMD51 MicroMod
  void enableDebugging(Stream &debugPort = SerialUSB); //Given a port to print to, enable debug messages.
#else
  void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages.
#endif
#else
  void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages.
#endif
#else
  void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages.
#endif
#else
  void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages.
#endif

  void disableDebugging(void); //Turn off debug statements

  void debugPrintStatus(ICM_42688_Status_e stat);

  // gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809
  void debugPrint(const char *);
  void debugPrint(const __FlashStringHelper *);
  void debugPrintln(const char *);
  void debugPrintln(const __FlashStringHelper *);
  void doDebugPrint(char (*)(const char *), const char *, bool newLine = false);

  void debugPrintf(int i);
  void debugPrintf(float f);

  ICM_42688_agt_t agt;          // Acceleometer, Gyroscope, Magenetometer, and Temperature data
  ICM_42688_agt_t getAGT(void); // Updates the agt field in the object and also returns a copy directly

  float magX(void); // micro teslas
  float magY(void); // micro teslas
  float magZ(void); // micro teslas

  float accX(void); // milli g's
  float accY(void); // milli g's
  float accZ(void); // milli g's

  float gyrX(void); // degrees per second
  float gyrY(void); // degrees per second
  float gyrZ(void); // degrees per second

  float temp(void); // degrees celsius

  ICM_42688_Status_e status;                                              // Status from latest operation
  const char *statusString(ICM_42688_Status_e stat = ICM_42688_Stat_NUM); // Returns a human-readable status message. Defaults to status member, but prints string for supplied status if supplied

  // Device Level
  ICM_42688_Status_e setBank(uint8_t bank);                                // Sets the bank
  ICM_42688_Status_e swReset(void);                                        // Performs a SW reset
  ICM_42688_Status_e sleep(bool on = false);                               // Set sleep mode for the chip
  ICM_42688_Status_e checkID(void);                                        // Return 'ICM_42688_Stat_Ok' if whoami matches ICM_42688_WHOAMI
  ICM_42688_Status_e SetI2C(void);                                         // Sets relevant registers to start I2C communication


  bool dataReady(void);    // Returns 'true' if data is ready
  uint8_t getWhoAmI(void); // Return whoami in out prarmeter
  bool isConnected(void);  // Returns true if communications with the device are sucessful

  // Internal Sensor Options
  ICM_42688_Status_e setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
  ICM_42688_Status_e setFullScale(uint8_t sensor_id_bm);

  // Interrupts on INT and FSYNC Pins
  ICM_42688_Status_e clearInterrupts(void);

  ICM_42688_Status_e cfgIntActiveLow(bool active_low);
  ICM_42688_Status_e cfgIntOpenDrain(bool open_drain);
  ICM_42688_Status_e cfgIntLatch(bool latching);         // If not latching then the interrupt is a 50 us pulse
  ICM_42688_Status_e cfgIntAnyReadToClear(bool enabled); // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first
  ICM_42688_Status_e cfgFsyncActiveLow(bool active_low);
  ICM_42688_Status_e cfgFsyncIntMode(bool interrupt_mode); // Can use FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

  ICM_42688_Status_e intEnableI2C(bool enable);
  ICM_42688_Status_e intEnablePLL(bool enable);
  ICM_42688_Status_e intEnableWOM(bool enable);
  ICM_42688_Status_e intEnableWOF(bool enable);
  ICM_42688_Status_e intEnableRawDataReady(bool enable);
  ICM_42688_Status_e intEnableOverflowFIFO(uint8_t bm_enable);
  ICM_42688_Status_e intEnableWatermarkFIFO(uint8_t bm_enable);

  ICM_42688_Status_e WOMThreshold(uint8_t threshold);

  // Interface Options
  ICM_42688_Status_e i2cMasterPassthrough(bool passthrough = true);
  ICM_42688_Status_e i2cMasterEnable(bool enable = true);
  ICM_42688_Status_e i2cMasterReset();

  //Used for configuring peripherals 0-3
  ICM_42688_Status_e i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false, uint8_t dataOut = 0);
  ICM_42688_Status_e i2cControllerPeriph4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr = true);

  //Provided for backward-compatibility only. Please update to i2cControllerConfigurePeripheral and i2cControllerPeriph4Transaction.
  //https://www.oshwa.org/2020/06/29/a-resolution-to-redefine-spi-pin-names/
  ICM_42688_Status_e i2cMasterConfigureSlave(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false);
  ICM_42688_Status_e i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr = true);

  // Default Setup
  ICM_42688_Status_e startupDefault(); 

  // direct read/write
  ICM_42688_Status_e read(uint8_t reg, uint8_t *pdata, uint32_t len);
  ICM_42688_Status_e write(uint8_t reg, uint8_t *pdata, uint32_t len);

  //FIFO
  ICM_42688_Status_e enableFIFO(bool enable = true);
  ICM_42688_Status_e setupFIFO(bool enable = true);
  ICM_42688_Status_e resetFIFO(void);
  ICM_42688_Status_e setFIFOmode(bool snapshot = false); // Default to Stream (non-Snapshot) mode
  ICM_42688_Status_e getFIFOcount(uint16_t *count);
  ICM_42688_Status_e readFIFO(uint8_t *data, uint8_t len = 1);
  ICM_42688_Status_e readData(icm_42688_data_t *data);


  ICM_42688_Status_e setGyroSF(unsigned char div, int gyro_level);
};

// I2C

// Forward declarations of TwoWire and Wire for board/variant combinations that don't have a default 'SPI'
//class TwoWire; // Commented by PaulZC 21/2/8 - this was causing compilation to fail on the Arduino NANO 33 BLE
//extern TwoWire Wire; // Commented by PaulZC 21/2/8 - this was causing compilation to fail on the Arduino NANO 33 BLE

class ICM_42688_I2C : public ICM_42688
{
private:
protected:
public:
  TwoWire *_i2c;
  uint8_t _addr;
  uint8_t _ad0;
  bool _ad0val;
  ICM_42688_Serif_t _serif;

  ICM_42688_I2C(); // Constructor

  virtual ICM_42688_Status_e begin(TwoWire &wirePort = Wire, bool ad0val = true, uint8_t ad0pin = ICM_42688_ARD_UNUSED_PIN);
};

// SPI
#define ICM_42688_SPI_DEFAULT_FREQ 4000000
#define ICM_42688_SPI_DEFAULT_ORDER MSBFIRST
#define ICM_42688_SPI_DEFAULT_MODE SPI_MODE0

// Forward declarations of SPIClass and SPI for board/variant combinations that don't have a default 'SPI'
//class SPIClass; // Commented by PaulZC 21/2/8 - this was causing compilation to fail on the Arduino NANO 33 BLE
//extern SPIClass SPI; // Commented by PaulZC 21/2/8 - this was causing compilation to fail on the Arduino NANO 33 BLE

class ICM_42688_SPI : public ICM_42688
{
private:
protected:
public:
  SPIClass *_spi;
  SPISettings _spisettings;
  uint8_t _cs;
  ICM_42688_Serif_t _serif;

  ICM_42688_SPI(); // Constructor

  ICM_42688_Status_e begin(uint8_t csPin, SPIClass &spiPort = SPI, uint32_t SPIFreq = ICM_42688_SPI_DEFAULT_FREQ);
};

#endif /* _ICM_42688_H_ */
