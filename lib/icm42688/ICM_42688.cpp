#include "ICM_42688.h"

#include "ICM_42688_REGISTERS.h"

// Forward Declarations
ICM_42688_Status_e ICM_42688_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_42688_Status_e ICM_42688_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
ICM_42688_Status_e ICM_42688_write_SPI(uint8_t reg, uint8_t *buff, uint32_t len, void *user);
ICM_42688_Status_e ICM_42688_read_SPI(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

// Base
ICM_42688::ICM_42688()
{
  status = ICM_42688_init_struct(&_device);
}

void ICM_42688::enableDebugging(Stream &debugPort)
{
  _debugSerial = &debugPort; //Grab which port the user wants us to use for debugging
  _printDebug = true;        //Should we print the commands we send? Good for debugging
}
void ICM_42688::disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

// Debug Printing: based on gfvalvo's flash string helper code:
// https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809

void ICM_42688::debugPrint(const char *line)
{
  doDebugPrint([](const char *ptr) { return *ptr; }, line);
}

void ICM_42688::debugPrint(const __FlashStringHelper *line)
{
  doDebugPrint([](const char *ptr) { return (char)pgm_read_byte_near(ptr); },
               (const char *)line);
}

void ICM_42688::debugPrintln(const char *line)
{
  doDebugPrint([](const char *ptr) { return *ptr; }, line, true);
}

void ICM_42688::debugPrintln(const __FlashStringHelper *line)
{
  doDebugPrint([](const char *ptr) { return (char)pgm_read_byte_near(ptr); },
               (const char *)line, true);
}

void ICM_42688::doDebugPrint(char (*funct)(const char *), const char *string, bool newLine)
{
  if (_printDebug == false)
    return; // Bail if debugging is not enabled

  char ch;

  while ((ch = funct(string++)))
  {
    _debugSerial->print(ch);
  }

  if (newLine)
  {
    _debugSerial->println();
  }
}

void ICM_42688::debugPrintf(int i)
{
  if (_printDebug == true)
    _debugSerial->print(i);
}

void ICM_42688::debugPrintf(float f)
{
  if (_printDebug == true)
    _debugSerial->print(f);
}

void ICM_42688::debugPrintStatus(ICM_42688_Status_e stat)
{
  switch (stat)
  {
  case ICM_42688_Stat_Ok:
    debugPrint(F("All is well."));
    break;
  case ICM_42688_Stat_Err:
    debugPrint(F("General Error"));
    break;
  case ICM_42688_Stat_NotImpl:
    debugPrint(F("Not Implemented"));
    break;
  case ICM_42688_Stat_ParamErr:
    debugPrint(F("Parameter Error"));
    break;
  case ICM_42688_Stat_WrongID:
    debugPrint(F("Wrong ID"));
    break;
  case ICM_42688_Stat_InvalSensor:
    debugPrint(F("Invalid Sensor"));
    break;
  case ICM_42688_Stat_NoData:
    debugPrint(F("Data Underflow"));
    break;
  case ICM_42688_Stat_SensorNotSupported:
    debugPrint(F("Sensor Not Supported"));
    break;
  case ICM_42688_Stat_DMPNotSupported:
    debugPrint(F("DMP Firmware Not Supported. Is #define ICM_42688_USE_DMP commented in util/ICM_42688_C.h?"));
    break;
  case ICM_42688_Stat_DMPVerifyFail:
    debugPrint(F("DMP Firmware Verification Failed"));
    break;
  case ICM_42688_Stat_FIFONoDataAvail:
    debugPrint(F("No FIFO Data Available"));
    break;
  case ICM_42688_Stat_FIFOIncompleteData:
    debugPrint(F("DMP data in FIFO was incomplete"));
    break;
  case ICM_42688_Stat_FIFOMoreDataAvail:
    debugPrint(F("More FIFO Data Available"));
    break;
  case ICM_42688_Stat_UnrecognisedDMPHeader:
    debugPrint(F("Unrecognised DMP Header"));
    break;
  case ICM_42688_Stat_UnrecognisedDMPHeader2:
    debugPrint(F("Unrecognised DMP Header2"));
    break;
  case ICM_42688_Stat_InvalDMPRegister:
    debugPrint(F("Invalid DMP Register"));
    break;
  default:
    debugPrint(F("Unknown Status"));
    break;
  }
}

ICM_42688_agt_t ICM_42688::getAGT(void)
{
  status = ICM_42688_get_agt(&_device, &agt);

  return agt;
}

float ICM_42688::accX(void)
{
  return getAccMG(agt.acc.axes.x);
}

float ICM_42688::accY(void)
{
  return getAccMG(agt.acc.axes.y);
}

float ICM_42688::accZ(void)
{
  return getAccMG(agt.acc.axes.z);
}

float ICM_42688::gyrX(void)
{
  return getGyrDPS(agt.gyr.axes.x);
}

float ICM_42688::gyrY(void)
{
  return getGyrDPS(agt.gyr.axes.y);
}

float ICM_42688::gyrZ(void)
{
  return getGyrDPS(agt.gyr.axes.z);
}

float ICM_42688::getGyrDPS(int16_t axis_val)
{
  switch (agt.fss.g)
  {
  case 0:
    return (((float)axis_val) / 131);
    break;
  case 1:
    return (((float)axis_val) / 65.5);
    break;
  case 2:
    return (((float)axis_val) / 32.8);
    break;
  case 3:
    return (((float)axis_val) / 16.4);
    break;
  default:
    return 0;
    break;
  }
}

float ICM_42688::temp(void)
{
  return getTempC(agt.tmp.val);
}

float ICM_42688::getTempC(int16_t val)
{
  return (((float)val - 21) / 333.87) + 21;
}

const char *ICM_42688::statusString(ICM_42688_Status_e stat)
{
  ICM_42688_Status_e val;
  if (stat == ICM_42688_Stat_NUM)
  {
    val = status;
  }
  else
  {
    val = stat;
  }

  switch (val)
  {
  case ICM_42688_Stat_Ok:
    return "All is well.";
    break;
  case ICM_42688_Stat_Err:
    return "General Error";
    break;
  case ICM_42688_Stat_NotImpl:
    return "Not Implemented";
    break;
  case ICM_42688_Stat_ParamErr:
    return "Parameter Error";
    break;
  case ICM_42688_Stat_WrongID:
    return "Wrong ID";
    break;
  case ICM_42688_Stat_InvalSensor:
    return "Invalid Sensor";
    break;
  case ICM_42688_Stat_NoData:
    return "Data Underflow";
    break;
  case ICM_42688_Stat_SensorNotSupported:
    return "Sensor Not Supported";
    break;
  case ICM_42688_Stat_DMPNotSupported:
    return "DMP Firmware Not Supported. Is #define ICM_42688_USE_DMP commented in util/ICM_42688_C.h?";
    break;
  case ICM_42688_Stat_DMPVerifyFail:
    return "DMP Firmware Verification Failed";
    break;
  case ICM_42688_Stat_FIFONoDataAvail:
    return "No FIFO Data Available";
    break;
  case ICM_42688_Stat_FIFOIncompleteData:
    return "DMP data in FIFO was incomplete";
    break;
  case ICM_42688_Stat_FIFOMoreDataAvail:
    return "More FIFO Data Available";
    break;
  case ICM_42688_Stat_UnrecognisedDMPHeader:
    return "Unrecognised DMP Header";
    break;
  case ICM_42688_Stat_UnrecognisedDMPHeader2:
    return "Unrecognised DMP Header2";
    break;
  case ICM_42688_Stat_InvalDMPRegister:
    return "Invalid DMP Register";
    break;
  default:
    return "Unknown Status";
    break;
  }
  return "None";
}

// Device Level
ICM_42688_Status_e ICM_42688::setBank(uint8_t bank)
{
  status = ICM_42688_set_bank(&_device, bank);
  return status;
}

ICM_42688_Status_e ICM_42688::swReset(void)
{
  status = ICM_42688_sw_reset(&_device);
  return status;
}

ICM_42688_Status_e ICM_42688::sleep(bool on)
{
  status = ICM_42688_sleep(&_device, on);
  return status;
} 

ICM_42688_Status_e ICM_42688::SetI2C(void)
{
  status = ICM_42688_set_I2C_mode(&_device);
  if (status != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688::checkID: ICM_42688_check_id returned: "));
    debugPrintStatus(status);
    debugPrintln(F(""));
  }
  return status;
}

ICM_42688_Status_e ICM_42688::checkID(void)
{
  status = ICM_42688_check_id(&_device);
  if (status != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688::checkID: ICM_42688_check_id returned: "));
    debugPrintStatus(status);
    debugPrintln(F(""));
  }
  return status;
}

bool ICM_42688::dataReady(void)
{
  status = ICM_42688_data_ready(&_device);
  if (status == ICM_42688_Stat_Ok)
  {
    return true;
  }
  return false;
}

uint8_t ICM_42688::getWhoAmI(void)
{
  uint8_t retval = 0x00;
  status = ICM_42688_get_who_am_i(&_device, &retval);
  return retval;
}

bool ICM_42688::isConnected(void)
{
  status = checkID();
  if (status == ICM_42688_Stat_Ok)
  {
    return true;
  }
  debugPrint(F("ICM_42688::isConnected: checkID returned: "));
  debugPrintStatus(status);
  debugPrintln(F(""));
  return false;
}

// Internal Sensor Options
ICM_42688_Status_e ICM_42688::setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode)
{
  //status = ICM_42688_set_sample_mode(&_device, (ICM_42688_InternalSensorID_bm)sensor_id_bm, (ICM_42688_LP_CONFIG_CYCLE_e)lp_config_cycle_mode);
  delay(1); // Give the ICM42688 time to change the sample mode (see issue #8)
  return status;
}

ICM_42688_Status_e ICM_42688::setFullScale(uint8_t sensor_id_bm)
{
  status = ICM_42688_set_full_scale(&_device, (ICM_42688_InternalSensorID_bm)sensor_id_bm);
  return status;
}



// Interrupts on INT Pin
/*
ICM_42688_Status_e ICM_42688::clearInterrupts(void)
{
  ICM_42688_INT_STATUS_t int_stat;
  ICM_42688_INT_STATUS_1_t int_stat_1;

  // read to clear interrupts
  status = ICM_42688_set_bank(&_device, 0);
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  status = ICM_42688_execute_r(&_device, AGB0_REG_INT_STATUS, (uint8_t *)&int_stat, sizeof(ICM_42688_INT_STATUS_t));
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  status = ICM_42688_execute_r(&_device, AGB0_REG_INT_STATUS_1, (uint8_t *)&int_stat_1, sizeof(ICM_42688_INT_STATUS_1_t));
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }

  // todo: there may be additional interrupts that need to be cleared, like FIFO overflow/watermark

  return status;
}*/
/*
ICM_42688_Status_e ICM_42688::cfgIntActiveLow(bool active_low)
{
  ICM_42688_INT_CONFIG_t reg;
  status = ICM_42688_int_pin_cfg(&_device, NULL, &reg); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  reg.INT2_POLARITY = active_low;                       // set the setting
  status = ICM_42688_int_pin_cfg(&_device, &reg, NULL); // write phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::cfgIntOpenDrain(bool open_drain)
{
  ICM_42688_INT_PIN_CFG_t reg;
  status = ICM_42688_int_pin_cfg(&_device, NULL, &reg); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  reg.INT1_OPEN = open_drain;                           // set the setting
  status = ICM_42688_int_pin_cfg(&_device, &reg, NULL); // write phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::cfgIntLatch(bool latching)
{
  ICM_42688_INT_PIN_CFG_t reg;
  status = ICM_42688_int_pin_cfg(&_device, NULL, &reg); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  reg.INT1_LATCH_EN = latching;                         // set the setting
  status = ICM_42688_int_pin_cfg(&_device, &reg, NULL); // write phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::cfgIntAnyReadToClear(bool enabled)
{
  ICM_42688_INT_PIN_CFG_t reg;
  status = ICM_42688_int_pin_cfg(&_device, NULL, &reg); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  reg.INT_ANYRD_2CLEAR = enabled;                       // set the setting
  status = ICM_42688_int_pin_cfg(&_device, &reg, NULL); // write phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::cfgFsyncActiveLow(bool active_low)
{
  ICM_42688_INT_PIN_CFG_t reg;
  status = ICM_42688_int_pin_cfg(&_device, NULL, &reg); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  reg.ACTL_FSYNC = active_low;                          // set the setting
  status = ICM_42688_int_pin_cfg(&_device, &reg, NULL); // write phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::cfgFsyncIntMode(bool interrupt_mode)
{
  ICM_42688_INT_PIN_CFG_t reg;
  status = ICM_42688_int_pin_cfg(&_device, NULL, &reg); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  reg.FSYNC_INT_MODE_EN = interrupt_mode;               // set the setting
  status = ICM_42688_int_pin_cfg(&_device, &reg, NULL); // write phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  return status;
}
*/
//      All these individual functions will use a read->set->write method to leave other settings untouched
ICM_42688_Status_e ICM_42688::intEnableI2C(bool enable)
{
  ICM_42688_INT_enable_t en;                          // storage
  status = ICM_42688_int_enable(&_device, NULL, &en); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  en.I2C_MST_INT_EN = enable;                        // change the setting
  status = ICM_42688_int_enable(&_device, &en, &en); // write phase w/ readback
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  if (en.I2C_MST_INT_EN != enable)
  {
    status = ICM_42688_Stat_Err;
    return status;
  }
  return status;
}


ICM_42688_Status_e ICM_42688::intEnablePLL(bool enable)
{
  ICM_42688_INT_enable_t en;                          // storage
  status = ICM_42688_int_enable(&_device, NULL, &en); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  en.PLL_RDY_EN = enable;                            // change the setting
  status = ICM_42688_int_enable(&_device, &en, &en); // write phase w/ readback
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  if (en.PLL_RDY_EN != enable)
  {
    status = ICM_42688_Stat_Err;
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::intEnableWOM(bool enable)
{
  ICM_42688_INT_enable_t en;                          // storage
  status = ICM_42688_int_enable(&_device, NULL, &en); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  en.WOM_INT_EN = enable;                            // change the setting
  status = ICM_42688_int_enable(&_device, &en, &en); // write phase w/ readback
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  if (en.WOM_INT_EN != enable)
  {
    status = ICM_42688_Stat_Err;
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::intEnableWOF(bool enable)
{
  ICM_42688_INT_enable_t en;                          // storage
  status = ICM_42688_int_enable(&_device, NULL, &en); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  en.REG_WOF_EN = enable;                            // change the setting
  status = ICM_42688_int_enable(&_device, &en, &en); // write phase w/ readback
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  if (en.REG_WOF_EN != enable)
  {
    status = ICM_42688_Stat_Err;
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::intEnableRawDataReady(bool enable)
{
  ICM_42688_INT_enable_t en;                          // storage
  status = ICM_42688_int_enable(&_device, NULL, &en); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  en.RAW_DATA_0_RDY_EN = enable;                     // change the setting
  status = ICM_42688_int_enable(&_device, &en, &en); // write phase w/ readback
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  if (en.RAW_DATA_0_RDY_EN != enable)
  {
    status = ICM_42688_Stat_Err;
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::intEnableOverflowFIFO(uint8_t bm_enable)
{
  ICM_42688_INT_enable_t en;                          // storage
  status = ICM_42688_int_enable(&_device, NULL, &en); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  en.FIFO_OVERFLOW_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
  en.FIFO_OVERFLOW_EN_1 = ((bm_enable >> 1) & 0x01);
  en.FIFO_OVERFLOW_EN_2 = ((bm_enable >> 2) & 0x01);
  en.FIFO_OVERFLOW_EN_3 = ((bm_enable >> 3) & 0x01);
  en.FIFO_OVERFLOW_EN_4 = ((bm_enable >> 4) & 0x01);
  status = ICM_42688_int_enable(&_device, &en, &en); // write phase w/ readback
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  return status;
}

ICM_42688_Status_e ICM_42688::intEnableWatermarkFIFO(uint8_t bm_enable)
{
  ICM_42688_INT_enable_t en;                          // storage
  status = ICM_42688_int_enable(&_device, NULL, &en); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  en.FIFO_WM_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
  en.FIFO_WM_EN_1 = ((bm_enable >> 1) & 0x01);
  en.FIFO_WM_EN_2 = ((bm_enable >> 2) & 0x01);
  en.FIFO_WM_EN_3 = ((bm_enable >> 3) & 0x01);
  en.FIFO_WM_EN_4 = ((bm_enable >> 4) & 0x01);
  status = ICM_42688_int_enable(&_device, &en, &en); // write phase w/ readback
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  return status;
}
/*
ICM_42688_Status_e ICM_42688::WOMThreshold(uint8_t threshold)
{
  ICM_42688_ACCEL_WOM_THR_t thr;                          // storage
  status = ICM_42688_wom_threshold(&_device, NULL, &thr); // read phase
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  thr.WOM_THRESHOLD = threshold;                          // change the setting
  status = ICM_42688_wom_threshold(&_device, &thr, &thr); // write phase w/ readback
  if (status != ICM_42688_Stat_Ok)
  {
    return status;
  }
  if (thr.WOM_THRESHOLD != threshold)
  {
    status = ICM_42688_Stat_Err;
    return status;
  }
  return status;
}
*/
// Interface Options
ICM_42688_Status_e ICM_42688::i2cMasterPassthrough(bool passthrough)
{
  status = ICM_42688_i2c_master_passthrough(&_device, passthrough);
  return status;
}

ICM_42688_Status_e ICM_42688::i2cMasterEnable(bool enable)
{
  status = ICM_42688_i2c_master_enable(&_device, enable);
  return status;
}

ICM_42688_Status_e ICM_42688::i2cMasterReset()
{
  status = ICM_42688_i2c_master_reset(&_device);
  return status;
}

ICM_42688_Status_e ICM_42688::i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  status = ICM_42688_i2c_controller_configure_peripheral(&_device, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, dataOut);
  return status;
}

//Provided for backward-compatibility only. Please update to i2cControllerConfigurePeripheral and i2cControllerPeriph4Transaction.
//https://www.oshwa.org/2020/06/29/a-resolution-to-redefine-spi-pin-names/
ICM_42688_Status_e ICM_42688::i2cMasterConfigureSlave(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap)
{
  return (i2cControllerConfigurePeripheral(peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, 0));
}

ICM_42688_Status_e ICM_42688::i2cControllerPeriph4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  status = ICM_42688_i2c_controller_periph4_txn(&_device, addr, reg, data, len, Rw, send_reg_addr);
  return status;
}

//Provided for backward-compatibility only. Please update to i2cControllerConfigurePeripheral and i2cControllerPeriph4Transaction.
//https://www.oshwa.org/2020/06/29/a-resolution-to-redefine-spi-pin-names/
ICM_42688_Status_e ICM_42688::i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  return (i2cControllerPeriph4Transaction(addr, reg, data, len, Rw, send_reg_addr));
}


ICM_42688_Status_e ICM_42688::startupDefault()
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;

  // wait for chip to power up fully
  delay(3000);

  retval = SetI2C();
  if (retval != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688::startupDefault: SetI2C returned: "));
    debugPrintStatus(retval);
    debugPrintln(F(""));
    status = retval;
    return status;
  }

  retval = checkID();
  if (retval != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688::startupDefault: checkID returned: "));
    debugPrintStatus(retval);
    debugPrintln(F(""));
    status = retval;
    return status;
  }
  
  retval = setFullScale((ICM_42688_Internal_Acc | ICM_42688_Internal_Gyr));
  if (retval != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688::startupDefault: setFullScale returned: "));
    debugPrintStatus(retval);
    debugPrintln(F(""));
    status = retval;
    return status;
  }

  retval = enableFIFO(true); //passed value sets FIFO_MODE 0 : bypass, 1 : stream to fifo register, FIFO_CONFIG1 is also setup
  if (retval != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688::startupDefault: enableFIFO returned: "));
    debugPrintStatus(retval);
    debugPrintln(F(""));
    status = retval;
    return status;
  }
  retval = setupFIFO(true); // init DMP, change to little endian, enable partial reads.
  if (retval != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688::startupDefault: setupFIFO returned: "));
    debugPrintStatus(retval);
    debugPrintln(F(""));
    status = retval;
    return status;
  }
  return status;
}

// direct read/write
ICM_42688_Status_e ICM_42688::read(uint8_t reg, uint8_t *pdata, uint32_t len)
{
  status = ICM_42688_execute_r(&_device, reg, pdata, len);
  return (status);
}

ICM_42688_Status_e ICM_42688::write(uint8_t reg, uint8_t *pdata, uint32_t len)
{
  status = ICM_42688_execute_w(&_device, reg, pdata, len);
  return (status);
}

// FIFO

ICM_42688_Status_e ICM_42688::enableFIFO(bool enable)
{
  status = ICM_42688_enable_FIFO(&_device, enable);
  return status;
}

ICM_42688_Status_e ICM_42688::setupFIFO(bool enable)
{
  status = ICM_42688_setup_FIFO(&_device, enable);
  return status;
}

ICM_42688_Status_e ICM_42688::resetFIFO(void)
{
  status = ICM_42688_reset_FIFO(&_device);
  return status;
}

ICM_42688_Status_e ICM_42688::setFIFOmode(bool snapshot)
{
  // Default to Stream (non-Snapshot) mode
  status = ICM_42688_set_FIFO_mode(&_device, snapshot);
  return status;
}

ICM_42688_Status_e ICM_42688::getFIFOcount(uint16_t *count)
{
  status = ICM_42688_get_FIFO_count(&_device, count);
  return status;
}

ICM_42688_Status_e ICM_42688::readFIFO(uint8_t *data, uint8_t len)
{
  status = ICM_42688_read_FIFO(&_device, data, len);
  return status;
}

ICM_42688_Status_e ICM_42688::setGyroSF(unsigned char div, int gyro_level)
{
    status = inv_icm42688_set_gyro_sf(&_device, div, gyro_level);
    debugPrint(F("ICM_42688::setGyroSF:  pll: "));
    debugPrintf((int)_device._gyroSFpll);
    debugPrint(F("  Gyro SF is: "));
    debugPrintf((int)_device._gyroSF);
    debugPrintln(F(""));
    return status;
}
ICM_42688_Status_e ICM_42688::readDataFromFIFO(icm_42688_data_t *data)
{
  status = inv_icm42688_read_data(&_device, data);
  return status;
}

// I2C
ICM_42688_I2C::ICM_42688_I2C()
{
}



ICM_42688_Status_e ICM_42688_I2C::begin(TwoWire &wirePort, bool ad0val, uint8_t ad0pin)
{
  // Associate
  _ad0 = ad0pin;
  _i2c = &wirePort;
  _ad0val = ad0val;

  _addr = ICM_42688_I2C_ADDR_AD0;
  if (_ad0val)
  {
    _addr = ICM_42688_I2C_ADDR_AD1;
  }

  // Set pinmodes
  if (_ad0 != ICM_42688_ARD_UNUSED_PIN)
  {
    pinMode(_ad0, OUTPUT);
  }

  // Set pins to default positions
  if (_ad0 != ICM_42688_ARD_UNUSED_PIN)
  {
    digitalWrite(_ad0, _ad0val);
  }

  // _i2c->begin(); // Moved into user's sketch

  // Set up the serif
  _serif.write = ICM_42688_write_I2C;
  _serif.read = ICM_42688_read_I2C;
  _serif.user = (void *)this; // refer to yourself in the user field

  // Link the serif
  _device._serif = &_serif;

  _device._last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of ICM_42688_set_bank.


  // Perform default startup
  status = startupDefault();
  if (status != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688_I2C::begin: startupDefault returned: "));
    debugPrintStatus(status);
    debugPrintln(F(""));
  }
  return status;
}

// SPI

// SPISettings ICM_42688_SPI_DEFAULT_SETTINGS(ICM_42688_SPI_DEFAULT_FREQ, ICM_42688_SPI_DEFAULT_ORDER, ICM_42688_SPI_DEFAULT_MODE);

ICM_42688_SPI::ICM_42688_SPI()
{
}

ICM_42688_Status_e ICM_42688_SPI::begin(uint8_t csPin, SPIClass &spiPort, uint32_t SPIFreq)
{
  if (SPIFreq > 7000000)
    SPIFreq = 7000000; // Limit SPI frequency to 7MHz

  // Associate
  _spi = &spiPort;
  _spisettings = SPISettings(SPIFreq, ICM_42688_SPI_DEFAULT_ORDER, ICM_42688_SPI_DEFAULT_MODE);
  _cs = csPin;

  // Set pinmodes
  pinMode(_cs, OUTPUT);

  // Set pins to default positions
  digitalWrite(_cs, HIGH);

  // _spi->begin(); // Moved into user's sketch

  // 'Kickstart' the SPI hardware.
  _spi->beginTransaction(_spisettings);
  _spi->transfer(0x00);
  _spi->endTransaction();

  // Set up the serif
  _serif.write = ICM_42688_write_SPI;
  _serif.read = ICM_42688_read_SPI;
  _serif.user = (void *)this; // refer to yourself in the user field

  // Link the serif
  _device._serif = &_serif;

#if defined(ICM_42688_USE_DMP)
  _device._dmp_firmware_available = true; // Initialize _dmp_firmware_available
#else
  _device._dmp_firmware_available = false; // Initialize _dmp_firmware_available
#endif

  _device._last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of ICM_42688_set_bank.
  _device._last_mems_bank = 255;    // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm42688_write_mems.
  _device._gyroSF = 0;              // Use this to record the GyroSF, calculated by inv_icm42688_set_gyro_sf
  _device._gyroSFpll = 0;

  // Perform default startup
  // Do a minimal startupDefault if using the DMP. User can always call startupDefault(false) manually if required.
  status = startupDefault();
  if (status != ICM_42688_Stat_Ok)
  {
    debugPrint(F("ICM_42688_SPI::begin: startupDefault returned: "));
    debugPrintStatus(status);
    debugPrintln(F(""));
  }

  return status;
}

// serif functions for the I2C and SPI classes
ICM_42688_Status_e ICM_42688_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
  if (user == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }
  TwoWire *_i2c = ((ICM_42688_I2C *)user)->_i2c; // Cast user field to ICM_42688_I2C type and extract the I2C interface pointer
  uint8_t addr = ((ICM_42688_I2C *)user)->_addr;
  if (_i2c == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }

  _i2c->beginTransmission(addr);
  _i2c->write(reg);
  _i2c->write(data, (uint8_t)len);
  _i2c->endTransmission();

  // for( uint32_t indi = 0; indi < len; indi++ ){
  //     _i2c->beginTransmission(addr);
  //     _i2c->write(reg + indi);
  //     _i2c->write(*(data + indi) );
  //     _i2c->endTransmission();
  //     delay(10);
  // }

  // delay(10);

  return ICM_42688_Stat_Ok;
}

ICM_42688_Status_e ICM_42688_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
  if (user == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }
  TwoWire *_i2c = ((ICM_42688_I2C *)user)->_i2c;
  uint8_t addr = ((ICM_42688_I2C *)user)->_addr;
  if (_i2c == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }

  _i2c->beginTransmission(addr);
  _i2c->write(reg);
  _i2c->endTransmission(false); // Send repeated start

  uint32_t num_received = _i2c->requestFrom(addr, len);

  if (num_received == len)
  {
    for (uint8_t i = 0; i < len; i++)
    {
      buff[i] = _i2c->read();
    }
    return ICM_42688_Stat_Ok;
  }
  else
  {
    return ICM_42688_Stat_NoData;
  }

  if (len != 0)
  {
    return ICM_42688_Stat_NoData;
  }
  return ICM_42688_Stat_Ok;
}

ICM_42688_Status_e ICM_42688_write_SPI(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
  if (user == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }
  SPIClass *_spi = ((ICM_42688_SPI *)user)->_spi; // Cast user field to ICM_42688_SPI type and extract the SPI interface pointer
  uint8_t cs = ((ICM_42688_SPI *)user)->_cs;
  SPISettings spisettings = ((ICM_42688_SPI *)user)->_spisettings;
  if (_spi == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }

  // 'Kickstart' the SPI hardware. This is a fairly high amount of overhead, but it guarantees that the lines will start in the correct states even when sharing the SPI bus with devices that use other modes
  _spi->beginTransaction(spisettings);
  _spi->transfer(0x00);
  _spi->endTransaction();

  _spi->beginTransaction(spisettings);
  digitalWrite(cs, LOW);
  // delayMicroseconds(5);
  _spi->transfer(((reg & 0x7F) | 0x00));
  //  SPI.transfer(data, len); // Can't do this thanks to Arduino's poor implementation
  for (uint32_t indi = 0; indi < len; indi++)
  {
    _spi->transfer(*(data + indi));
  }
  // delayMicroseconds(5);
  digitalWrite(cs, HIGH);
  _spi->endTransaction();

  return ICM_42688_Stat_Ok;
}

ICM_42688_Status_e ICM_42688_read_SPI(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
  if (user == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }
  SPIClass *_spi = ((ICM_42688_SPI *)user)->_spi;
  uint8_t cs = ((ICM_42688_SPI *)user)->_cs;
  SPISettings spisettings = ((ICM_42688_SPI *)user)->_spisettings;
  if (_spi == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }

  // 'Kickstart' the SPI hardware. This is a fairly high amount of overhead, but it guarantees that the lines will start in the correct states
  _spi->beginTransaction(spisettings);
  _spi->transfer(0x00);
  _spi->endTransaction();

  _spi->beginTransaction(spisettings);
  digitalWrite(cs, LOW);
  //   delayMicroseconds(5);
  _spi->transfer(((reg & 0x7F) | 0x80));
  //  SPI.transfer(data, len); // Can't do this thanks to Arduino's stupid implementation
  for (uint32_t indi = 0; indi < len; indi++)
  {
    *(buff + indi) = _spi->transfer(0x00);
  }
  //   delayMicroseconds(5);
  digitalWrite(cs, HIGH);
  _spi->endTransaction();

  return ICM_42688_Stat_Ok;
}


