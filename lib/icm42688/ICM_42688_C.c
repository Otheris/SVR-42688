#include "ICM_42688_C.h"
#include <stdint.h>

const ICM_42688_Serif_t NullSerif = {
    NULL, // write
    NULL, // read
    NULL, // user
};

// Private function prototypes

// Function definitions

ICM_42688_Status_e ICM_42688_init_struct(ICM_42688_Device_t *pdev)
{
  // Initialize all elements by 0 except for _last_bank
  // Initialize _last_bank to 4 (invalid bank number)
  // so ICM_42688_set_bank function does not skip issuing bank change operation
  static const ICM_42688_Device_t init_device = { ._last_bank = 4 };
  *pdev = init_device;
  return ICM_42688_Stat_Ok;
}

ICM_42688_Status_e ICM_42688_link_serif(ICM_42688_Device_t *pdev, const ICM_42688_Serif_t *s)
{
  if (s == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }
  if (pdev == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }
  pdev->_serif = s;
  return ICM_42688_Stat_Ok;
}

ICM_42688_Status_e ICM_42688_execute_w(ICM_42688_Device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len)
{
  if (pdev->_serif->write == NULL)
  {
    return ICM_42688_Stat_NotImpl;
  }
  return (*pdev->_serif->write)(regaddr, pdata, len, pdev->_serif->user);
}

ICM_42688_Status_e ICM_42688_execute_r(ICM_42688_Device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len)
{
  if (pdev->_serif->read == NULL)
  {
    return ICM_42688_Stat_NotImpl;
  }
  return (*pdev->_serif->read)(regaddr, pdata, len, pdev->_serif->user);
}


ICM_42688_Status_e ICM_42688_set_bank(ICM_42688_Device_t *pdev, uint8_t bank)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  if ((bank > 4) & (bank != 3))
  {
    return ICM_42688_Stat_ParamErr;
  } // Only 4 possible banks (bank 3 not on datasheet)

  if (bank == pdev->_last_bank) // Do we need to change bank?
    return ICM_42688_Stat_Ok;   // Bail if we don't need to change bank to avoid unnecessary bus traffic

  pdev->_last_bank = bank;   // Store the requested bank (before we bit-shift)
  retval = ICM_42688_execute_w(pdev, REGISTER_BANK_SEL, &bank, 1);
  return retval;
}

ICM_42688_Status_e ICM_42688_sw_reset(ICM_42688_Device_t *pdev)
{
  ICM_42688_set_bank(pdev, 0); // Must be in the right bank
  uint8_t configdata;
  ICM_42688_Status_e retval =  ICM_42688_execute_r(pdev, REGISTER_BANK_SEL, &configdata,1);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  ICM_42688_DEVICE_CONFIG_t config = *((ICM_42688_DEVICE_CONFIG_t*)&configdata);
  config.SOFT_RESET_CONFIG = 1;
  configdata = *((uint8_t*)&config);
  retval =  ICM_42688_execute_w(pdev, REGISTER_BANK_SEL, &configdata, 1);
  delay(150);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_42688_Status_e ICM_42688_get_who_am_i(ICM_42688_Device_t *pdev, uint8_t *whoami)
{
  if (whoami == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }
  ICM_42688_set_bank(pdev, 0); // Must be in the right bank
  return ICM_42688_execute_r(pdev, WHO_AM_I, whoami, 1);
}

ICM_42688_Status_e ICM_42688_set_I2C_mode(ICM_42688_Device_t *pdev)
{
    // Setting values needed for I2C operation see pg58 datasheet
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  
  ICM_42688_set_bank(pdev, 1);
  uint8_t value;
  ICM_42688_INTF_CONFIG6_t Field;
  
  //set I3C_EN=1
	retval |= ICM_42688_execute_r(pdev, INTF_CONFIG6, &value, 1);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
	Field = *((ICM_42688_INTF_CONFIG6_t*)&value);
  Field.I3C_EN = 1;
  Field.I3C_SDR_EN = 0;
  Field.I3C_DDR_EN = 0;
  value = *((uint8_t*)&Field);
	retval |= ICM_42688_execute_w(pdev, INTF_CONFIG6, &value, 1);
  if (retval != ICM_42688_Stat_Ok)  
  {
    return retval;
  }


  value = 0;
  ICM_42688_set_bank(pdev, 1);

  // set I2C_SLEW_RATE=1 & SPI_SLEW_RATE=1
  ICM_42688_DRIVE_CONFIG_t Field2;
	retval |= ICM_42688_execute_r(pdev, DRIVE_CONFIG, &value, 1);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
	Field2 = *((ICM_42688_DRIVE_CONFIG_t*)&value);
  Field2.I2C_SLEW_RATE = 1;
  Field2.SPI_SLEW_RATE = 1;
  value = *((uint8_t*)&Field2);
	retval |= ICM_42688_execute_w(pdev, DRIVE_CONFIG, &value, 1);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  delay(50);

  // Enable Gyro and Accel
  ICM_42688_PWR_MGMT0_t reg;
  ICM_42688_set_bank(pdev, 0); // Must be in the right bank

  retval = ICM_42688_execute_r(pdev, PWR_MGMT0, (uint8_t *)&reg, sizeof(ICM_42688_PWR_MGMT0_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  reg.GYRO_MODE = 0b11;
  reg.ACCEL_MODE = 0b11; 
  reg.IDLE = 0;

  retval = ICM_42688_execute_w(pdev, PWR_MGMT0, (uint8_t *)&reg, sizeof(ICM_42688_PWR_MGMT0_t));
  delay(300); //Wait for accel/gyro startup
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  return retval;
}

ICM_42688_Status_e ICM_42688_check_id(ICM_42688_Device_t *pdev)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  uint8_t whoami = 0x00;
  retval = ICM_42688_get_who_am_i(pdev, &whoami);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  if (whoami != ICM_42688_WHOAMI)
  {
    return ICM_42688_Stat_WrongID;
  }
  return retval;
}

ICM_42688_Status_e ICM_42688_data_ready(ICM_42688_Device_t *pdev)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  ICM_42688_INT_STATUS_t reg;
  retval = ICM_42688_set_bank(pdev, 0); // Must be in the right bank
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  retval = ICM_42688_execute_r(pdev, INT_STATUS, (uint8_t *)&reg, sizeof(ICM_42688_INT_STATUS_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  if (!reg.DATA_RDY_INT)
  {
    retval = ICM_42688_Stat_NoData;
  }
  return retval;
}
//  TODO: do plan to use int based maybe...
// Interrupt Configuration
ICM_42688_Status_e ICM_42688_int_pin_cfg(ICM_42688_Device_t *pdev, ICM_42688_INT_CONFIG_t *write, ICM_42688_INT_CONFIG_t *read)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  retval = ICM_42688_set_bank(pdev, 0); // Must be in the right bank
  if (write != NULL)
  { // write first, if available
    retval = ICM_42688_execute_w(pdev, INT_CONFIG, (uint8_t *)write, sizeof(ICM_42688_INT_CONFIG_t));
    if (retval != ICM_42688_Stat_Ok)
    {
      return retval;
    }
  }
  if (read != NULL)
  { // then read, to allow for verification
    retval = ICM_42688_execute_r(pdev, INT_CONFIG, (uint8_t *)read, sizeof(ICM_42688_INT_CONFIG_t));
    if (retval != ICM_42688_Stat_Ok)
    {
      return retval;
    }
  }
  return retval;
}

// TODO, while the FS is set default here, we might want to up the 1khz ODR??
ICM_42688_Status_e ICM_42688_set_full_scale(ICM_42688_Device_t *pdev, ICM_42688_InternalSensorID_bm sensors)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;

  if (!(sensors & (ICM_42688_Internal_Acc | ICM_42688_Internal_Gyr)))
  {
    return ICM_42688_Stat_SensorNotSupported;
  }
  
  if (sensors & ICM_42688_Internal_Acc)
  {
    ICM_42688_ACCEL_CONFIG0_t reg;
    retval |= ICM_42688_set_bank(pdev, 0); // Must be in the right bank
    if (retval != ICM_42688_Stat_Ok)
    {
      return retval;
    }    
    retval = ICM_42688_execute_r(pdev, ACCEL_CONFIG0, (uint8_t *)&reg, sizeof(ICM_42688_ACCEL_CONFIG0_t));
    if (retval != ICM_42688_Stat_Ok)
    {
      return retval;
    }    
    reg.ACCEL_FS_SEL = 0;
    reg.ACCEL_ODR = 0b0010; //TODO: Set to 16khz???
    retval = ICM_42688_execute_w(pdev, ACCEL_CONFIG0, (uint8_t *)&reg, sizeof(ICM_42688_ACCEL_CONFIG0_t));
    if (retval != ICM_42688_Stat_Ok)
    {
      return retval;
    }  
  }
  if (sensors & ICM_42688_Internal_Gyr)
  {
    ICM_42688_GYRO_CONFIG0_t reg;
    retval = ICM_42688_set_bank(pdev, 0); // Must be in the right bank
    if (retval != ICM_42688_Stat_Ok)
    {
      return retval;
    }
    retval = ICM_42688_execute_r(pdev, GYRO_CONFIG0, (uint8_t *)&reg, sizeof(ICM_42688_GYRO_CONFIG0_t));
      if (retval != ICM_42688_Stat_Ok)
    {
      return retval;
    }
    reg.GYRO_FS_SEL = 0;
    reg.GYRO_ODR = 0b0010;
    retval |= ICM_42688_execute_w(pdev, GYRO_CONFIG0, (uint8_t *)&reg, sizeof(ICM_42688_GYRO_CONFIG0_t));
    if (retval != ICM_42688_Stat_Ok)
    {
      return retval;
    }  
  }
  return retval;
}



// Interface Things
//something to do with INT bypass, but only reference to 42688 bypass is in FIFO mode (and is deafult). likely irrelevant.
ICM_42688_Status_e ICM_42688_i2c_master_passthrough(ICM_42688_Device_t *pdev, bool passthrough)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
/*
  ICM_42688_INT_PIN_CFG_t reg;
  retval = ICM_42688_set_bank(pdev, 0);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  retval = ICM_42688_execute_r(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)&reg, sizeof(ICM_42688_INT_PIN_CFG_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  reg.BYPASS_EN = passthrough;
  retval = ICM_42688_execute_w(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)&reg, sizeof(ICM_42688_INT_PIN_CFG_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
*/
  return retval;
}

// Higher Levels
ICM_42688_Status_e ICM_42688_get_agt(ICM_42688_Device_t *pdev, ICM_42688_agt_t *pagt)
{
  if (pagt == NULL)
  {
    return ICM_42688_Stat_ParamErr;
  }

  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  const uint8_t numbytes = 14; //Read Temp, Accel, gyro
  uint8_t buff[numbytes];

  // Get readings
  retval |= ICM_42688_set_bank(pdev, 0);
  retval |= ICM_42688_execute_r(pdev, (uint8_t)TEMP_DATA1, buff, numbytes);
  pagt->tmp.val = ((buff[0] << 8) | (buff[1] & 0xFF));

  pagt->acc.axes.x = ((buff[2] << 8) | (buff[3] & 0xFF));
  pagt->acc.axes.y = ((buff[4] << 8) | (buff[5] & 0xFF));
  pagt->acc.axes.z = ((buff[6] << 8) | (buff[7] & 0xFF));

  pagt->gyr.axes.x = ((buff[8] << 8) | (buff[9] & 0xFF));
  pagt->gyr.axes.y = ((buff[10] << 8) | (buff[11] & 0xFF));
  pagt->gyr.axes.z = ((buff[12] << 8) | (buff[13] & 0xFF));


  // Get settings to be able to compute scaled values
  retval |= ICM_42688_set_bank(pdev, 2);
  ICM_42688_ACCEL_CONFIG0_t acfg;
  retval |= ICM_42688_execute_r(pdev, (uint8_t)ACCEL_CONFIG0, (uint8_t *)&acfg, 1 * sizeof(acfg));
  pagt->fss.a = acfg.ACCEL_FS_SEL; // Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range
                                    // Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
  retval |= ICM_42688_set_bank(pdev, 2);
  ICM_42688_GYRO_CONFIG0_t gcfg1;
  retval |= ICM_42688_execute_r(pdev, (uint8_t)GYRO_CONFIG0, (uint8_t *)&gcfg1, 1 * sizeof(gcfg1));
  pagt->fss.g = gcfg1.GYRO_FS_SEL;
  ICM_42688_ACCEL_CONFIG1_t acfg2;
  retval |= ICM_42688_execute_r(pdev, (uint8_t)ACCEL_CONFIG1, (uint8_t *)&acfg2, 1 * sizeof(acfg2));

  return retval;
}

// FIFO

ICM_42688_Status_e ICM_42688_enable_FIFO(ICM_42688_Device_t *pdev, bool enable)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;

  INT_42688_FIFO_CONFIG_t ctrl;
  retval = ICM_42688_set_bank(pdev, 0);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  retval = ICM_42688_execute_r(pdev, FIFO_CONFIG, (uint8_t *)&ctrl, sizeof(INT_42688_FIFO_CONFIG_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  if (enable)
    ctrl.FIFO_MODE = 3;
  else
    ctrl.FIFO_MODE = 0;

  retval = ICM_42688_execute_w(pdev, FIFO_CONFIG, (uint8_t *)&ctrl, sizeof(INT_42688_FIFO_CONFIG_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  
  ICM_42688_FIFO_CONFIG1_t ctrl2;
  retval = ICM_42688_set_bank(pdev, 0);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  retval = ICM_42688_execute_r(pdev, FIFO_CONFIG1, (uint8_t *)&ctrl2, sizeof(ICM_42688_FIFO_CONFIG1_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  ctrl2.FIFO_RESUME_PARTIAL_RD = 1;
  ctrl2.FIFO_HIRES_EN = 0;
  ctrl2.FIFO_ACCEL_EN = 1;
  ctrl2.FIFO_GYRO_EN = 1;
  ctrl2.FIFO_TEMP_EN = 1;

  retval = ICM_42688_execute_w(pdev, FIFO_CONFIG1, (uint8_t *)&ctrl2, sizeof(ICM_42688_FIFO_CONFIG1_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }


  ICM_42688_SIGNAL_PATH_RESET_t ctrl3;
  retval = ICM_42688_execute_r(pdev, SIGNAL_PATH_RESET, (uint8_t *)&ctrl3, sizeof(ICM_42688_SIGNAL_PATH_RESET_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  ctrl3.DMP_INIT_EN = 1;
  ctrl3.FIFO_FLUSH = 1;

  retval = ICM_42688_execute_w(pdev, SIGNAL_PATH_RESET, (uint8_t *)&ctrl3, sizeof(ICM_42688_SIGNAL_PATH_RESET_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  return retval;
}
ICM_42688_Status_e ICM_42688_setup_FIFO(ICM_42688_Device_t *pdev, bool snapshot)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  ICM_42688_INTF_CONFIG0_t ctrl;
  retval = ICM_42688_execute_r(pdev, INTF_CONFIG0, (uint8_t *)&ctrl, sizeof(ICM_42688_INTF_CONFIG0_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  ctrl.FIFO_COUNT_ENDIAN = 0;
  ctrl.SENSOR_DATA_ENDIAN = 0;
  ctrl.FIFO_COUNT_REC = 0;

  retval = ICM_42688_execute_w(pdev, INTF_CONFIG0, (uint8_t *)&ctrl, sizeof(ICM_42688_INTF_CONFIG0_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  return retval;
  //read fifo mode expect 01
  //set DMP_INIT_EN? 1
  //set FIFO_COUNT_ENDIAN to 0 for little endian
  //set SENSOR_DATA_ENDIAN to 0 for little endian
  //set FIFO_RESUME_PARTIAL_RD 1 to allow partial reads FIFO_CONFIG1
  // read ZG_DISABLE shoould be 0 etc.. to see if enabled. if not change that...
  // I3C_BUS_MODE = 0
}
ICM_42688_Status_e ICM_42688_set_FIFO_mode(ICM_42688_Device_t *pdev, bool snapshot)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  /* FIFO was setup in stream mode...
  ICM_42688_FIFO_MODE_t ctrl;
  retval = ICM_42688_set_bank(pdev, 0);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  retval = ICM_42688_execute_r(pdev, AGB0_REG_FIFO_MODE, (uint8_t *)&ctrl, sizeof(ICM_42688_FIFO_MODE_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  if (snapshot)
    ctrl.FIFO_MODE = 0x1F; // Datasheet says "FIFO_MODE[4:0]"
  else
    ctrl.FIFO_MODE = 0;

  retval = ICM_42688_execute_w(pdev, AGB0_REG_FIFO_MODE, (uint8_t *)&ctrl, sizeof(ICM_42688_FIFO_MODE_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
*/ 
  return retval;
}

ICM_42688_Status_e ICM_42688_get_FIFO_count(ICM_42688_Device_t *pdev, uint16_t *count)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;
  ICM_42688_FIFO_COUNTL_t ctrll;
  ICM_42688_FIFO_COUNTH_t ctrlh;
  retval = ICM_42688_set_bank(pdev, 0);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  retval = ICM_42688_execute_r(pdev, FIFO_COUNTH, (uint8_t *)&ctrlh, sizeof(ICM_42688_FIFO_COUNTH_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  retval = ICM_42688_execute_r(pdev, FIFO_COUNTL, (uint8_t *)&ctrll, sizeof(ICM_42688_FIFO_COUNTL_t));
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }
  *count = ((uint16_t)ctrll.FIFO_COUNTL << 8) | (uint16_t)ctrlh.FIFO_COUNTH;

  return retval;
}

ICM_42688_Status_e ICM_42688_read_FIFO(ICM_42688_Device_t *pdev, uint8_t *data, uint8_t len)
{
  ICM_42688_Status_e retval = ICM_42688_Stat_Ok;

  retval = ICM_42688_set_bank(pdev, 0);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  retval = ICM_42688_execute_r(pdev, FIFO_DATA, data, len);
  if (retval != ICM_42688_Stat_Ok)
  {
    return retval;
  }

  return retval;
}
int16_t inv_icm42688_swap_bytes(int16_t x)
{
  int16_t hi = (x & 0xff00); 
  int16_t lo = (x & 0xff);
  int16_t ret = (lo << 8);
  ret |= (hi >> 8);
  return ret;
}

ICM_42688_Status_e inv_icm42688_read_data(ICM_42688_Device_t *pdev, icm_42688_data_t *data)
{
  ICM_42688_Status_e result = ICM_42688_Stat_Ok;// set default status to OK
  icm_42688_data_t temp;
  result = ICM_42688_read_FIFO(pdev, &temp, icm_42688_Packet_Bytes);
  if (result != ICM_42688_Stat_Ok)
    return result;
  
  //temp.Raw_Accel_X = inv_icm42688_swap_bytes(temp.Raw_Accel_X);
  //temp.Raw_Accel_Y = inv_icm42688_swap_bytes(temp.Raw_Accel_Y);
  //temp.Raw_Accel_Z = inv_icm42688_swap_bytes(temp.Raw_Accel_Z);
  
  //temp.Raw_Gyro_X = inv_icm42688_swap_bytes(temp.Raw_Gyro_X);
  //temp.Raw_Gyro_Y = inv_icm42688_swap_bytes(temp.Raw_Gyro_Y);
  //temp.Raw_Gyro_Z = inv_icm42688_swap_bytes(temp.Raw_Gyro_Z);


  *data = temp;

  return result;
}
