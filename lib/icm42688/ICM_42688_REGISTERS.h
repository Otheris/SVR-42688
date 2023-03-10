/*

This file contains a useful c translation of the datasheet register map

*/

#ifndef _ICM_42688_REGISTERS_H_
#define _ICM_42688_REGISTERS_H_

#include <stdint.h>

#ifdef __cplusplusc
extern "C"
{
#endif /* __cplusplus */

  typedef enum
  {
    // User Bank 0
    DEVICE_CONFIG = 0x11,
    DRIVE_CONFIG = 0x13,
    INT_CONFIG = 0x14,
    FIFO_CONFIG = 0x16,
    TEMP_DATA1 = 0x1D,
    TEMP_DATA0 = 0x1E,
    ACCEL_DATA_X1 = 0x1F,
    ACCEL_DATA_X0 = 0x20,
    ACCEL_DATA_Y1 = 0x21,
    ACCEL_DATA_Y0 = 0x22,
    ACCEL_DATA_Z1 = 0x23,
    ACCEL_DATA_Z0 = 0x24,
    GYRO_DATA_X1 = 0x25,
    GYRO_DATA_X0 = 0x26,
    GYRO_DATA_Y1 = 0x27,
    GYRO_DATA_Y0 = 0x28,
    GYRO_DATA_Z1 = 0x29,
    GYRO_DATA_Z0 = 0x2A,
    TMST_FSYNCH = 0x2B,
    TMST_FSYNCL = 0x2C,
    INT_STATUS = 0x2D,
    FIFO_COUNTH = 0x2E,
    FIFO_COUNTL = 0x2F,
    FIFO_DATA = 0x30,
    APEX_DATA0 = 0x31,
    APEX_DATA1 = 0x32,
    APEX_DATA2 = 0x33,
    APEX_DATA3 = 0x34,
    APEX_DATA4 = 0x35,
    APEX_DATA5 = 0x36,
    INT_STATUS2 = 0x37,
    INT_STATUS3 = 0x38,
    // ??
    SIGNAL_PATH_RESET = 0x4B,
    INTF_CONFIG0 = 0x4C,
    INTF_CONFIG1 = 0x4D,
    PWR_MGMT0 = 0x4E,
    GYRO_CONFIG0 = 0x4F,
    ACCEL_CONFIG0 = 0x50,
    GYRO_CONFIG1 = 0x51,
    GYRO_ACCEL_CONFIG0 = 0x52,
    ACCEL_CONFIG1 = 0x53,
    TMST_CONFIG = 0x54,
    //??
    APEX_CONFIG0 = 0x56,
    SMD_CONFIG = 0x57,
    //??
    FIFO_CONFIG1 = 0x5F,
    FIFO_CONFIG2 = 0x60,
    FIFO_CONFIG3 = 0x61,
    FSYNC_CONFIG = 0x62,
    INT_CONFIG0 = 0x63,
    INT_CONFIG1 = 0x64,
    INT_SOURCE0 = 0x65,
    INT_SOURCE1 = 0x66,
    //??
    INT_SOURCE3 = 0x68,
    INT_SOURCE4 = 0x69,
    //??
    FIFO_LOST_PKT0 = 0x6C,
    FIFO_LOST_PKT1 = 0x6D,
    //??
    SELF_TEST_CONFIG = 0x70,
    //??
    WHO_AM_I = 0x75,
    REGISTER_BANK_SEL = 0x76,

    // User Bank 1
    SENSOR_CONFIG0 = 0x03,
    GYRO_CONFIG_STATIC2 = 0x0B,
    GYRO_CONFIG_STATIC3 = 0x0C,
    GYRO_CONFIG_STATIC4 = 0x0D,
    GYRO_CONFIG_STATIC5 = 0x0E,
    GYRO_CONFIG_STATIC6 = 0x0F,
    GYRO_CONFIG_STATIC7 = 0x10,
    GYRO_CONFIG_STATIC8 = 0x11,
    GYRO_CONFIG_STATIC9 = 0x12,
    GYRO_CONFIG_STATIC10 = 0x13,
    XG_ST_DATA = 0x5F,
    YG_ST_DATA = 0x60,
    ZG_ST_DATA = 0x61,
    TMSTVAL0 = 0x62,
    TMSTVAL1 = 0x63,
    TMSTVAL2 = 0x64,
    INTF_CONFIG4 = 0x7A,
    INTF_CONFIG5 = 0x7B,
    INTF_CONFIG6 = 0x7C,

    // User Bank 2
    ACCEL_CONFIG_STATIC2 = 0x03,
    ACCEL_CONFIG_STATIC3 = 0x04,
    ACCEL_CONFIG_STATIC4 = 0x05,
    XA_ST_DATA = 0x3B,
    YA_ST_DATA = 0x3C,
    ZA_ST_DATA = 0x3D,

    // User Bank 4
    APEX_CONFIG1 = 0x40,
    APEX_CONFIG2 = 0x41,
    APEX_CONFIG3 = 0x42,
    APEX_CONFIG4 = 0x43,
    APEX_CONFIG5 = 0x44,
    APEX_CONFIG6 = 0x45,
    APEX_CONFIG7 = 0x46,
    APEX_CONFIG8 = 0x47,
    APEX_CONFIG9 = 0x48,
    ACCEL_WOM_X_THR = 0x4A,
    ACCEL_WOM_Y_THR = 0x4B,
    ACCEL_WOM_Z_THR = 0x4C,
    INT_SOURCE6 = 0x4D,
    INT_SOURCE7 = 0x4E,
    INT_SOURCE8 = 0x4F,
    INT_SOURCE9 = 0x50,
    INT_SOURCE10 = 0x51,
    OFFSET_USER0 = 0x77,
    OFFSET_USER1 = 0x78,
    OFFSET_USER2 = 0x79,
    OFFSET_USER3 = 0x7A,
    OFFSET_USER4 = 0x7B,
    OFFSET_USER5 = 0x7C,
    OFFSET_USER6 = 0x7D,
    OFFSET_USER7 = 0x7E,
    OFFSET_USER8 = 0x7F
    
  } ICM_42688_Reg_Addr_e; // These enums are not needed for the user, so they stay in this scope (simplifies naming among other things)

  typedef struct {
    uint8_t SOFT_RESET_CONFIG : 1;
    uint8_t reserved_0 : 3;
    uint8_t SPI_MODE : 1;
    uint8_t reserved_1 : 3;
  } ICM_42688_DEVICE_CONFIG_t;

  typedef struct {
    uint8_t SPI_SLEW_RATE : 3;
    uint8_t I2C_SLEW_RATE : 3;
    uint8_t reserved_0 : 2;
  } ICM_42688_DRIVE_CONFIG_t;

  typedef struct {
    uint8_t INT1_POLARITY : 1;
    uint8_t INT1_DRIVE_CIRCUIT : 1;
    uint8_t INT1_MODE : 1;
    uint8_t INT2_POLARITY : 1;
    uint8_t INT2_DRIVE_CIRCUIT : 1;
    uint8_t INT2_MODE : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_INT_CONFIG_t;

  typedef struct {
    uint8_t reserved_0 : 6;
    uint8_t FIFO_MODE : 2;
  } INT_42688_FIFO_CONFIG_t;

  typedef struct
  {
    uint8_t TEMP_DATA1;
  } ICM_42688_TEMP_DATA1_t;

  typedef struct
  {
    uint8_t TEMP_DATA0;
  } ICM_42688_TEMP_DATA0_t;

  typedef struct
  {
    uint8_t ACCEL_DATA_X1;
  } ICM_42688_ACCEL_DATA_X1_t;

  typedef struct
  {
    uint8_t ACCEL_DATA_X0;
  } ICM_42688_ACCEL_DATA_X0_t;

  typedef struct
  {
    uint8_t ACCEL_DATA_Y1;
  } ICM_42688_ACCEL_DATA_Y1_t;

  typedef struct
  {
    uint8_t ACCEL_DATA_Y0;
  } ICM_42688_ACCEL_DATA_Y0_t;

  typedef struct
  {
    uint8_t ACCEL_DATA_Z1;
  } ICM_42688_ACCEL_DATA_Z1_t;

  typedef struct
  {
    uint8_t ACCEL_DATA_Z0;
  } ICM_42688_ACCEL_DATA_Z0_t;

  typedef struct
  {
    uint8_t GYRO_DATA_X1;
  } ICM_42688_GYRO_DATA_X1_t;

  typedef struct
  {
    uint8_t GYRO_DATA_X0;
  } ICM_42688_GYRO_DATA_X0_t;

  typedef struct
  {
    uint8_t GYRO_DATA_Y1;
  } ICM_42688_GYRO_DATA_Y1_t;

  typedef struct
  {
    uint8_t GYRO_DATA_Y0;
  } ICM_42688_GYRO_DATA_Y0_t;

  typedef struct
  {
    uint8_t GYRO_DATA_Z1;
  } ICM_42688_GYRO_DATA_Z1_t;

  typedef struct
  {
    uint8_t GYRO_DATA_Z0;
  } ICM_42688_GYRO_DATA_Z0_t;

  typedef struct
  {
    uint8_t TMST_FXYNCH;
  } ICM_42688_TMST_FXYNCH_t;

  typedef struct
  {
    uint8_t TMST_FXYNCL;
  } ICM_42688_TMST_FXYNCL_t;

  typedef struct 
  {
    uint8_t AGC_RDY_INT : 1;
    uint8_t FIFO_FULL_INT : 1;
    uint8_t FIFO_THS_INT : 1;
    uint8_t DATA_RDY_INT : 1;
    uint8_t RESET_DONE_INT : 1;
    uint8_t PLL_RDY_INT : 1;
    uint8_t UI_FSYNC_INT : 1;
    uint8_t reserved_0 : 1;
  } ICM_42688_INT_STATUS_t;

  typedef struct 
  {
    uint8_t FIFO_COUNTH;
  } ICM_42688_FIFO_COUNTH_t;

  typedef struct 
  {
    uint8_t FIFO_COUNTL;
  } ICM_42688_FIFO_COUNTL_t;

  typedef struct 
  {
    uint8_t FIFO_DATA;
  } ICM_42688_FIFO_DATA_t;

  typedef struct 
  {
    uint8_t STEP_CNTH;
  } ICM_42688_APEX_DATA0_t;

  typedef struct 
  {
    uint8_t STEP_CNTL;
  } ICM_42688_APEX_DATA1_t;

  typedef struct 
  {
    uint8_t STEP_CADENCE;
  } ICM_42688_APEX_DATA2_t;
  
  typedef struct 
  {
    uint8_t ACTIVITY_CLASS : 2;
    uint8_t DMP_IDLE : 1;
    uint8_t reserved_0 : 5;
  } ICM_42688_APEX_DATA3_t;

  typedef struct 
  {
    uint8_t TAP_DIR : 1;
    uint8_t TAP_AXIS : 2;
    uint8_t TAP_NUM : 2;
    uint8_t reserved_0 : 3;
  } ICM_42688_APEX_DATA4_t;

  typedef struct 
  {
    uint8_t DOUBLE_TAP_TIMING : 6;
    uint8_t reserved_0 : 2;
  } ICM_42688_APEX_DATA5_t;

  typedef struct 
  {
    uint8_t WOM_X_INT : 1;
    uint8_t WOM_Y_INT : 1;
    uint8_t WOM_Z_INT : 1;
    uint8_t SMD_INT : 1;
    uint8_t reserved_0 : 4;
  } ICM_42688_INT_STATUS2_t;

  typedef struct 
  {
    uint8_t TAP_DET_INT : 1;
    uint8_t SLEEP_INT : 1;
    uint8_t WAKE_INT : 1;
    uint8_t TILT_DET_INT : 1;
    uint8_t STEP_CNT_OVF_INT : 1;
    uint8_t STEP_DET_INT : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_INT_STATUS3_t;

  typedef struct 
  {
    uint8_t reserved_0 : 1;
    uint8_t FIFO_FLUSH : 1;
    uint8_t TMST_STROBE : 1;
    uint8_t ABORT_AND_RESET : 1;
    uint8_t reserved_1 : 1;
    uint8_t DMP_MEM_RESET_EN : 1;
    uint8_t DMP_INIT_EN : 1;
    uint8_t reserved_2 : 1;
  } ICM_42688_SIGNAL_PATH_RESET_t;

  typedef struct 
  {
    uint8_t UI_SIFS_CFG : 2;
    uint8_t reserved_0 : 2;
    uint8_t SENSOR_DATA_ENDIAN : 1;
    uint8_t FIFO_COUNT_ENDIAN : 1;
    uint8_t FIFO_COUNT_REC : 1;
    uint8_t FIFO_HOLD_LAST_DATA_EN : 1;
  } ICM_42688_INTF_CONFIG0_t;

  typedef struct 
  {
    uint8_t CLKSEL : 2;
    uint8_t RTC_MODE : 1;
    uint8_t ACCEL_LP_CLK_SEL : 1;
    uint8_t reserved_0 : 4;
  } ICM_42688_INTF_CONFIG1_t;

  typedef struct 
  {
    uint8_t ACCEL_MODE : 2;
    uint8_t GYRO_MODE : 2;
    uint8_t IDLE : 1;
    uint8_t TEMP_DIS : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_PWR_MGMT0_t;

  typedef struct 
  {
    uint8_t GYRO_ODR : 4;
    uint8_t reserved_0 : 1;
    uint8_t GYRO_FS_SEL : 3;
  } ICM_42688_GYRO_CONFIG0_t;

  typedef struct 
  {
    uint8_t ACCEL_ODR : 4;
    uint8_t reserved_0 : 1;
    uint8_t ACCEL_FS_SEL : 3;
  } ICM_42688_ACCEL_CONFIG0_t;

  typedef struct 
  {
    uint8_t GYRO_DEC2_M2_ORD : 2;
    uint8_t GYRO_UI_FILT_ORD : 2;
    uint8_t reserved_0 : 1;
    uint8_t TEMP_FILT_BW : 3;
  } ICM_42688_GYRO_CONFIG1_t;

  typedef struct 
  {
    uint8_t GYRO_UI_FILT_BW : 4;
    uint8_t ACCEL_UI_FILT_BW : 4;
  } ICM_42688_GYRO_ACCEL_CONFIG0_t;

  typedef struct 
  {
    uint8_t reserved_0 : 1;
    uint8_t ACCEL_DEC2_M2_ORD : 2;
    uint8_t ACCEL_UI_FILT_ORD : 2;
    uint8_t reserved_1 : 3;
  } ICM_42688_ACCEL_CONFIG1_t;

  typedef struct 
  {
    uint8_t TMST_EN : 1;
    uint8_t TMST_FSYNC_EN : 1;
    uint8_t TMST_DELTA_EN : 1;
    uint8_t TMST_RES : 1;
    uint8_t TMST_TO_REGS_EN : 1;
    uint8_t reserved_0 : 3;
  } ICM_42688_TMST_CONFIG_t;

  typedef struct 
  {
    uint8_t DMP_ODR : 2;
    uint8_t reserved_0 : 1;
    uint8_t R2W_EN : 1;
    uint8_t TILT_ENABLE : 1;
    uint8_t PED_ENABLE : 1;
    uint8_t TAP_ENABLE : 1;
    uint8_t DMP_POWER_SAVE : 1;
  } ICM_42688_APEX_CONFIG0_t;

  typedef struct 
  {
    uint8_t SMD_MODE : 2;
    uint8_t WOM_MODE : 1;
    uint8_t WOM_INT_MODE : 1;
    uint8_t reserved_0 : 4;
  } ICM_42688_SMD_CONFIG_t;

  typedef struct 
  {
    uint8_t FIFO_ACCEL_EN : 1;
    uint8_t FIFO_GYRO_EN : 1;
    uint8_t FIFO_TEMP_EN : 1;
    uint8_t FIFO_TMST_FSYNC_EN : 1;
    uint8_t FIFO_HIRES_EN : 1;
    uint8_t FIFO_WM_GT_TH : 1;
    uint8_t FIFO_RESUME_PARTIAL_RD : 1;
    uint8_t reserved_0 : 1;
  } ICM_42688_FIFO_CONFIG1_t;

  typedef struct 
  {
    uint8_t FIFO_WML;
  } ICM_42688_FIFO_CONFIG2_t;

  typedef struct 
  {
    uint8_t FIFO_WMH : 4;
    uint8_t reserved_0 : 4;
  } ICM_42688_FIFO_CONFIG3_t;

  typedef struct 
  {
    uint8_t FSYNC_POLARITY : 1;
    uint8_t FSYNC_UI_FLAG_CLEAR_SEL : 1;
    uint8_t reserved_0 : 2;
    uint8_t FSYNC_UI_SEL : 3;
    uint8_t reserved_1 : 1;
  } ICM_42688_FSYNC_CONFIG_t;

  typedef struct 
  {
    uint8_t FIFO_FULL_INT_CLEAR : 2;
    uint8_t FIFO_THS_INT_CLEAR : 2;
    uint8_t UI_DRDY_INT_CLEAR : 2;
    uint8_t reserved_0 : 2;
  } ICM_42688_INT_CONFIG0_t;

  typedef struct 
  {
    uint8_t reserved_0 : 4;
    uint8_t INT_ASYNC_RESET : 1;
    uint8_t INT_TDEASSERT_DISABLE : 1;
    uint8_t INT_TPULSE_DURATION : 1;
    uint8_t reserved_1 : 1;
  } ICM_42688_INT_CONFIG1_t;

  typedef struct 
  {
    uint8_t UI_AGC_RDY_INT1_EN : 1;
    uint8_t FIFO_FULL_INT1_EN : 1;
    uint8_t FIFO_THS_INT1_EN : 1;
    uint8_t UI_DRDY_INT1_EN : 1;
    uint8_t RESET_DONE_INT1_EN : 1;
    uint8_t PLL_RDY_INT1_EN : 1;
    uint8_t UI_FXYNC_INT1_EN : 1;
    uint8_t reserved_0 : 1;
  } ICM_42688_INT_SOURCE0_t;

  typedef struct 
  {
    uint8_t WOM_X_INT1_EN : 1;
    uint8_t WOM_Y_INT1_EN : 1;
    uint8_t WOM_Z_INT1_EN : 1;
    uint8_t SMD_INT1_EN : 1;
    uint8_t reserved_0 : 2;
    uint8_t I3C_PROTOCOL_ERROR_INT1_EN : 1;
    uint8_t reserved_1 : 1;
  } ICM_42688_INT_SOURCE1_t;

  typedef struct 
  {
    uint8_t UI_AGC_RDY_INT2_EN : 1;
    uint8_t FIFO_FULL_INT2_EN : 1;
    uint8_t FIFO_THS_INT2_EN : 1;
    uint8_t UI_DRDY_INT2_EN : 1;
    uint8_t RESET_DONE_INT2_EN : 1;
    uint8_t PLL_RDY_INT2_EN : 1;
    uint8_t UI_FXYNC_INT2_EN : 1;
    uint8_t reserved_0 : 1;
  } ICM_42688_INT_SOURCE3_t;

  typedef struct 
  {
    uint8_t WOM_X_INT2_EN : 1;
    uint8_t WOM_Y_INT2_EN : 1;
    uint8_t WOM_Z_INT2_EN : 1;
    uint8_t SMD_INT2_EN : 1;
    uint8_t reserved_0 : 2;
    uint8_t I3C_PROTOCOL_ERROR_INT2_EN : 1;
    uint8_t reserved_1 : 1;
  } ICM_42688_INT_SOURCE4_t;

  typedef struct 
  {
    uint8_t FIFO_LOST_PKT_CNTH;
  } ICM_42688_FIFO_LOST_PKT0_t;

  typedef struct 
  {
    uint8_t FIFO_LOST_PKT_CNTL;
  } ICM_42688_FIFO_LOST_PKT1_t;

  typedef struct 
  {
    uint8_t EN_GX_ST : 1;
    uint8_t EN_GY_ST : 1;
    uint8_t EN_GZ_ST : 1;
    uint8_t EN_AX_ST : 1;
    uint8_t EN_AY_ST : 1;
    uint8_t EN_AZ_ST : 1;
    uint8_t ACCEL_ST_POWER : 1;
    uint8_t reserved_0 : 1;
  } ICM_42688_SELF_TEST_CONFIG_t;

  typedef struct 
  {
    uint8_t WHO_AM_I;
  } ICM_42688_WHO_AM_I_t;

  typedef struct 
  {
    uint8_t BANK_SEL : 3;
    uint8_t reserved_0 : 5;
  } ICM_42688_REG_BANK_SEL_t;

// User Bank 1
  typedef struct 
  {
    uint8_t XA_DISABLE : 1;
    uint8_t YA_DISABLE : 1;
    uint8_t ZA_DISABLE : 1;
    uint8_t XG_DISABLE : 1;
    uint8_t YG_DISABLE : 1;
    uint8_t ZG_DISABLE : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_SENSOR_CONFIG0_t;

  typedef struct 
  {
    uint8_t GYRO_NF_DIS : 1;
    uint8_t GYRO_AAF_DIS : 1;
    uint8_t reserved_0 : 6;
  } ICM_42688_GYRO_CONFIG_STATIC2_t;

  typedef struct 
  {
    uint8_t GYRO_AAF_DELT : 6;
    uint8_t reserved_0 : 2;
  } ICM_42688_GYRO_CONFIG_STATIC3_t;

  typedef struct 
  {
    uint8_t GYRO_AAF_DELTSQRL;
  } ICM_42688_GYRO_CONFIG_STATIC4_t;

  typedef struct 
  {
    uint8_t GYRO_AAF_DELTSQRH : 4;
    uint8_t GYRO_AAF_BITSHIFT : 4;
  } ICM_42688_GYRO_CONFIG_STATIC5_t;

  typedef struct 
  {
    uint8_t GYRO_X_NF_COSWZ;
  } ICM_42688_GYRO_CONFIG_STATIC6_t;

  typedef struct 
  {
    uint8_t GYRO_Y_NF_COSWZ;
  } ICM_42688_GYRO_CONFIG_STATIC7_t;

  typedef struct 
  {
    uint8_t GYRO_Z_NF_COSWZ;
  } ICM_42688_GYRO_CONFIG_STATIC8_t;

  typedef struct 
  {
    uint8_t GYRO_X_NF_COSWZ : 1;
    uint8_t GYRO_Y_NF_COSWZ : 1;
    uint8_t GYRO_Z_NF_COSWZ : 1;
    uint8_t GYRO_X_NF_COSWZ_SEL : 1;
    uint8_t GYRO_Y_NF_COSWZ_SEL : 1;
    uint8_t GYRO_Z_NF_COSWZ_SEL : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_GYRO_CONFIG_STATIC9_t;

  typedef struct 
  {
    uint8_t reserved_0 : 4;
    uint8_t GYRO_NF_BW_SEL : 3;
    uint8_t reserved_1 : 1;
  } ICM_42688_GYRO_CONFIG_STATIC10_t;

  typedef struct 
  {
    uint8_t XG_ST_DATA;
  } ICM_42688_XG_ST_DATA_t;

  typedef struct 
  {
    uint8_t YG_ST_DATA;
  } ICM_42688_YG_ST_DATA_t;

  typedef struct 
  {
    uint8_t ZG_ST_DATA;
  } ICM_42688_ZG_ST_DATA_t;

  typedef struct 
  {
    uint8_t TMST_VALUEL;
  } ICM_42688_TMSTVAL0_t;

  typedef struct 
  {
    uint8_t TMST_VALUEM;
  } ICM_42688_TMSTVAL1_t;

  typedef struct 
  {
    uint8_t TMST_VALUEH : 4;
    uint8_t reserved_0 : 4;
  } ICM_42688_TMSTVAL2_t;

  typedef struct 
  {
    uint8_t reserved_0 : 1;
    uint8_t SPI_AP_4WIRE : 1;
    uint8_t reserved_1 : 4;
    uint8_t I3C_BUS_MODE : 1;
    uint8_t reserved_2 : 1;
  } ICM_42688_INTF_CONFIG4_t;

  typedef struct 
  {
    uint8_t reserved_0 : 1;
    uint8_t PIN9_FUNCTION : 2;
    uint8_t reserved_1 : 5;
  } ICM_42688_INTF_CONFIG5_t;

  typedef struct 
  {
    uint8_t I3C_SDR_EN : 1;
    uint8_t I3C_DDR_EN : 1;
    uint8_t I3C_IBI_EN : 1;
    uint8_t I3C_IBI_BYTE_EN : 1;
    uint8_t I3C_EN : 1;
    uint8_t reserved_0 : 2;
    uint8_t ASYNCTIME0_DIS : 1;
  } ICM_42688_INTF_CONFIG6_t;

// User Bank 2
  typedef struct 
  {
    uint8_t ACCEL_AAF_DIS : 1;
    uint8_t ACCEL_AAF_DELT : 6;
    uint8_t reserved_0 : 1;
  } ICM_42688_ACCEL_CONFIG_STATIC2_t;

  typedef struct 
  {
    uint8_t ACCEL_AAF_DELTSQRL;
  } ICM_42688_ACCEL_CONFIG_STATIC3_t;

  typedef struct 
  {
    uint8_t ACCEL_AAF_DELTSQRH : 4;
    uint8_t ACCEL_AAF_BITSHIFT : 4;
  } ICM_42688_ACCEL_CONFIG_STATIC4_t;

  typedef struct 
  {
    uint8_t XA_ST_DATA;
  } ICM_42688_XA_ST_DATA_t;

  typedef struct 
  {
    uint8_t YA_ST_DATA;
  } ICM_42688_YA_ST_DATA_t;

  typedef struct 
  {
    uint8_t ZA_ST_DATA;
  } ICM_42688_ZA_ST_DATA_t;

// User Bank 4
  typedef struct 
  {
    uint8_t DMP_POWER_SAVE_TIME_SEL : 4;
    uint8_t LOW_ENERGY_AMP_TH_SEL : 4;
  } ICM_42688_APEX_CONFIG1_t;

  typedef struct 
  {
    uint8_t PED_STEP_CNT_TH_SEL : 4;
    uint8_t PED_AMP_TH_SEL : 4;
  } ICM_42688_APEX_CONFIG2_t;

  typedef struct 
  {
    uint8_t PED_HI_EN_TH_SEL : 2;
    uint8_t PED_SB_TIMER_TH_SEL : 3;
    uint8_t PED_STEP_DET_TH_SEL : 3;
  } ICM_42688_APEX_CONFIG3_t;

  typedef struct 
  {
    uint8_t reserved_0 : 3;
    uint8_t SLEEP_TIME_OUT : 3;
    uint8_t TILT_WAIT_TIME_SEL : 2;
  } ICM_42688_APEX_CONFIG4_t;

  typedef struct 
  {
    uint8_t MOUNTING_MATRIX : 3;
    uint8_t reserved_0 : 5;
  } ICM_42688_APEX_CONFIG5_t;

  typedef struct 
  {
    uint8_t SLEEP_GESTURE_DELAY : 3;
    uint8_t reserved_0 : 5;
  } ICM_42688_APEX_CONFIG6_t;

  typedef struct 
  {
    uint8_t TAP_MAX_PEAK_TOL : 2;
    uint8_t TAP_MIN_JERK_THR : 6;
  } ICM_42688_APEX_CONFIG7_t;

  typedef struct 
  {
    uint8_t TAP_TMIN : 3;
    uint8_t TAP_TAVG : 2;
    uint8_t TAP_TMAX : 2;
    uint8_t reserved_0 : 1;
  } ICM_42688_APEX_CONFIG8_t;

  typedef struct 
  {
    uint8_t SENSITIVITY_MODE : 1;
    uint8_t reserved_0 : 7;
  } ICM_42688_APEX_CONFIG9_t;

  typedef struct 
  {
    uint8_t WOM_X_TH;
  } ICM_42688_ACCEL_WOM_X_THR_t;

  typedef struct 
  {
    uint8_t WOM_Y_TH;
  } ICM_42688_ACCEL_WOM_Y_THR_t;

  typedef struct 
  {
    uint8_t WOM_Z_TH;
  } ICM_42688_ACCEL_WOM_Z_THR_t;

  typedef struct 
  {
    uint8_t TAP_DET_INT1_EN : 1;
    uint8_t SLEEP_DET_INT1_EN : 1;
    uint8_t WAKE_DET_INT1_EN : 1;
    uint8_t TILT_DET_INT1_EN : 1;
    uint8_t STEP_CNT_OFL_INT1_EN : 1;
    uint8_t STEP_DET_INT1_EN : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_INT_SOURCE6_t;

  typedef struct 
  {
    uint8_t TAP_DET_INT2_EN : 1;
    uint8_t SLEEP_DET_INT2_EN : 1;
    uint8_t WAKE_DET_INT2_EN : 1;
    uint8_t TILT_DET_INT2_EN : 1;
    uint8_t STEP_CNT_OFL_INT2_EN : 1;
    uint8_t STEP_DET_INT2_EN : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_INT_SOURCE7_t;

  typedef struct 
  {
    uint8_t AGC_RDY_IBI_EN : 1;
    uint8_t FIFO_FULL_IBI_EN : 1;
    uint8_t FIFO_THS_IBI_EN : 1;
    uint8_t UI_DRDY_IBI_EN : 1;
    uint8_t PLL_RDY_IBI_EN : 1;
    uint8_t FSYNC_IBI_EN : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_INT_SOURCE8_t;

  typedef struct 
  {
    uint8_t reserved_0 : 1;
    uint8_t WOM_X_IBI_EN : 1;
    uint8_t WOM_Y_IBI_EN : 1;
    uint8_t WOM_Z_IBI_EN : 1;
    uint8_t SMD_IBI_EN : 1;
    uint8_t reserved_1 : 2;
    uint8_t I3C_PROTOCOL_ERROR_IBI_EN : 1;
  } ICM_42688_INT_SOURCE9_t;

  typedef struct 
  {
    uint8_t TAP_DET_IBI_EN : 1;
    uint8_t SLEEP_DET_IBI_EN : 1;
    uint8_t WAKE_DET_IBI_EN : 1;
    uint8_t TILT_DET_IBI_EN : 1;
    uint8_t STEP_CNT_OFL_IBI_EN : 1;
    uint8_t STEP_DET_IBI_EN : 1;
    uint8_t reserved_0 : 2;
  } ICM_42688_INT_SOURCE10_t;

  typedef struct 
  {
    uint8_t GYRO_X_OFFUSERL;
  } ICM_42688_OFFSET_USER0_t;

  typedef struct 
  {
    uint8_t GYRO_X_OFFUSERH : 4;
    uint8_t GYRO_Y_OFFUSERH : 4;
  } ICM_42688_OFFSET_USER1_t;

  typedef struct 
  {
    uint8_t GYRO_Y_OFFUSERL;
  } ICM_42688_OFFSET_USER2_t;

  typedef struct 
  {
    uint8_t GYRO_Z_OFFUSERL;
  } ICM_42688_OFFSET_USER3_t;

  typedef struct 
  {
    uint8_t GYRO_Z_OFFUSERH : 4;
    uint8_t ACCEL_X_OFFUSERH : 4;
  } ICM_42688_OFFSET_USER4_t;

  typedef struct 
  {
    uint8_t ACCEL_X_OFFUSERL;
  } ICM_42688_OFFSET_USER5_t;

  typedef struct 
  {
    uint8_t ACCEL_Y_OFFUSERL;
  } ICM_42688_OFFSET_USER6_t;

  typedef struct 
  {
    uint8_t ACCEL_Y_OFFUSERH : 4;
    uint8_t ACCEL_Z_OFFUSERH : 4;
  } ICM_42688_OFFSET_USER7_t;

  typedef struct 
  {
    uint8_t ACCEL_Z_OFFUSERL;
  } ICM_42688_OFFSET_USER8_t;

#ifdef __cplusplusc
}
#endif /* __cplusplus */

#endif /* _ICM_42688_REGISTERS_H_ */
