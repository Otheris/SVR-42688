/*

This is a C-compatible interface to the features presented by the ICM 42688 9-axis device
The imementation of the interface is flexible

*/

#ifndef _ICM_42688_C_H_
#define _ICM_42688_C_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ICM_42688_FIFO.h>
#include "ICM_42688_REGISTERS.h"
//#include "ICM_42688_ENUMERATIONS.h" // This is to give users access to usable value definiitons
//#include "ICM_42688_DMP.h"

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

extern int memcmp(const void *, const void *, size_t); // Avoid compiler warnings




#define ICM_42688_I2C_ADDR_AD0 0x68 // Or 0x69 when AD0 is high
#define ICM_42688_I2C_ADDR_AD1 0x69 //
#define ICM_42688_WHOAMI 0x47

#define MAG_AK09916_I2C_ADDR 0x0C
#define MAG_AK09916_WHO_AM_I 0x4809
#define MAG_REG_WHO_AM_I 0x00

/** @brief Max size that can be read across I2C or SPI data lines */
#define INV_MAX_SERIAL_READ 16
/** @brief Max size that can be written across I2C or SPI data lines */
#define INV_MAX_SERIAL_WRITE 16

  typedef enum
  {
    ICM_42688_Stat_Ok = 0x00, // The only return code that means all is well
    ICM_42688_Stat_Err,       // A general error
    ICM_42688_Stat_NotImpl,   // Returned by virtual functions that are not implemented
    ICM_42688_Stat_ParamErr,
    ICM_42688_Stat_WrongID,
    ICM_42688_Stat_InvalSensor, // Tried to apply a function to a sensor that does not support it (e.g. DLPF to the temperature sensor)
    ICM_42688_Stat_NoData,
    ICM_42688_Stat_SensorNotSupported,
    ICM_42688_Stat_DMPNotSupported,    // DMP not supported (no #define ICM_42688_USE_DMP)
    ICM_42688_Stat_DMPVerifyFail,      // DMP was written but did not verify correctly
    ICM_42688_Stat_FIFONoDataAvail,    // FIFO contains no data
    ICM_42688_Stat_FIFOIncompleteData, // FIFO contained incomplete data
    ICM_42688_Stat_FIFOMoreDataAvail,  // FIFO contains more data
    ICM_42688_Stat_UnrecognisedDMPHeader,
    ICM_42688_Stat_UnrecognisedDMPHeader2,
    ICM_42688_Stat_InvalDMPRegister, // Invalid DMP Register

    ICM_42688_Stat_NUM,
    ICM_42688_Stat_Unknown,
  } ICM_42688_Status_e;

  typedef enum
  {
    ICM_42688_Internal_Acc = (1 << 0),
    ICM_42688_Internal_Gyr = (1 << 1),
    ICM_42688_Internal_Mag = (1 << 2),
    ICM_42688_Internal_Tmp = (1 << 3),
    ICM_42688_Internal_Mst = (1 << 4), // I2C Master Ineternal
  } ICM_42688_InternalSensorID_bm;     // A bitmask of internal sensor IDs

  typedef union
  {
    int16_t i16bit[3];
    uint8_t u8bit[6];
  } ICM_42688_axis3bit16_t;

  typedef union
  {
    int16_t i16bit;
    uint8_t u8bit[2];
  } ICM_42688_axis1bit16_t;

  typedef struct
  {
    uint8_t a : 2;
    uint8_t g : 2;
    uint8_t reserved_0 : 4;
  } ICM_42688_fss_t; // Holds full-scale settings to be able to extract measurements with units


  typedef struct
  {
    uint16_t a;
    uint8_t g;
  } ICM_42688_smplrt_t;

  typedef struct
  {
    uint8_t I2C_MST_INT_EN : 1;
    uint8_t DMP_INT1_EN : 1;
    uint8_t PLL_RDY_EN : 1;
    uint8_t WOM_INT_EN : 1;
    uint8_t REG_WOF_EN : 1;
    uint8_t RAW_DATA_0_RDY_EN : 1;
    uint8_t FIFO_OVERFLOW_EN_4 : 1;
    uint8_t FIFO_OVERFLOW_EN_3 : 1;
    uint8_t FIFO_OVERFLOW_EN_2 : 1;
    uint8_t FIFO_OVERFLOW_EN_1 : 1;
    uint8_t FIFO_OVERFLOW_EN_0 : 1;
    uint8_t FIFO_WM_EN_4 : 1;
    uint8_t FIFO_WM_EN_3 : 1;
    uint8_t FIFO_WM_EN_2 : 1;
    uint8_t FIFO_WM_EN_1 : 1;
    uint8_t FIFO_WM_EN_0 : 1;
  } ICM_42688_INT_enable_t;

  typedef union
  {
    ICM_42688_axis3bit16_t raw;
    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } axes;
  } ICM_42688_axis3named_t;

  typedef struct
  {
    ICM_42688_axis3named_t acc;
    ICM_42688_axis3named_t gyr;
    ICM_42688_axis3named_t mag;
    union
    {
      ICM_42688_axis1bit16_t raw;
      int16_t val;
    } tmp;
    ICM_42688_fss_t fss; // Full-scale range settings for this measurement
    uint8_t magStat1;
    uint8_t magStat2;
  } ICM_42688_agt_t;

  typedef struct
  {
    ICM_42688_Status_e (*write)(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
    ICM_42688_Status_e (*read)(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
    // void				(*delay)(uint32_t ms);
    void *user;
  } ICM_42688_Serif_t;                      // This is the vtable of serial interface functions
  extern const ICM_42688_Serif_t NullSerif; // Here is a default for initialization (NULL)

  typedef struct
  {
    const ICM_42688_Serif_t *_serif; // Pointer to the assigned Serif (Serial Interface) vtable
    bool _dmp_firmware_available;    // Indicates if the DMP firmware has been included. It
    uint8_t _last_bank;              // Keep track of which bank was selected last - to avoid unnecessary writes
    uint8_t _last_mems_bank;         // Keep track of which bank was selected last - to avoid unnecessary writes
    int32_t _gyroSF;                 // Use this to record the GyroSF, calculated by inv_icm42688_set_gyro_sf
    int8_t _gyroSFpll;
    uint16_t _dataOutCtl1;            // Diagnostics: record the setting of DATA_OUT_CTL1
    uint16_t _dataOutCtl2;            // Diagnostics: record the setting of DATA_OUT_CTL2
    uint16_t _dataRdyStatus;          // Diagnostics: record the setting of DATA_RDY_STATUS
    uint16_t _motionEventCtl;         // Diagnostics: record the setting of MOTION_EVENT_CTL
    uint16_t _dataIntrCtl;            // Diagnostics: record the setting of DATA_INTR_CTL
  } ICM_42688_Device_t;               // Definition of device struct type

  ICM_42688_Status_e ICM_42688_init_struct(ICM_42688_Device_t *pdev); // Initialize ICM_42688_Device_t

  // ICM_42688_Status_e ICM_42688_Startup( ICM_42688_Device_t* pdev ); // For the time being this performs a standardized startup routine

  ICM_42688_Status_e ICM_42688_link_serif(ICM_42688_Device_t *pdev, const ICM_42688_Serif_t *s); // Links a SERIF structure to the device

  // use the device's serif to perform a read or write
  ICM_42688_Status_e ICM_42688_execute_r(ICM_42688_Device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len); // Executes a R or W witht he serif vt as long as the pointers are not null
  ICM_42688_Status_e ICM_42688_execute_w(ICM_42688_Device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len);

  // Single-shot I2C on Master IF
  ICM_42688_Status_e ICM_42688_i2c_controller_periph4_txn(ICM_42688_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);
  ICM_42688_Status_e ICM_42688_i2c_master_single_w(ICM_42688_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data);
  ICM_42688_Status_e ICM_42688_i2c_master_single_r(ICM_42688_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data);

  // Device Level
  ICM_42688_Status_e ICM_42688_set_bank(ICM_42688_Device_t *pdev, uint8_t bank);                                 // Sets the bank
  ICM_42688_Status_e ICM_42688_sw_reset(ICM_42688_Device_t *pdev);                                               // Performs a SW reset
  ICM_42688_Status_e ICM_42688_sleep(ICM_42688_Device_t *pdev, bool on);                                         // Set sleep mode for the chip
  ICM_42688_Status_e ICM_42688_low_power(ICM_42688_Device_t *pdev, bool on);                                     // Set low power mode for the chip
  ICM_42688_Status_e ICM_42688_get_who_am_i(ICM_42688_Device_t *pdev, uint8_t *whoami);                          // Return whoami in out prarmeter
  ICM_42688_Status_e ICM_42688_set_I2C_mode(ICM_42688_Device_t *pdev);                                               // TODO
  ICM_42688_Status_e ICM_42688_check_id(ICM_42688_Device_t *pdev);                                               // Return 'ICM_42688_Stat_Ok' if whoami matches ICM_42688_WHOAMI
  ICM_42688_Status_e ICM_42688_data_ready(ICM_42688_Device_t *pdev);                                             // Returns 'Ok' if data is ready

  // Interrupt Configuration
  //ICM_42688_Status_e ICM_42688_int_pin_cfg(ICM_42688_Device_t *pdev, ICM_42688_INT_PIN_CFG_t *write, ICM_42688_INT_PIN_CFG_t *read); // Set the INT pin configuration
  ICM_42688_Status_e ICM_42688_int_enable(ICM_42688_Device_t *pdev, ICM_42688_INT_enable_t *write, ICM_42688_INT_enable_t *read);    // Write and or read the interrupt enable information. If non-null the write operation occurs before the read, so as to verify that the write was successful

  // WoM Threshold Level Configuration
  //ICM_42688_Status_e ICM_42688_wom_threshold(ICM_42688_Device_t *pdev, ICM_42688_ACCEL_WOM_THR_t *write, ICM_42688_ACCEL_WOM_THR_t *read); // Write and or read the Wake on Motion threshold. If non-null the write operation occurs before the read, so as to verify that the write was successful

  // Internal Sensor Options
  //ICM_42688_Status_e ICM_42688_set_sample_mode(ICM_42688_Device_t *pdev, ICM_42688_InternalSensorID_bm sensors, ICM_42688_LP_CONFIG_CYCLE_e mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
  ICM_42688_Status_e ICM_42688_set_full_scale(ICM_42688_Device_t *pdev, ICM_42688_InternalSensorID_bm sensors);
  ICM_42688_Status_e ICM_42688_set_sample_rate(ICM_42688_Device_t *pdev, ICM_42688_InternalSensorID_bm sensors, ICM_42688_smplrt_t smplrt);

  // Interface Things
  ICM_42688_Status_e ICM_42688_i2c_master_passthrough(ICM_42688_Device_t *pdev, bool passthrough);
  ICM_42688_Status_e ICM_42688_i2c_master_enable(ICM_42688_Device_t *pdev, bool enable);
  ICM_42688_Status_e ICM_42688_i2c_master_reset(ICM_42688_Device_t *pdev);
  ICM_42688_Status_e ICM_42688_i2c_controller_configure_peripheral(ICM_42688_Device_t *pdev, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut);

  // Higher Level
  ICM_42688_Status_e ICM_42688_get_agt(ICM_42688_Device_t *pdev, ICM_42688_agt_t *p);

  // FIFO

  ICM_42688_Status_e ICM_42688_enable_FIFO(ICM_42688_Device_t *pdev, bool enable);
  ICM_42688_Status_e ICM_42688_reset_FIFO(ICM_42688_Device_t *pdev);
  ICM_42688_Status_e ICM_42688_setup_FIFO(ICM_42688_Device_t *pdev, bool enable);
  ICM_42688_Status_e ICM_42688_set_FIFO_mode(ICM_42688_Device_t *pdev, bool snapshot);
  ICM_42688_Status_e ICM_42688_get_FIFO_count(ICM_42688_Device_t *pdev, uint16_t *count);
  ICM_42688_Status_e ICM_42688_read_FIFO(ICM_42688_Device_t *pdev, uint8_t *data, uint8_t len);

  ICM_42688_Status_e inv_icm42688_read_data(ICM_42688_Device_t *pdev, icm_42688_data_t *data);
  ICM_42688_Status_e inv_icm42688_set_gyro_sf(ICM_42688_Device_t *pdev, unsigned char div, int gyro_level);




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ICM_42688_C_H_ */
