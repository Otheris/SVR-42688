#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define icm_42688_Header_Bytes 1
#define icm_42688_Raw_Accel_Bytes 6
#define icm_42688_Raw_Gyro_Bytes 6
#define icm_42688_Temperature_Bytes 1
#define icm_42688_Timestamp_Bytes 2
#define icm_42688_Raw_Accel_EX_Bytes 3
#define icm_42688_Maximum_Bytes 20 // The most bytes we will attempt to read from the FIFO in one go
#define icm_42688_Packet_Bytes 16
#define icm_42688_EXT_Packet_Bytes 20


  typedef struct {
    uint8_t HEADER_ODR_GYRO : 1;
    uint8_t HEADER_ODR_ACCEL : 1;
    uint16_t HEADER_TIMESTAMP_FSYNC  : 2;
    uint8_t HEADER_20 : 1;
    uint8_t HEADER_GYRO : 1;
    uint8_t HEADER_ACCEL : 1;
    uint8_t HEADER_MSG : 1;
  }ICM_42688_HEADER_t;
/*/
typedef struct
  {
    uint8_t Header;
    
    int16_t Raw_Accel_X;
    int16_t Raw_Accel_Y;
    int16_t Raw_Accel_Z;

    int16_t Raw_Gyro_X;
    int16_t Raw_Gyro_Y;
    int16_t Raw_Gyro_Z;

    uint16_t Temperature;

    uint16_t Time;

    uint8_t Ext_1; 
    uint8_t Ext_2; 
    uint8_t Ext_3; 

  } icm_42688_data_t;
//*/
  typedef struct
  {
    uint8_t Header;
    
    int16_t Raw_Accel_X;
    int16_t Raw_Accel_Y;
    int16_t Raw_Accel_Z;

    int16_t Raw_Gyro_X;
    int16_t Raw_Gyro_Y;
    int16_t Raw_Gyro_Z;

    uint16_t Temperature;

    uint16_t Time;

    uint8_t Ext_1; 
    uint8_t Ext_2; 
    uint8_t Ext_3; 

  } icm_42688_data_t;


#ifdef __cplusplus
}
#endif /* __cplusplus */