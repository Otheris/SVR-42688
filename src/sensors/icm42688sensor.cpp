/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 S.J. Remington & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#include "icm42688sensor.h"
#include "calibration.h"
#include <i2cscan.h>
#include "network/network.h"
#include "GlobalVars.h"
#include <math.h>

// seconds after previous save (from start) when calibration (DMP Bias) data will be saved to NVS. Increments through the list then stops; to prevent unwelcome eeprom wear.
int bias_save_periods[] = { 120, 180, 300, 600, 600 }; // 2min + 3min + 5min + 10min + 10min (no more saves after 30min)



#define TYPICAL_SENSITIVITY_LSB 16.4
// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

#define ACCEL_SENSITIVITY_16G 2048.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE_16G = ((32768. / ACCEL_SENSITIVITY_16G) / 32768.) * EARTH_GRAVITY;

void ICM42688Sensor::motionSetup()
{
    connectSensor();
    startMotionLoop();
}

void ICM42688Sensor::motionLoop()
{
#if ENABLE_INSPECTION
    {
        (void)imu.getagt();

        float rX = imu.gyrX();
        float rY = imu.gyrY();
        float rZ = imu.gyrZ();

        float aX = imu.accX();
        float aY = imu.accY();
        float aZ = imu.accZ();

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255);
    }
#endif
    timer.tick();
    //Serial.printf("start loop");
    readFIFOToEnd();
    //Serial.printf("read fifo");
    readRotation();
    //m_Logger.error("read rotation");
    checkSensorTimeout();
    //m_Logger.error("checked timeout and redo");
}

void ICM42688Sensor::readFIFOToEnd()
{
    ICM_42688_Status_e readStatus = imu.readDataFromFIFO(&DataTemp); 
    #ifdef DEBUG_SENSOR
    {
        //m_Logger.trace("error: %0x", readStatus);
        //Serial.printf("\rx: %X,", Data.Header);
        //fflush(stdout);
    }
    #endif

    if(readStatus == ICM_42688_Stat_Ok)
    {
        
        Data = DataTemp;
        //readFIFOToEnd();
    }
}

void ICM42688Sensor::sendData()
{
    if(newData && lastDataSent + 7 < millis())
    {
        lastDataSent = millis();
        newData = false;

        #if(USE_6_AXIS)
        {
            Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, 0, sensorId);
        }
        #else
        {
            Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, dmpData.Quat9.Data.Accuracy, sensorId);
        }
        #endif

        #if SEND_ACCELERATION
        {
            Network::sendAccel(acceleration, sensorId);
        }
        #endif
    }
}

void ICM42688Sensor::connectSensor()
{
    #ifdef DEBUG_SENSOR
        imu.enableDebugging(Serial);
    #endif
    // SparkFun_ICM-42688_ArduinoLibrary only supports 0x68 or 0x69 via boolean, if something else throw a error
    boolean isOnSecondAddress = false;

    if (addr == 0x68) { // IMU default is 0x68 when nothing is connected to AD0
        isOnSecondAddress = false;
    } else if (addr == 0x69){
        isOnSecondAddress = true;
    } else {
        m_Logger.fatal("I2C Address not supported by ICM-42688 library: 0x%02x", addr);
        return;
    }
    m_Logger.debug("Beginning IMU setup");
    ICM_42688_Status_e imu_err = imu.begin(Wire, isOnSecondAddress);
    if (imu_err != ICM_42688_Stat_Ok) {
        m_Logger.fatal("Can't connect to ICM-42688 at address 0x%02x, error code: 0x%02x", addr, imu_err);
        ledManager.pattern(50, 50, 200);
        return;
    }
}

void ICM42688Sensor::startMotionLoop()
{
    lastData = millis();
    working = true;
}

void ICM42688Sensor::checkSensorTimeout()
{
    if(lastData + 5000 < millis()) {
        working = false;
        lastData = millis();
        m_Logger.error("Sensor timeout I2C Address 0x%02x", addr);
        Network::sendError(1, this->sensorId);
    }
}
void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    //Serial.printf("\r");
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            Serial.printf("%u", byte);
        }
    }
    Serial.printf("\t");
}
void ICM42688Sensor::readRotation()
{
    int16_t ax, ay, az, gx, gy, gz;
    ax = Data.Raw_Accel_X;
    ay = Data.Raw_Accel_Y;
    az = Data.Raw_Accel_Z;

    gx = Data.Raw_Gyro_X;
    gy = Data.Raw_Gyro_Y;
    gz = Data.Raw_Gyro_Z;

    float Gxyz[3], Axyz[3];

    Gxyz[0] = (float)gx * GSCALE;
    Gxyz[1] = (float)gy * GSCALE;
    Gxyz[2] = (float)gz * GSCALE;

    float thing = 0.0625;
    Axyz[0] = (float)ax * thing;
    Axyz[1] = (float)ay * thing;
    Axyz[2] = (float)az * thing;

    m_Logger.debug("\rData Header:%x, Timestamp:%d, Accel_X:%d", Data.Header, Data.Time, Data.Raw_Accel_X);
    fflush(stdout);
    mahonyQuaternionUpdate(q, Gxyz[0],  Gxyz[1],  Gxyz[2], Axyz[0], Axyz[1],  Axyz[2], 0.0000625);
    quaternion.set(-q[2], q[1], q[3], q[0]);        
    //fflush(stdout);

    newData = true;
    lastData = millis();
}

void ICM42688Sensor::calculateAccelerationWithoutGravity(Quat *quaternion)
{
    #if SEND_ACCELERATION
    {
        if((Data.Header & 0x8000) > 0)
        {
            this->acceleration[0] = (float)this->Data.Raw_Accel_X;
            this->acceleration[1] = (float)this->Data.Raw_Accel_Y;
            this->acceleration[2] = (float)this->Data.Raw_Accel_Z;

            // get the component of the acceleration that is gravity
            float gravity[3];
            gravity[0] = 2 * ((-quaternion->x) * (-quaternion->z) - quaternion->w * quaternion->y);
            gravity[1] = -2 * (quaternion->w * (-quaternion->x) + quaternion->y * (-quaternion->z));
            gravity[2] = quaternion->w * quaternion->w - quaternion->x * quaternion->x - quaternion->y * quaternion->y + quaternion->z * quaternion->z;

            // subtract gravity from the acceleration vector
            this->acceleration[0] -= gravity[0] * ACCEL_SENSITIVITY_8G;
            this->acceleration[1] -= gravity[1] * ACCEL_SENSITIVITY_8G;
            this->acceleration[2] -= gravity[2] * ACCEL_SENSITIVITY_8G;

            // finally scale the acceleration values to mps2
            this->acceleration[0] *= ASCALE_8G;
            this->acceleration[1] *= ASCALE_8G;
            this->acceleration[2] *= ASCALE_8G;
        }
    }
    #endif
}



