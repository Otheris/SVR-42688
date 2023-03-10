/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#ifndef SLIMEVR_ICM42688SENSOR_H_
#define SLIMEVR_ICM42688SENSOR_H_

#include <ICM_42688.h>
#include "sensor.h"
#include <arduino-timer.h> // Used for periodically saving bias
#include "madgwick.h"

class ICM42688Sensor : public Sensor
{
public:
    ICM42688Sensor(uint8_t id, uint8_t address, float rotation) : Sensor("ICM42688Sensor", IMU_ICM42688, id, address, rotation) {}
    ~ICM42688Sensor() override = default;
    void motionSetup() override final;
    void postSetup() override {
        this->lastData = millis();
    }

    void motionLoop() override final;
    void sendData() override final;

private:
    void calculateAccelerationWithoutGravity(Quat *quaternion);
    unsigned long lastData = 0;
    unsigned long lastDataSent = 0;
    int bias_save_counter = 0;
    bool newTap;
    int16_t rawAccel[3];
    float q[4] {1.0f, 0.0f, 0.0f, 0.0f};
    uint16_t now = 0, last = 0, loglast =0;   //micros() timers
    float deltat = 0;                  //loop time in seconds
   

    SlimeVR::Configuration::ICM42688CalibrationConfig m_Calibration;

    void connectSensor();
    void startMotionLoop();
    void checkSensorTimeout();
    void readRotation();
    void readDataToEnd();
    //void startCalibration(int calibrationType) override final;


#define OVERRIDEDMPSETUP true

    Timer<> timer = timer_create_default();

    ICM_42688_I2C imu;
    ICM_42688_Device_t pdev;
    icm_42688_data_t Data{};
    icm_42688_data_t DataTemp{};

    // TapDetector tapDetector;

};

#endif // SLIMEVR_ICM42688SENSOR_H_
