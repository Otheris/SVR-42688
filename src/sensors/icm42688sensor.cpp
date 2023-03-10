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
float angles[3];

#define TYPICAL_SENSITIVITY_LSB 65.5

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.);

#define ACCEL_SENSITIVITY_4G 8192.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE_4G = ((32768. / ACCEL_SENSITIVITY_4G) / 32768.) * EARTH_GRAVITY;

int counter = 1;
float avgs[200][8];
float currenttemperatureBand = 0;
int prevTemperatureBand = 0;
int totalCount =0;
int calibrationSampleSize = 5000;
int calibrationTemperature = 29;
float calibrations[27][6] = {
{-0.079048,  -0.006001,   -0.090166,    -0.000614,     -0.097352,     0.082721},
{-0.078921,  -0.006168,   -0.090470,    0.000143,      -0.097427,     0.091047},
{-0.078836,  -0.006407,   -0.090676,    -0.000273,     -0.096837,     0.095173},
{-0.079010,  -0.006700,   -0.091376,    -0.000297,     -0.096600,     0.097499},
{-0.078616,  -0.006370,   -0.091300,    0.000403,      -0.096050,     0.099685},
{-0.078478,  -0.006802,   -0.091744,    -0.001258,     -0.094962,     0.100877},
{-0.078641,  -0.007245,   -0.092134,    0.001091,      -0.094409,     0.102406},
{-0.078230,  -0.007712,   -0.092408,    0.000665,      -0.093485,     0.103886},
{-0.078554,  -0.007826,   -0.093410,    -0.000020,     -0.092953,     0.105349},
{-0.078164,  -0.007634,   -0.093565,    0.000164,      -0.092846,     0.106086},
{-0.078303,  -0.007821,   -0.094249,    -0.001181,     -0.092351,     0.107789},
{-0.078292,  -0.007621,   -0.094336,    -0.000818,     -0.091694,     0.108216},
{-0.077936,  -0.007708,   -0.094121,    -0.000719,     -0.091030,     0.109018},
{-0.077882,  -0.008030,   -0.094421,    -0.000258,     -0.090498,     0.109265},
{-0.077636,  -0.007867,   -0.094646,    -0.001898,     -0.089789,     0.110115},
{-0.077645,  -0.007959,   -0.094947,    -0.001430,     -0.089364,     0.110608},
{-0.077461,  -0.007902,   -0.095088,    -0.000650,     -0.088815,     0.111028},
{-0.077897,  -0.008335,   -0.095899,    -0.000274,     -0.088158,     0.111015},
{-0.077469,  -0.008589,   -0.095945,    0.000605,      -0.087856,     0.111084},
{-0.077273,  -0.008808,   -0.096003,    -0.000239,     -0.087659,     0.111147},
{-0.077253,  -0.009259,   -0.096410,    0.000342,      -0.087416,     0.111154},
{-0.076974,  -0.009365,   -0.096678,    0.000112,      -0.087486,     0.110606},
{-0.076468,  -0.010099,   -0.097033,    0.000681,      -0.087170,     0.109182},
{-0.075748,  -0.010248,   -0.096850,    0.000754,      -0.087080,     0.109673},
{-0.076495,  -0.009542,   -0.095897,    0.019677,      -0.087688,     0.112078},
{-0.075815,  -0.010209,   -0.097045,    -0.022182,     -0.087706,     0.110018},
{-0.075830,  -0.010210,   -0.096928,    -0.004860,     -0.087181,     0.110468} 
};


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
    readDataToEnd();
    //Serial.printf("read fifo");
    readRotation();
    //m_Logger.error("read rotation");
    checkSensorTimeout();
    //m_Logger.error("checked timeout and redo");
}

void ICM42688Sensor::readDataToEnd()
{
    ICM_42688_Status_e readStatus = imu.readData(&DataTemp); 
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
    //Quat defaultrot = Quat(0.7071067f,0.0f,0.0f,0.7071067f); a poor attempt to solve the x boundary issue
    //quaternion *= defaultrot;
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


// theres 3 accel parameters per direction because the offsets change depending on their directions. i need this in matplotlib
void ICM42688Sensor::readRotation()
{
    float Temperature = (((float)Data.Temperature)/132.48)+25;          //temperature as float
    int TemperatureBand = (int)(round(Temperature*2));                  //twice temperature rounded to nearest int for indexing in arrays

    float Gxyz[3], Axyz[3];

//*/
    float GxTempOffset;
    float GyTempOffset;
    float GzTempOffset;

    float AxTempOffset;
    float AyTempOffset;
    float AzTempOffset;



/*

there are two trypes of calibration for the accelerometer
you have already temperature normalised the values, which means that the drift should now be as independant of temperature as possible
so now you only really have the solid drift, which you want to correct by feeding data over time into magneto
it will output the bias matrices you need to use for each cardinal direction. look up a use guide and implementation guide 
once you have done this, maybe take a look at the quaternions to try and figure out why it keeps drifting back towards -90z for some reason.

*/


    if(TemperatureBand-58<0 || TemperatureBand > 450){ // when temp < 25 Degrees, temp value underflows. band > 450 catches this
        //Serial.printf("LOW TEMP");
        GxTempOffset = calibrations[0][0];
        GyTempOffset = calibrations[0][1];
        GzTempOffset = calibrations[0][2];

        AzTempOffset = calibrations[0][3];
        AzTempOffset = calibrations[0][4];
        AzTempOffset = calibrations[0][5];
    }else if(TemperatureBand-58>27){
        //Serial.printf("HIGH TEMP");
        GxTempOffset = calibrations[27][0];
        GyTempOffset = calibrations[27][1];
        GzTempOffset = calibrations[27][2];

        AxTempOffset = calibrations[27][3];
        AyTempOffset = calibrations[27][4];
        AzTempOffset = calibrations[27][5];
    }else{
        //Serial.printf("IN RANGE");
        GxTempOffset = calibrations[(int)((((float)TemperatureBand/2.)-29.)*2)][0];
        GyTempOffset = calibrations[(int)((((float)TemperatureBand/2.)-29.)*2)][1];
        GzTempOffset = calibrations[(int)((((float)TemperatureBand/2.)-29.)*2)][2];

        AxTempOffset = calibrations[(int)((((float)TemperatureBand/2.)-29.)*2)][3];
        AyTempOffset = calibrations[(int)((((float)TemperatureBand/2.)-29.)*2)][4];
        AzTempOffset = calibrations[(int)((((float)TemperatureBand/2.)-29.)*2)][5];
    }
/*
{-0.079102,  -0.005919,   -0.091216,    0.040545,      -0.094051,     0.071410},   27.000000,  23.0000,
{-0.078855,  -0.006109,   -0.089373,    -0.009103,     -0.095531,     0.066624},   28.500000,  101.000,
{-0.079048,  -0.006001,   -0.090166,    -0.000614,     -0.097352,     0.082721},   29.000000,  3891.00,
{-0.078921,  -0.006168,   -0.090470,    0.000143,      -0.097427,     0.091047},   29.500000,  5119.00,
{-0.078836,  -0.006407,   -0.090676,    -0.000273,     -0.096837,     0.095173},   30.000000,  5003.00,
{-0.079010,  -0.006700,   -0.091376,    -0.000297,     -0.096600,     0.097499},   30.500000,  5251.00,
{-0.078616,  -0.006370,   -0.091300,    0.000403,      -0.096050,     0.099685},   31.000000,  5485.00,
{-0.078478,  -0.006802,   -0.091744,    -0.001258,     -0.094962,     0.100877},   31.500000,  5639.00,
{-0.078641,  -0.007245,   -0.092134,    0.001091,      -0.094409,     0.102406},   32.000000,  6211.00,
{-0.078230,  -0.007712,   -0.092408,    0.000665,      -0.093485,     0.103886},   32.500000,  6366.00,
{-0.078554,  -0.007826,   -0.093410,    -0.000020,     -0.092953,     0.105349},   33.000000,  7085.00,
{-0.078164,  -0.007634,   -0.093565,    0.000164,      -0.092846,     0.106086},   33.500000,  6636.00,
{-0.078303,  -0.007821,   -0.094249,    -0.001181,     -0.092351,     0.107789},   34.000000,  8206.00,
{-0.078292,  -0.007621,   -0.094336,    -0.000818,     -0.091694,     0.108216},   34.500000,  8904.00,
{-0.077936,  -0.007708,   -0.094121,    -0.000719,     -0.091030,     0.109018},   35.000000,  8967.00,
{-0.077882,  -0.008030,   -0.094421,    -0.000258,     -0.090498,     0.109265},   35.500000,  10620.0,
{-0.077636,  -0.007867,   -0.094646,    -0.001898,     -0.089789,     0.110115},   36.000000,  11757.0,
{-0.077645,  -0.007959,   -0.094947,    -0.001430,     -0.089364,     0.110608},   36.500000,  12405.0,
{-0.077461,  -0.007902,   -0.095088,    -0.000650,     -0.088815,     0.111028},   37.000000,  15298.0,
{-0.077897,  -0.008335,   -0.095899,    -0.000274,     -0.088158,     0.111015},   37.500000,  16783.0,
{-0.077469,  -0.008589,   -0.095945,    0.000605,      -0.087856,     0.111084},   38.000000,  19013.0,
{-0.077273,  -0.008808,   -0.096003,    -0.000239,     -0.087659,     0.111147},   38.500000,  20709.0,
{-0.077253,  -0.009259,   -0.096410,    0.000342,      -0.087416,     0.111154},   39.000000,  27970.0,
{-0.076974,  -0.009365,   -0.096678,    0.000112,      -0.087486,     0.110606},   39.500000,  34451.0,
{-0.076468,  -0.010099,   -0.097033,    0.000681,      -0.087170,     0.109182},   40.000000,  45282.0,
{-0.075748,  -0.010248,   -0.096850,    0.000754,      -0.087080,     0.109673},   40.500000,  32095.0,
{-0.076495,  -0.009542,   -0.095897,    0.019677,      -0.087688,     0.112078},   41.000000,  16.0000,
{-0.075815,  -0.010209,   -0.097045,    -0.022182,     -0.087706,     0.110018},   42.000000,  83.0000,
{-0.075830,  -0.010210,   -0.096928,    -0.004860,     -0.087181,     0.110468}   42.500000,  566.000
*/
//*/
    //Serial.printf("\rtemp: %i", TemperatureBand-58);
    //fflush(stdout);

    Gxyz[0] = ((float)Data.Raw_Gyro_X / 65.5 / 6.)-2*(GxTempOffset*0.985); //+61
    Gxyz[1] = ((float)Data.Raw_Gyro_Y / 65.5 / 6.)-2*(GyTempOffset*0.985); //+1
    Gxyz[2] = ((float)Data.Raw_Gyro_Z / 65.5 / 6.)-2*(GzTempOffset*0.985); //+72.5S

    Axyz[0] = (((float)Data.Raw_Accel_X / 8192)*EARTH_GRAVITY)-AxTempOffset;
    Axyz[1] = (((float)Data.Raw_Accel_Y / 8192)*EARTH_GRAVITY)-AyTempOffset;
    Axyz[2] = (((float)Data.Raw_Accel_Z / 8192)*EARTH_GRAVITY)-AzTempOffset;


    angles[0]+=Gxyz[0];
    angles[1]+=Gxyz[1];
    angles[2]+=Gxyz[2];
    /*
    
    ok so
    hook up to python matplotlib
    watch the graphics
    figure out a good manual tuning
    then
    starting from 25degrees or whatever
    turn it on and log the deviation every .25 degrees, like below SHOULD do???


    BELOW DID NOT WORK

    algo to get data to implement similar filtering as bmi160
    basically

    tempdiff = current temp - calibrationtemprerature

    raw data + (base offset to start from + offsetbytemperaturelist[currenttemperatureasint])
    
    so we need to populate the offset by temperature list

    we can do this by:

    running a test over many samples where:
     - there is an array of floats
     - as the samples come in, they are added to the correct float bucket to try and get a true offset at that location
     - this becomes the offset that we add to the measurement, again, depending on temp. so.


    avgs is the array

//*/
/*/
    //Serial.printf("\rtemp: %i", TemperatureBand);

    
    avgs[TemperatureBand][0]+=Gxyz[0];
    avgs[TemperatureBand][1]+=Gxyz[1];
    avgs[TemperatureBand][2]+=Gxyz[2];

    avgs[TemperatureBand][3]+=Axyz[0];
    avgs[TemperatureBand][4]+=Axyz[1];
    avgs[TemperatureBand][5]+=Axyz[2];

    avgs[TemperatureBand][6]++;
    avgs[TemperatureBand][7]=1;

    if(counter==calibrationSampleSize){
        Serial.printf("\n");
        for(int i=0;i<calibrationSampleSize;i++){
            if(avgs[i][7]==1){
                Serial.printf("gx: %f, gy: %f, gz: %f, ax: %f, ay: %f, az: %f, T: %f, sz: %f\n", 
                    avgs[i][0]/avgs[i][6], avgs[i][1]/avgs[i][6], avgs[i][2]/avgs[i][6], 
                    avgs[i][3]/avgs[i][6], avgs[i][4]/avgs[i][6], avgs[i][5]/avgs[i][6],
                    (float)i/2, avgs[i][6]);
            }
        }
        counter=1;
    }else{
        if(counter%1000==0){
            Serial.printf("\rprogress: %i / %i", counter, calibrationSampleSize);
            fflush(stdout);
        }
    }
    counter++;
//*/
    Serial.printf("\rgx:%f, \tgy:%f, \tgz:%f \tax:%f \tay:%f \taz:%f \tt:%f", angles[0],  angles[1],  angles[2], Axyz[0], Axyz[1],  Axyz[2], Temperature);
    //Serial.printf(/*"\rgx:%f, gy:%f, gz:%f */"\rax:%f ay:%f az:%f t:%f #T COMING FROM:%i          ", /*angles[0],  angles[1],  angles[2],*/ Axyz[0], Axyz[1],  Axyz[2], Temperature, (int)((((float)TemperatureBand/2.)-29.)*2));
    fflush(stdout);

    now = micros();
    deltat = now - last; //seconds since last update
    last = now;
    madgwickQuaternionUpdate(q, Axyz[0], Axyz[1],  Axyz[2], Gxyz[0],  Gxyz[1],  Gxyz[2], 0.001);
    quaternion.set(-q[2], q[1], q[3], q[0]);


    //Serial.printf("\rx:%f,\t y:%f,\t z:%f,\t w:%f", quaternion.x, quaternion.y, quaternion.z, quaternion.w);
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



