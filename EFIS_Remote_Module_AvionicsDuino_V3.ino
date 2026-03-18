/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                                                 EFIS_Remote_Module_Avionicsduino_V3
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  EFIS_Remote_Module_Avionicsduino_V3 is free software:
  MIT License (MIT)
  
  Copyright (c) 2026 AvionicsDuino - benjamin.fremond@avionicsduino.com
  https://avionicsduino.com/index.php/en/digital-compass/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
 *****************************************************************************************************************************/

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  External libraries inclusions
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <SHT31.h>             // https://github.com/RobTillaart/SHT31
#include <Adafruit_LIS3MDL.h>  //  https://github.com/adafruit/Adafruit_LIS3MDL
#include <Adafruit_Sensor.h>
#include <FlexCAN_T4.h>       //  https://github.com/tonton81/FlexCAN_T4
#include <TeensyTimerTool.h>  // https://github.com/luni64/TeensyTimerTool
using namespace TeensyTimerTool;

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                           Objects instantiation
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
SHT31 sht;
Adafruit_LIS3MDL lis3mdl;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_remoteModule;
PeriodicTimer timerReadSensors(TCK);
PeriodicTimer timer5Hz(TCK);
PeriodicTimer timer10Hz(TCK);

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Declarations of global variables and constants
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SHT31_ADDRESS 0x44
#define LIS3MDL_ADDRESS 0x1C
float magx, magy, magz;

// Corrective data from the hard and soft iron calibration of the magnetometer used. This data is applicable to measurements in µTesla.
// This data was obtained from calibration using Paul Stoffregen's MotionCal software (http://www.pjrc.com/teensy/beta/imuread/MotionCal.exe)
// This data is valid exclusively for the magnetometer module with which it was obtained through calibration.
// Calibration data obtained through in-flight data recording
float MagOffset[3] = { -31.07, 4.97, 0.96 };  // Offsets for x, y et z axis
float mCal[3][3] = {
  { +0.982, +0.048, -0.016 },
  { +0.048, +0.994, +0.005 },
  { -0.016, +0.005, +1.027 }
};

float magxc, magyc, magzc, Xh, previousFilteredXhValue, Yh, previousFilteredYhValue, magHeading, unfilteredMagHeadingValue;
volatile float pitch = 0.0, roll = 0.0;
float oat, rh, previousFilteredOatValue, previousFilteredRhValue;

CAN_message_t msgTemperature;
CAN_message_t msgRelativeHumidity;
CAN_message_t msgmagHeading;
CAN_message_t msgMagxMagy;
CAN_message_t msgMagz;

bool flagReadSensors = true;
bool flag5Hz = true;
bool flag10Hz = true;
bool debugPrint = false;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           SETUP
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  //pinMode(0, OUTPUT); //for debugging purposes
  //pinMode(1, OUTPUT);
  //pinMode(2, OUTPUT);
  Serial.begin(115200);
  delay(500);

  // ***************************************************************************************** CAN bus initialization  *****************************************************************************************************
  CAN_remoteModule.begin();
  CAN_remoteModule.setBaudRate(500000);
  CAN_remoteModule.setMaxMB(16);
  CAN_remoteModule.enableFIFO();
  CAN_remoteModule.enableFIFOInterrupt();
  CAN_remoteModule.onReceive(FIFO, canSniff);
  CAN_remoteModule.mailboxStatus();

  // Assigning CAN message IDs and lengths
  msgTemperature.id = 42;
  msgRelativeHumidity.id = 44;
  msgmagHeading.id = 28;
  msgMagxMagy.id = 43;
  msgMagz.id = 45;

  msgTemperature.len = 4;
  msgRelativeHumidity.len = 4;
  msgmagHeading.len = 4;
  msgMagxMagy.len = 8;
  msgMagz.len = 4;

  delay(100);

  // ********************************************************************************* Initializing the temperature/humidity sensor *******************************************************************************************
  Wire.begin();
  Wire.setClock(100000);
  sht.begin();
  sht.read();
  previousFilteredOatValue = sht.getTemperature();
  previousFilteredRhValue = sht.getHumidity();

  // ********************************************************************************* Magnetometer sensor initialization *******************************************************************************************
  if (!lis3mdl.begin_I2C(LIS3MDL_ADDRESS, &Wire1))
    {
      Serial.println("Failed to find LIS3MDL chip");
    }
  else Serial.println("LIS3MDL Found!");
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, true, false, true);


  // ********************************************************************************* Timers initializations *******************************************************************************************
  timerReadSensors.begin(setflagReadSensors, 6667us);  // This defines a sensor reading frequency of 150 Hz
  timer5Hz.begin(setflag5Hz, 200ms);                   // Some data is sent at 5 Hz to the CAN bus, therefore every 200 ms
  timer10Hz.begin(setflag10Hz, 100ms);                 // Other data is sent at 10 Hz to the CAN bus, therefore every 100 ms
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                             LOOP
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{
  //digitalWrite(0, !digitalRead(0)); // for debugging purposes
  if (flagReadSensors)
    {
      flagReadSensors = false;

      // ********************************************************************************** Temperature and humidity sensor reading ***********************************************************************************************
      sht.read();
      oat = sht.getTemperature();
      rh = sht.getHumidity();
      oat =iirFilter(previousFilteredOatValue, oat, 0.03);
      previousFilteredOatValue = oat;
      rh =iirFilter(previousFilteredRhValue, rh, 0.03);
      previousFilteredRhValue = rh;

      // ********************************************************************************** Magnetometer reading ***********************************************************************************************
      lis3mdl.read();
      magx = lis3mdl.x / 68.42; // Conversion of raw digital data into µTesla
      magy = lis3mdl.y / 68.42; // See LIS3MDL datasheet p 4 magnetic characteristics (https://www.st.com/resource/en/datasheet/lis3mdl.pdf)
      magz = lis3mdl.z / 68.42;

      // Calculation of the matrix product (correction for hard iron and soft iron)
      magxc = mCal[0][0] * (magx - MagOffset[0]) + mCal[0][1] * (magy - MagOffset[1]) + mCal[0][2] * (magz - MagOffset[2]);
      magyc = mCal[1][0] * (magx - MagOffset[0]) + mCal[1][1] * (magy - MagOffset[1]) + mCal[1][2] * (magz - MagOffset[2]);
      magzc = mCal[2][0] * (magx - MagOffset[0]) + mCal[2][1] * (magy - MagOffset[1]) + mCal[2][2] * (magz - MagOffset[2]);

      // Tilt compensation (roll and pitch compensation)
      Xh = magxc * cos(pitch) + magzc * sin(pitch);
      Yh = magxc * sin(roll) * sin(pitch) + magyc * cos(roll) - magzc * sin(roll) * cos(pitch);

      // Calculation of the unfiltered magnetic heading (for debugging purposes)
      unfilteredMagHeadingValue = atan2(Yh, Xh) * (180 / PI);
      if (unfilteredMagHeadingValue < 0) unfilteredMagHeadingValue += 360;


      Xh =iirFilter(previousFilteredXhValue, Xh, 0.005);
      previousFilteredXhValue = Xh;
      Yh =iirFilter(previousFilteredYhValue, Yh, 0.005);
      previousFilteredYhValue = Yh;

      // Calculation of the filtered magnetic heading
      magHeading = atan2(Yh, Xh) * (180 / PI);
      if (magHeading < 0) magHeading += 360;

      // ******************************************************** Sending data to the EFIS RDAM via the CAN bus and some debugging displays ***************************************************************************
      // 5 Hz transmissions
      if (flag5Hz)
        {
          //digitalWrite(1, !digitalRead(1));
          flag5Hz = false;
          for (uint8_t i = 0; i < 4; i++)
            {
              msgTemperature.buf[i] = ((byte *)&oat)[i];
              msgRelativeHumidity.buf[i] = ((byte *)&rh)[i];
              msgMagxMagy.buf[i] = ((byte *)&magx)[i];
              msgMagxMagy.buf[i + 4] = ((byte *)&magy)[i];
              msgMagz.buf[i] = ((byte *)&magz)[i];
            }
          CAN_remoteModule.write(msgTemperature);
          CAN_remoteModule.write(msgRelativeHumidity);
          CAN_remoteModule.write(msgMagxMagy);
          CAN_remoteModule.write(msgMagz);
        }

      // 10 Hz transmission and debug displays
      if (flag10Hz)
        {
          //digitalWrite(2, !digitalRead(2));
          flag10Hz = false;
          if (debugPrint)
            {
              Serial.printf("CapMagNoFilter : %5.1f%c", unfilteredMagHeadingValue, '\t');
              Serial.printf("CapMag : %5.1f%c", magHeading, '\t');
              Serial.printf("Humidity : %5.1f%c", rh, '\t');
              Serial.printf("OAT : %5.1f%c", oat, '\n');
            }

          for (uint8_t i = 0; i < 4; i++)
            {
              msgmagHeading.buf[i] = ((byte *)&magHeading)[i];
            }
          CAN_remoteModule.write(msgmagHeading);
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  CAN bus ISR: receiving data from the RDAM : roll and pitch, for tilt compensation
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void canSniff(const CAN_message_t &msg)
{
  switch (msg.id)
    {
      case 20:
        {
          roll = *(float *)(msg.buf);                         // roll is sent by the AHRS in radians (unfiltered).
          pitch = -1 * (*(float *)(msg.buf + 4)) * PI / 180;  // pitch is sent by the AHRS in degrees (unfiltered), so it needs to be converted back to radians for calculations.
          break;
        }

      default:
        break;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Other functions
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Infinite impulse response filter function
float iirFilter(float valeurFiltreePrecedente, float valeurCourante, float iirFilterCoefficient)
{
  return valeurFiltreePrecedente * (1 - iirFilterCoefficient) + valeurCourante * iirFilterCoefficient;
}

void setflagReadSensors()
{
  flagReadSensors = true;
}

void setflag5Hz()
{
  flag5Hz = true;
}

void setflag10Hz()
{
  flag10Hz = true;
}