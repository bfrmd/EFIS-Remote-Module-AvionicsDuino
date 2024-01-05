/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                                                 EFIS_Remote_Module_Avionicsduino
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  EFIS_Remote_Module_Avionicsduino is free software:
  MIT License (MIT)
  
  Copyright (c) 2023 AvionicsDuino - benjamin.fremond@avionicsduino.com
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

// Le module Magnétomètre, capteur de température et d'humidité relative est déporté en bout d'aile, le plus loin possible des interférences magnétiques.
// Il communique par le CAN Bus avec le module EFIS du tableau de bord. Il est animé par une carte Teensy 4.0.
// Le magnétomètre est un capteur triaxial LIS3MDL de STMicroelectronics, monté sur une carte Adafruit ID 4099. 
// Les indications brutes du LIS3MDL sont compensées en roll et pitch, grâce aux indications correspondantes envoyées via le CAN Bus par l'EFIS. Ce dernier reçoit donc en retour un cap magnétique compensé.
// Pour la température et l'humidité relative, le cateur est un Adafruit/Sensirion SHT30 (PRODUCT ID: 4099), "Weather proof".
// ************************************************************************************************************************************************************************************************************************************ 

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                Connexions physiques des capteurs et composants externes avec la carte Teensy 4.0
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// Adafruit/Sensirion SHT30, avec la bibliothèque de Rob Tillaart (URL: https://github.com/RobTillaart/SHT31)
// ---------------SHT30------------------------Teensy 4.0--------------------------------------------------SUB D 9 -------
//                GND     ------------------->   GND   -----------------------------------------------> Pin 8
//            Alim +3.3v  ------------------->   + 3.3 volts -----------------------------------------> Pin 9
//                SDA   --------------------->   SDA0 (pin 18) pas besoin de résistance pull up ------> Pin 1 (fil vert)
//                SCL   --------------------->   SCL0 (pin 19) pas besoin de résistance pull up ------> Pin 6 (fil jaune)
//           Adresse : 0x44

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// Magnétomètre, avec la bibliothèque Adafruit (https://github.com/adafruit/Adafruit_LIS3MDL)
// ---------------SHT30------------------------Teensy 4.0------------------------
//                GND     ------------------->   GND
//            Alim +3.3v  ------------------->   + 3.3 volts
//                SDA   --------------------->   SDA1 (pin 17) pas besoin de résistance pull up
//                SCL   --------------------->   SCL1 (pin 16) pas besoin de résistance pull up
//                INT   --------------------->   Pin 15
//                DDRY  --------------------->   Pin 14
//           Adresse : 0x1C

// Un transceiver CAN MCP 2562 EP (CAN 2.0) est connecté au bus CAN1, avec la library FlexCAN_T4 : https://github.com/tonton81/FlexCAN_T4
//-------------- MCP 2562 ------------------------ Teensy 4.0 ------------------------
//             Pin1 TXD ------------------------> Pin 22 CTX1
//             Pin2 GND ------------------------> GND
//             Pin3 VDD ------------------------> + 5V
//             Pin4 RXD ------------------------> Pin 23 CRX1
//             Pin5 Vio ------------------------> + 3.3V
//             Pin6 CAN LOW --------------------> sortie CanBus
//             Pin7 CAN HIGH -------------------> Sortie CanBus
//             Pin8 Standby --------------------> GND

// L'ensemble des entrées et sorties de ce module se fait via une prise Sub-D 9 soudée sur le circuit imprimé : 4 broches pour le capteur température-humidité, et 4 broches pour le CAN-Bus

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  Inclusions des bibliothèques
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "Wire.h"
#include "SHT31.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <FlexCAN_T4.h> 

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Création des objets
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

SHT31 sht;
Adafruit_LIS3MDL lis3mdl;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_Module_Distant; 

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Déclarations des variables et constantes globales
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SHT31_ADDRESS   0x44
#define LIS3MDL_ADDRESS 0x1C
float magx, magy, magz;

// Données correctives issues de la calibration fer dur et fer doux du magnétomètre utilisé. Ces données sont applicables à des mesures en µTesla.
// Ces données sont issues de la calibration avec le logiciel MotionCal de Paul Stoffregen (http://www.pjrc.com/teensy/beta/imuread/MotionCal.exe)
float MagOffset[3] = {-29.8, 1.93, -5.67}; // Offsets pour les axes x, y et z
float mCal[3][3] = 
{
  {+1.002, +0.034, +0.003},
  {+0.034, +0.978, -0.014},
  {+0.003, -0.014, +1.021}
};


float magxc, magyc, magzc, Xh, Yh, capmagnetique, valcapmagnetiqueFiltreePrecedente;
uint32_t capmagnetiqueEntier;
volatile float pitch=0.0, roll=0.0;
float oat, rh, valoatFiltreePrecedente, valrhFiltreePrecedente;

CAN_message_t msgTemperature;
CAN_message_t msgRelativeHumidity;
CAN_message_t msgCapMagnetique;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           SETUP
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

// ***************************************************************************************** Initialisation du CAN bus  *****************************************************************************************************
  CAN_Module_Distant.begin();
  CAN_Module_Distant.setBaudRate(500000);
  CAN_Module_Distant.setMaxMB(16);
  CAN_Module_Distant.enableFIFO();
  CAN_Module_Distant.enableFIFOInterrupt();
  CAN_Module_Distant.onReceive(FIFO,canSniff);
  CAN_Module_Distant.mailboxStatus();
  delay(1000);  
  
// ********************************************************************************* Initialisation du capteur température/humidité  *******************************************************************************************
  Wire.begin();
  sht.begin(SHT31_ADDRESS);
  Wire.setClock(100000);

  uint16_t stat = sht.readStatus();
  Serial.print(stat, HEX);
  Serial.println();

  sht.read();  
  valoatFiltreePrecedente = sht.getTemperature();
  valrhFiltreePrecedente =  sht.getHumidity();

// ********************************************************************************* Initialisation du capteur magnétomètre  *******************************************************************************************
  if (! lis3mdl.begin_I2C(LIS3MDL_ADDRESS, &Wire1)) 
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
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                             LOOP
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{
// ********************************************************************************** Lecture du capteur de température et humidité ***********************************************************************************************
  sht.read();         
  oat = sht.getTemperature();
  rh =  sht.getHumidity();
  oat = filtrageRII (valoatFiltreePrecedente, oat, 0.03); 
  valoatFiltreePrecedente = oat; 
  rh = filtrageRII (valrhFiltreePrecedente, rh, 0.03); 
  valrhFiltreePrecedente = rh; 
 
// ********************************************************************************** Lecture du magnétomètre ***********************************************************************************************
  
  lis3mdl.read();      
  magx = lis3mdl.x/68.42; 
  magy = lis3mdl.y/68.42; 
  magz = lis3mdl.z/68.42;
   
// Calcul du produit des matrices (correction fer dur et fer doux)
  magxc = mCal[0][0]*(magx-MagOffset[0])+ mCal[0][1]*(magy-MagOffset[1]) + mCal[0][2]*(magz-MagOffset[2]);
  magyc = mCal[1][0]*(magx-MagOffset[0])+ mCal[1][1]*(magy-MagOffset[1]) + mCal[1][2]*(magz-MagOffset[2]);
  magzc = mCal[2][0]*(magx-MagOffset[0])+ mCal[2][1]*(magy-MagOffset[1]) + mCal[2][2]*(magz-MagOffset[2]);

// Tilt compensation (compensation en roulis et tangage)
  Xh = magxc * cos(pitch) + magzc * sin(pitch);
  Yh = magxc * sin(roll) * sin(pitch) + magyc * cos(roll) - magzc * sin(roll) * cos(pitch);
  
// Calcul du cap magnétique
  capmagnetique = atan2(Yh,Xh)*(180/PI); 
  if ((capmagnetique<0) && (valcapmagnetiqueFiltreePrecedente<90)) 
    {
      valcapmagnetiqueFiltreePrecedente = 360;
    }
  if ((capmagnetique>0) && (valcapmagnetiqueFiltreePrecedente>270))
    {
      valcapmagnetiqueFiltreePrecedente = 0;
    } 
  if (capmagnetique<0) capmagnetique+=360;
  capmagnetique = filtrageRII (valcapmagnetiqueFiltreePrecedente, capmagnetique, 0.07);
  valcapmagnetiqueFiltreePrecedente = capmagnetique;      
  capmagnetiqueEntier = uint32_t(capmagnetique+0.5); 

// ********************************************************************************* Envoi des données à l'EFIS via le CAN bus ***********************************************************************************************

// Assignation des ID et des longueurs des messages
  msgTemperature.id =      42;
  msgRelativeHumidity.id = 43;
  msgCapMagnetique.id =    44;

  msgTemperature.len =      4;
  msgRelativeHumidity.len = 4;
  msgCapMagnetique.len =    4;  

// Chargement des buffers d'envoi
  for (uint8_t i = 0; i < 4; i++ ) 
   {
      msgTemperature.buf[i] = ((byte*) &oat)[i];
      msgRelativeHumidity.buf[i] = ((byte*) &rh)[i];
      msgCapMagnetique.buf[i] = ((byte*) &capmagnetiqueEntier)[i];
   }
// Envoi effectif
  CAN_Module_Distant.write(msgTemperature);
  CAN_Module_Distant.write(msgRelativeHumidity);
  CAN_Module_Distant.write(msgCapMagnetique);
// ******************************************************************************* Fin envoi des données au CAN bus *****************************************************************
  delay(50); //on envoie les données à environ 20 Hz
}
//********************************************************************************** Fin de la boucle infinie Loop ***********************************************************************************************

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  ISR du CAN bus : réception des données en provenance de l'unité centrale
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void canSniff(const CAN_message_t &msg) 
{
  switch (msg.id)
      {
        case 52:// roll est envoyé par l'EFIS en radians (filtré).
            {
            roll =  *(float*)(msg.buf);
            break;
            }
        case 53: // pitch est envoyé par l'EFIS en degrés, filtré, donc à convertir en radians pour les calculs.
            { 
            pitch =  -1*(*(float*)(msg.buf))*PI/180;
            break;
            }
        default:
            break;
      }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonction utilisée pour le filtrage des données brutes issues des capteurs
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Fonction de filtre à réponse impulsionnelle infinie
float filtrageRII (float valeurFiltreePrecedente, float valeurCourante , float coeffFiltrageRII)
{
  return valeurFiltreePrecedente  * (1-coeffFiltrageRII) + valeurCourante * coeffFiltrageRII ;
}
