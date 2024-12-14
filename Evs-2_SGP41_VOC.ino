#include <Arduino.h>
/*
 * I2C-Generator: 0.3.0
 * Yaml Version: 0.1.0
 * Template Version: 0.7.0-62-g3d691f9
 */
/*
 * 
 * Portions Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Portions Copyright (c) 2024, RJ Hegler Technologies, LLC
 * Other Copyright information can be found in the .h files listed below
 *
 *  For the ESP32-C3 SuperMini included in the kit, 
 *    choose "Nologo ESP32C3 Super Mini", 11/21/24 RJH 
 *  File -> Preferences -> Additiional Boards Manager URL's:
 *    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *  Required Librarys:
 *    Sensirion Gas Index Algorithm by Sensirion
 *    Sensirion I2C SGP41 by Sensirion 
 *    Sensirion I2C SHR4x by Sensirion
 *    DFRobot_ICP10111 by DFRobot
 *    U8g2 by Oliver
 *    
 */

#include <Wire.h>
#include <NOxGasIndexAlgorithm.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2cSht4x.h>
#include <VOCGasIndexAlgorithm.h>
#include <DFRobot_ICP10111.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); //U8g2lib has 'drivers' for quite a few other displays 

SensirionI2cSht4x sht4x;        // original code had: SensirionI2CSht4x (capitol C)
SensirionI2CSgp41 sgp41;
DFRobot_ICP10111 icp;

VOCGasIndexAlgorithm voc_algorithm;
NOxGasIndexAlgorithm nox_algorithm;

// Time in seconds needed for NOx conditioning
// WARNING: To avoid damage to the sensing material the conditioning must not
// exceed 10s!
uint16_t conditioning_s = 10;

char errorMessage[64];

void setup() {

    Serial.begin(115200);

    Wire.begin();

    sht4x.begin(Wire, 0x44);        //original code did not have the I2C address

    sgp41.begin(Wire);

    u8g2.begin();
    u8g2.enableUTF8Print();
     u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_7x14B_tf);
    //u8g2.setFont(u8g2_font_t0_11_tf);
    u8g2.drawStr(0,12, "SGP41 VOC Sensor");
    u8g2.drawStr(0,28, "Initializing...");
    u8g2.sendBuffer();					// transfer internal memory to the display
    

    
    delay(1000);

    int32_t index_offset;
    int32_t learning_time_offset_hours;
    int32_t learning_time_gain_hours;
    int32_t gating_max_duration_minutes;
    int32_t std_initial;
    int32_t gain_factor;

    voc_algorithm.get_tuning_parameters(
        index_offset, learning_time_offset_hours, learning_time_gain_hours,
        gating_max_duration_minutes, std_initial, gain_factor);

    Serial.println("\nVOC Gas Index Algorithm parameters");
    Serial.print("Index offset:\t");
    Serial.println(index_offset);
    Serial.print("Learing time offset hours:\t");
    Serial.println(learning_time_offset_hours);
    Serial.print("Learing time gain hours:\t");
    Serial.println(learning_time_gain_hours);
    Serial.print("Gating max duration minutes:\t");
    Serial.println(gating_max_duration_minutes);
    Serial.print("Std inital:\t");
    Serial.println(std_initial);
    Serial.print("Gain factor:\t");
    Serial.println(gain_factor);

    nox_algorithm.get_tuning_parameters(
        index_offset, learning_time_offset_hours, learning_time_gain_hours,
        gating_max_duration_minutes, std_initial, gain_factor);

    Serial.println("\nNOx Gas Index Algorithm parameters");
    Serial.print("Index offset:\t");
    Serial.println(index_offset);
    Serial.print("Learing time offset hours:\t");
    Serial.println(learning_time_offset_hours);
    Serial.print("Gating max duration minutes:\t");
    Serial.println(gating_max_duration_minutes);
    Serial.print("Gain factor:\t");
    Serial.println(gain_factor);
    Serial.println("");

     // Initialize ICP-10111 - barometric pressure sensor 
  if (icp.begin() != 0) {
    Serial.println("Failed to initialize ICP-10111 sensor");
  }
  else{
  Serial.println("ICP-10111 sensor found!");
  /**
      * @brief Set work mode
      * |------------------|--------------|-------------------|----------------------|
      * |       api        |   mode       |Conversion Time(ms)|Pressure RMS Noise(Pa)|
      * |icp.eLowPower     |  Low Power   |      1.8          |        3.2           |
      * |icp.eNormal       |  Normal      |      6.3          |        1.6           |
      * |icp.eLowNoise     |  Low Noise   |      23.8         |        0.8           |
      * |icp.eUltraLowNoise|  Ultra-low Noise |      94.5         |        0.4           |
      */

  icp.setWorkPattern(icp.eNormal);
  }
  
}

void loop() {
    uint16_t error;
    float humidity = 0;     // %RH
    float temperature = 0;  // degreeC
    uint16_t temperatureTicks;
    uint16_t humidityTicks;
    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;
    uint16_t defaultCompensationRh = 0x8000;  // in ticks as defined by SGP41
    uint16_t defaultCompensationT = 0x6666;   // in ticks as defined by SGP41
    uint16_t compensationRh = 0;              // in ticks as defined by SGP41
    uint16_t compensationT = 0;               // in ticks as defined by SGP41

    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_7x14B_tf);
    //u8g2.setFont(u8g2_font_t0_11_tf);
    u8g2.drawStr(0,12, "SGP41 VOC Sensor");
    
    // 1. Sleep: Measure every second (1Hz), as defined by the Gas Index
    // Algorithm
    //    prerequisite
    delay(1000);

    // 2. Measure temperature and humidity for SGP internal compensation
    error = sht4x.measureHighPrecision(temperature, humidity);
    error = sht4x.measureHighPrecisionTicks(temperatureTicks, humidityTicks);
    //temperature = 175 * temperatureTicks/65535-45;
    //humidity = 125 * humidityTicks/65535-6;
    if (error) {
        Serial.print(
            "SHT4x - Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
        Serial.println("Fallback to use default values for humidity and "
                       "temperature compensation for SGP41");
        compensationRh = defaultCompensationRh;
        compensationT = defaultCompensationT;
    } else {
        compensationRh = humidityTicks;        // compensationRh for SGP41
        compensationT = temperatureTicks;      // compensationT for SGP41
        Serial.println("SHT40");
        Serial.print("T: ");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("RH: ");
        Serial.println(humidity);
        Serial.print("Tticks: ");
        Serial.print(temperatureTicks);
        Serial.print("\t");
        Serial.print("RHticks: ");
        Serial.println(humidityTicks);
        Serial.println("");
        float temp = (temperature * 9/5) + 32;    //must keep temperature in ticks for signal conditioning

        u8g2.setCursor(0,28);
        u8g2.print(String(temp) + "°F, RH " + String(humidity) + "%");


        // convert temperature and humidity to ticks as defined by SGP41
        // interface
        // NOTE: in case you read RH and T raw signals check out the
        // ticks specification in the datasheet, as they can be different for
        // different sensors
        //compensationT = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
        //compensationRh = static_cast<uint16_t>(humidity + 6 * 65535 / 125);
    }

    // 3. Measure SGP4x signals
    if (conditioning_s > 0) {
        // During NOx conditioning (10s) SRAW NOx will remain 0
        error = sgp41.executeConditioning(compensationRh, compensationT, srawVoc);
        conditioning_s--;
    } else {
        error = sgp41.measureRawSignals(compensationRh, compensationT, srawVoc, srawNox);
    }

    // 4. Process raw signals by Gas Index Algorithm to get the VOC and NOx
    // index values
    if (error) {
        Serial.print("SGP41 - Error trying to execute measureRawSignals(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.println("SGP41");
        Serial.print("compensationRh: ");
        Serial.print(compensationRh);
        Serial.print("\t");
        Serial.print("compensationT: ");
        Serial.println(compensationT);
        Serial.print("raw VOC Index: ");
        Serial.print(srawVoc);
        Serial.print("\t");
        Serial.print("raw NOx Index: ");
        Serial.println(srawNox);

        int32_t voc_index = voc_algorithm.process(srawVoc);
        int32_t nox_index = nox_algorithm.process(srawNox);
        Serial.print("VOC Index: ");
        Serial.print(voc_index);
        Serial.print("\t");
        Serial.print("NOx Index: ");
        Serial.println(nox_index);
        Serial.println("");

        u8g2.setCursor(0,43);
        u8g2.print("NOx = " + String(nox_index));
        u8g2.print(", VOC = " + String(voc_index));
    }

    // Read ICP-10111 data
  Serial.println("ICP-10111 ");

  Serial.print("Temperature: ");
  Serial.print(icp.getTemperature());
  Serial.println("C");

  float ap = icp.getAirPressure();
  Serial.print(String(ap));
  Serial.print("Pa");
  Serial.print(" \t");
  float Hg = ap/3386;
  Serial.print(String(Hg));
  Serial.println("inHg");

  Serial.print("Altitude: ");
  float alt = icp.getElevation();
  Serial.print(String(alt));
  Serial.println("m");
  Serial.println("------------------------------");

  u8g2.setCursor(0,58);
  u8g2.print(String(Hg) + "inHg");
  u8g2.sendBuffer();					// transfer internal memory to the display

  delay(2000); // Wait for 2 seconds before next reading
}
