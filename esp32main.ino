#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include "DHT.h"
#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#include <SPI.h>
#include <LoRa.h>
// HSPI pin mapping
#define LORA_SCK 14
#define LORA_MISO 12
#define LORA_MOSI 13
#define LORA_SS 5
#define LORA_RST 15
#define LORA_DIO0 26
SPIClass SPI_LORA(HSPI);

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // send every 5 seconds


#define DHTPIN 4      // ESP32 GPIO connected to the DHT11 data pin
#define DHTTYPE DHT11  // Sensor type: DHT11
DHT dht(DHTPIN, DHTTYPE);
String receivedData = "";  // Store the complete string
bool recording = false;    // Track whether data reception has started

Adafruit_ADS1115 ads;
const int windSensorPin = 34; // Select an ADC input pin
const float voltageRef = 3.3; // ESP32 reference voltage
const int adcResolution = 4095; // ESP32 ADC 12-bit resolution
// MQ-135 parameters remain unchanged
const float RL = 1.0;  // MQ module usually has RL=1kΩ
const float a_135 = 110.47;
const float b_135 = -2.862;
const float Ro_clean_air_ratio_135 = 3.6;
float R0_135;

// MQ-2 parameters (smoke)
const float a_2 = 574.25;
const float b_2 = -2.222;
const float Ro_clean_air_ratio_2 = 9.83;
float R0_2;

// MQ-7 parameters (carbon monoxide)
const float a_7 = 99.042;
const float b_7 = -1.518;
const float Ro_clean_air_ratio_7 = 27.5;
float R0_7;

// MQ-9 parameters (flammable gas)
const float a_9 = 1000.5;
const float b_9 = -2.186;
const float Ro_clean_air_ratio_9 = 9.6;
float R0_9;
BluetoothSerial SerialBT;

bool firstDataIgnored = false;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_Master", true); // Run as Bluetooth master
    Serial.println("Searching for ESP32-CAM...");

  // Connect to ESP32-CAM
    if (!SerialBT.connected()) {
        Serial.println("⚠️ ESP32-CAM disconnected. Trying to reconnect...");
        connectToESP32CAM();
    }

    Wire.begin();
 // RX = GPI014
    dht.begin();
    if (!ads.begin(0x48)) {
        Serial.println("ADS1115 not detected!");
        while (1);
    }

    ads.setGain(GAIN_ONE);
    Serial.println("Sensor preheating (recommended 30 minutes)...");
    delay(5000);  // Preheating delay; change to 30 minutes if required

    // MQ-135 calibration
    float V_ADC_135 = ads.readADC_SingleEnded(0) * (4.096 / 32767.0);
    float RS_135 = RL * ((5.0 / V_ADC_135) - 1);
    R0_135 = RS_135 / Ro_clean_air_ratio_135;

    // MQ-2 calibration
    float V_ADC_2 = ads.readADC_SingleEnded(1) * (4.096 / 32767.0);
    float RS_2 = RL * ((5.0 / V_ADC_2) - 1);
    R0_2 = RS_2 / Ro_clean_air_ratio_2;

    // MQ-7 calibration
    float V_ADC_7 = ads.readADC_SingleEnded(2) * (4.096 / 32767.0);
    float RS_7 = RL * ((5.0 / V_ADC_7) - 1);
    R0_7 = RS_7 / Ro_clean_air_ratio_7;

    // MQ-9 calibration
    float V_ADC_9 = ads.readADC_SingleEnded(3) * (4.096 / 32767.0);
    float RS_9 = RL * ((5.0 / V_ADC_9) - 1);
    R0_9 = RS_9 / Ro_clean_air_ratio_9;

    Serial.println("All sensors calibrated!");
      SPI_LORA.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setSPI(SPI_LORA);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    
    LoRa.setSPIFrequency(1E6);
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12);

    if (!LoRa.begin(915E6)) {
      Serial.println("❌ LoRa.begin() failed!");
      while (1);
    }
    Serial.println("✅ LoRa Transceiver Ready!");
  
}

void loop() {
    String fireValue="";
    int windadcValue = analogRead(windSensorPin); 
    float windvoltage = (windadcValue / (float)adcResolution) * voltageRef; // Calculate the actual voltage
    float windSpeed = windvoltage*1000* 0.027; // Calculate wind speed based on the datasheet formula
    Serial.print("wind ADC: ");
    Serial.print(windadcValue);
    Serial.print(" | voltage: ");
    Serial.print(windvoltage, 3);
    Serial.print(" V | wind speed: ");
    Serial.print(windSpeed, 2);
    Serial.println(" m/s");


    float V_ADC_135 = ads.readADC_SingleEnded(0) * (4.096 / 32767.0);
    float RS_135 = RL * ((5.0 / V_ADC_135) - 1);
    float ppm_135 = a_135 * pow((RS_135 / R0_135), b_135) * 100;

    float V_ADC_2 = ads.readADC_SingleEnded(1) * (4.096 / 32767.0);
    float RS_2 = RL * ((5.0 / V_ADC_2) - 1);
    float ppm_2 =  a_135 * pow((RS_2 / R0_2), b_135) * 100; // Smoke ppm

    float V_ADC_7 = ads.readADC_SingleEnded(2) * (4.096 / 32767.0);
    float RS_7 = RL * ((5.0 / V_ADC_7) - 1);
    float ppm_7 = a_7 * pow(RS_7 / R0_7, b_7);

    float V_ADC_9 = ads.readADC_SingleEnded(3) * (4.096 / 32767.0);
    float RS_9 = RL * ((5.0 / V_ADC_9) - 1);
    float ppm_9 = pow(10, (-2.3 * log10(RS_9 / R0_9) + 0.75));

    Serial.print("MQ-135 (CO2): "); Serial.print(ppm_135); Serial.println(" ppm");
    Serial.print("MQ-2 (Smoke): "); Serial.print(ppm_2); Serial.println(" ppm");
    Serial.print("MQ-7 (CO): "); Serial.print(ppm_7); Serial.println(" ppm");
    Serial.print("MQ-9 (Flammable Gas): "); Serial.print(ppm_9); Serial.println(" ppm");

    float temperature = dht.readTemperature();  // Read temperature in Celsius
    float humidity = dht.readHumidity();       // Read humidity

    if (!isnan(temperature) && !isnan(humidity)) {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" *C | Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
    } else {
        Serial.println("Failed to read from DHT11 sensor!");
    } 

   
    while (SerialBT.available()) {  // Keep reading while data is available
        while (SerialBT.available()) {
          char c = SerialBT.read();
          receivedData += c;

          if (receivedData.endsWith("<END>")) {
            break;
          }
        }
      

        // Skip the first packet to avoid incomplete data
      if (!firstDataIgnored) {
        firstDataIgnored = true;
        receivedData="";
            continue;  
        }

      receivedData.trim();

      if (receivedData.startsWith("<START>") && receivedData.endsWith("<END>")) {
        String extractedData = receivedData.substring(7, receivedData.length() - 5);
        

            // Parse Fire and No Fire probabilities

        int fireIndex = extractedData.indexOf("Fire Probability: ");
        int noFireIndex = extractedData.indexOf("No Fire Probability: ");       
        if (fireIndex != -1 && noFireIndex != -1) {

            int fireStart = fireIndex + 18;  // Length of "Fire Probability: " is 18
            int noFireStart = noFireIndex + 22;  // Length of "No Fire Probability: " is 22
            fireValue = extractedData.substring(fireStart, noFireIndex);
            fireValue.trim();  // Get the string first, then trim it

            String noFireValue = extractedData.substring(noFireStart);
            noFireValue.trim();  // Get the string first, then trim it

        // Extract the numeric value part

        Serial.println("Fire: " + fireValue);
        Serial.println("No Fire: " + noFireValue);
        clearBluetoothBuffer();
        receivedData = "";
        break;  // Exit the while loop after parsing one complete packet, then run other tasks
            }

        } else {
            Serial.println("drop worng data: " +   String(receivedData));
        }
    }

    String payload = "{";
    payload += "\"fire_prob\":" + fireValue;
    payload += "\"wind_speed\":" + String(windSpeed, 2) + ",";
    payload += "\"mq135\":" + String(ppm_135, 2) + ",";
    payload += "\"mq2\":" + String(ppm_2, 2) + ",";
    payload += "\"mq7\":" + String(ppm_7, 2) + ",";
    payload += "\"mq9\":" + String(ppm_9, 2);
    payload += "}";
    if (millis() - lastSendTime > sendInterval) {
      String message = "Hello from ESP32! " + String(millis() / 1000) + "s";
      Serial.print("Sending: ");
      Serial.println(payload);

      LoRa.beginPacket();
      LoRa.print(payload);
      LoRa.endPacket();

      lastSendTime = millis();
    }

    delay(5000);
}



void clearBluetoothBuffer() {
    while (SerialBT.available()) {
        SerialBT.read();  // Read and discard data from the buffer
    }
}
void connectToESP32CAM() {
    int maxRetries = 5;
    int retryCount = 0;

    Serial.println("🔍 Searching for ESP32-CAM_BT...");

    while (!SerialBT.connect("ESP32-CAM_BT") && retryCount < maxRetries) {  
        Serial.println("⚠️ Connection failed. Retrying...");
        retryCount++;
        delay(2000);  // Retry after 2 seconds
    }

    if (SerialBT.connected()) {
        Serial.println("✅ Successfully connected to ESP32-CAM!");
    } else {
        Serial.println("❌ Unable to connect to ESP32-CAM. Continuing with other tasks...");
    }
}
