

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3DH.h>

#include "WiFi.h" 
#define LED_Status 2 // LED_Status Onboard GPIO=2 

Adafruit_BMP280 bme; // I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH(); //I2C

void setup() {
  Serial.begin(115200);
  Serial.println(F("BMP280 test"));
  bme.begin(0x76);
  Serial.println("LIS3DH test!");
  lis.begin(0x19);
  
  pinMode(LED_Status, OUTPUT);
  
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
  case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
  case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
  case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
  case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
  case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
  case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
  case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

  case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
  case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
  case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }
}

void loop() {
  Serial.println(":BMP280-Sensor Data:");
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" ℃");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure());
  Serial.println(" Pa");

  Serial.print("Approx altitude = ");
  Serial.print(bme.readAltitude(1007.45)); // this should be adjusted to your local forcase
  Serial.println(" m");
  Serial.println();
  delay(2000);

  lis.read();  // get X Y and Z data at once 
  sensors_event_t event;
  lis.getEvent(&event);
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.println(":LIS3DH-Sensor Data:"); 
  Serial.print("LIS3DH Event: ");
  Serial.print(" \tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
  Serial.println(" m/s^2 ");
  Serial.println();
  delay(2000);

  Serial.println("Scan WiFi");
    digitalWrite(LED_Status, !digitalRead(LED_Status));
    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("Scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
            delay(10);
        }
    }
    Serial.println("");
    delay(1000);
}
