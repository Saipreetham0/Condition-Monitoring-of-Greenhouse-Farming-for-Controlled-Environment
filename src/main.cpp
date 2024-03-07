#include <Arduino.h>

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL3rjsoHVgj"
#define BLYNK_TEMPLATE_NAME "Green House"
#define BLYNK_AUTH_TOKEN "bn9g-Msz4YQ14sytdebjlwWLOtpL-kUW"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define fanRelayPin 14
#define heaterRelayPin 26
#define waterMotorRelayPin 27

#define DHTPIN 4
#define DHTTYPE DHT11

char ssid[] = "KSP";
char pass[] = "9550421866";

DHT dht(DHTPIN, DHTTYPE);

const int mq4Pin = 33;          // Analog pin for MQ4 gas sensor
const int soilMoisturePin = 32; // Analog pin for soil moisture sensor
const int ldrPin = 5;           // Analog pin for LDR sensor

BlynkTimer timer;

float temperatureThreshold = 25.0; // Temperature threshold in Celsius
float humidityThreshold = 70.0;    // Humidity threshold in percentage
// int lightThreshold = 50;          // LDR threshold value
int moistureThreshold = 50; // Soil moisture threshold value
// int gasThreshold = 500;            // MQ4 gas sensor threshold value

float temperature = 0.0;   // Temperature reading
float humidity = 0.0;      // Humidity reading
int gasValue = 0;          // MQ4 gas sensor reading
int soilMoistureValue = 0; // Soil moisture sensor reading
int ldrValue = 0;          // LDR sensor reading
float soilMoisturePercentage = 0.0;

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void sendSensorData()
{
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  gasValue = analogRead(mq4Pin);
  soilMoistureValue = analogRead(soilMoisturePin);
  ldrValue = digitalRead(ldrPin);

  if (isnan(temperature) || isnan(humidity))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  soilMoisturePercentage = map(soilMoistureValue, 0, 4095, 100, 0);
  // map(value, 0, 4095, 100, 0);

  // Print sensor data to Serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Gas Value: ");
  Serial.println(gasValue);

  Serial.print("Soil Moisture Percentage: ");
  Serial.print(soilMoisturePercentage);
  Serial.println(" %");

  Serial.print("LDR Value: ");
  Serial.println(ldrValue);

  Blynk.virtualWrite(V0, temperature);            // Virtual pin V5 for temperature
  Blynk.virtualWrite(V1, humidity);               // Virtual pin V6 for humidity
  Blynk.virtualWrite(V2, gasValue);               // Virtual pin V7 for gas sensor
  Blynk.virtualWrite(V3, soilMoisturePercentage); // Virtual pin V8 for soil moisture sensor
  Blynk.virtualWrite(V4, ldrValue);               // Virtual pin V9 for LDR sensor

  lcd.clear();
  // Display sensor values on LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %");

  delay(2000); // Wait 2 seconds before clearing the LCD

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Gas: ");
  lcd.print(gasValue);

  lcd.setCursor(0, 1);
  lcd.print("Moisture: ");
  lcd.print(soilMoisturePercentage);
  lcd.print(" %");

  delay(2000); // Wait 2 seconds before clearing the LCD
  lcd.clear();

  // lcd.setCursor(0, 0);
  // lcd.print("LDR: ");
  // lcd.print(ldrValue);

  // delay(2000); // Wait 2 seconds before clearing the LCD
  // lcd.clear();
}

void connectWiFi()
{
  // Connect to Wi-Fi
  // WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    lcd.setCursor(0, 0);
    lcd.print("WiFi: Connecting...");
  }
  lcd.setCursor(0, 0);
  lcd.print("WiFi: Connected   ");
}
void setup()
{
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  lcd.init();      // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
  lcd.setCursor(0, 0);
  lcd.print("WiFi: Connecting...");

  connectWiFi(); // Connect to WiFi network

  dht.begin();

  pinMode(mq4Pin, INPUT);
  pinMode(soilMoisturePin, INPUT);
  pinMode(ldrPin, INPUT);

  pinMode(fanRelayPin, OUTPUT);
  pinMode(heaterRelayPin, OUTPUT);
  pinMode(waterMotorRelayPin, OUTPUT);

  // Initially turn off all relays
  digitalWrite(fanRelayPin, LOW);
  digitalWrite(heaterRelayPin, LOW);
  digitalWrite(waterMotorRelayPin, LOW);

  timer.setInterval(1000L, sendSensorData); // Sending sensor data every 10 seconds
}

void loop()
{
  Blynk.run();
  timer.run();

  if (temperature > temperatureThreshold)
  {
    digitalWrite(heaterRelayPin, HIGH); // Turn on heater
  }
  else
  {
    digitalWrite(heaterRelayPin, LOW); // Turn off heater
  }

  if (humidity > humidityThreshold)
  {
    digitalWrite(fanRelayPin, HIGH); // Turn on fan
  }
  else
  {
    digitalWrite(fanRelayPin, LOW); // Turn off fan
  }

  if (soilMoistureValue < moistureThreshold)
  {
    digitalWrite(waterMotorRelayPin, HIGH); // Turn on water motor
  }
  else
  {
    digitalWrite(waterMotorRelayPin, LOW); // Turn off water motor
  }
}
