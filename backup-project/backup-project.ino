#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "RTClib.h"
#include <DHT.h>
#include <MPU6050.h>
#include <math.h>

// -------------------- SENSOR OBJECTS --------------------
MAX30105 particleSensor;
RTC_DS3231 rtc;
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu(0x69); // AD0 HIGH

// -------------------- BUZZER --------------------
const int buzzerPin = 3;

// -------------------- TIMERS --------------------
unsigned long fallAlarmStart = 0;
unsigned long timeAlarmStart = 0;
unsigned long bpmAlarmStart  = 0;
const unsigned long alarmDuration = 20000; // 20 seconds
const unsigned long beepInterval = 3000;   // 3 seconds cycle
unsigned long lastBeepTime = 0;

// -------------------- ALARM FLAGS --------------------
bool fallAlarmActive = false;
bool timeAlarmActive = false;
bool bpmAlarmActive  = false;

// -------------------- MPU6050 FALL DETECTION --------------------
float lowerThreshold = 0.5;
float upperThreshold = 2.0;

// -------------------- DS3231 TIME ALARM --------------------
bool timeAlarmTriggered = false;

// -------------------- MAX30102 BPM RANGE --------------------
float bpmLowerLimit = 68;
float bpmUpperLimit = 75;

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  Wire.begin();

  // -------------------- MAX30102 SETUP --------------------
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);

  // -------------------- DHT11 SETUP --------------------
  dht.begin();

  // -------------------- DS3231 SETUP --------------------
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // -------------------- MPU6050 SETUP --------------------
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("All sensors initialized!");
}

// -------------------- HELPER FUNCTION --------------------
void beepBuzzerNonBlocking() {
  unsigned long currentMillis = millis();

  // Reset alarms if duration expired
  if (fallAlarmActive && currentMillis - fallAlarmStart >= alarmDuration) fallAlarmActive = false;
  if (timeAlarmActive && currentMillis - timeAlarmStart >= alarmDuration) timeAlarmActive = false;
  if (bpmAlarmActive  && currentMillis - bpmAlarmStart  >= alarmDuration) bpmAlarmActive  = false;

  // Buzzer beeping every 3 sec for all active alarms
  if (fallAlarmActive || timeAlarmActive || bpmAlarmActive) {
    if (currentMillis - lastBeepTime >= beepInterval) {
      digitalWrite(buzzerPin, HIGH);
      delay(500);  // beep 0.5s
      digitalWrite(buzzerPin, LOW);
      lastBeepTime = currentMillis;

      // Serial messages for active alarms
      if (fallAlarmActive) Serial.println("[ALARM] Fall detected!");
      if (timeAlarmActive) Serial.println("[ALARM] Time 21:00 reached!");
      if (bpmAlarmActive)  Serial.println("[ALARM] Heart rate out of range!");
    }
  }
}

// -------------------- MAIN LOOP --------------------
void loop() {
  unsigned long currentMillis = millis();

  // -------------------- DHT11 READ --------------------
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  if (isnan(temp) || isnan(hum)) {
    Serial.println("Failed to read DHT sensor!");
  } else {
    Serial.print("Temperature: "); Serial.print(temp); Serial.print(" °C, ");
    Serial.print("Humidity: "); Serial.println(hum);
  }

  // -------------------- DS3231 READ --------------------
  DateTime now = rtc.now();
  int hour = now.hour();
  int minute = now.minute();
  Serial.print("Time: ");
  Serial.print(hour); Serial.print(":"); Serial.println(minute);

  // Trigger time alarm at 21:00
  if (hour == 21 && minute == 0 && !timeAlarmTriggered) {
    timeAlarmStart = currentMillis;
    timeAlarmActive = true;
    timeAlarmTriggered = true;
  }
  if (!(hour == 21 && minute == 0)) {
    timeAlarmTriggered = false; // reset for next day
  }

  // -------------------- MAX30102 READ --------------------
  long irValue  = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  float bpm  = (irValue / 1000.0) * 2.4;
  float spo2 = 110 - 25 * ((float)redValue / (float)irValue);

  Serial.print("Heart Rate (BPM): "); Serial.print(bpm);
  Serial.print(" | SpO2 (%): "); Serial.println(spo2);

  // Trigger BPM alarm
  if (bpm < bpmLowerLimit || bpm > bpmUpperLimit) {
    bpmAlarmStart = currentMillis;
    bpmAlarmActive = true;
  }

  // -------------------- MPU6050 FALL DETECTION --------------------
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float Ax = ax / 16384.0;
  float Ay = ay / 16384.0;
  float Az = az / 16384.0;
  float accelMagnitude = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  Serial.print("Accel Magnitude: "); Serial.println(accelMagnitude);

  if (accelMagnitude < lowerThreshold || accelMagnitude > upperThreshold) {
    fallAlarmStart = currentMillis;
    fallAlarmActive = true;
    Serial.println("[ALARM] Fall detected! Alarm triggered for 20 seconds.");
  }

  // -------------------- BUZZER CONTROL --------------------
  beepBuzzerNonBlocking();

  Serial.println("--------------------------");
  delay(2000); // 2 seconds between readings
}
