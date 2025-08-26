#include <Wire.h>              // I2C communication
#include "MAX30105.h"           // MAX30102 heart rate + SpO2 sensor
#include "heartRate.h"          // Heart rate algorithm
#include "RTClib.h"             // Real-time clock (DS3231)
#include <DHT.h>                // Temperature & humidity (DHT11)
#include <MPU6050.h>            // Accelerometer/gyroscope
#include <math.h>               // For sqrt() etc.
#include <LiquidCrystal_I2C.h>  // LCD display via I2C


// -------------------- SENSOR OBJECTS --------------------
MAX30105 particleSensor;      // This creates an object called particleSensor from the MAX30105 class (library).
RTC_DS3231 rtc;               // This creates an RTC object named rtc (RTC DS3231). 
#define DHTPIN 4              // means the sensor’s data pin is connected to Arduino pin D4.
#define DHTTYPE DHT11         // tells the library you’re using a DHT11 (not DHT22 or DHT21).
DHT dht(DHTPIN, DHTTYPE);     // creates a DHT sensor object named dht
MPU6050 mpu(0x69);            // AD0 HIGH. Default address is 0x68 when the AD0 pin is LOW (GND).

// -------------------- BUZZER --------------------
const int buzzerPin = 3;      // Buzzer at D3 for alarms.

// -------------------- LCD --------------------
LiquidCrystal_I2C lcd(0x27, 16, 2); // What it is: Creates an LCD object for an HD44780 16×2 character LCD using an I²C backpack.

// -------------------- TIMERS --------------------

// long = a 32-bit integer (can store positive or negative numbers).
// unsigned = means no negative values allowed.
unsigned long fallAlarmStart = 0;  // When fall alarm started
unsigned long timeAlarmStart = 0;  // When medicine time alarm started
unsigned long bpmAlarmStart  = 0;  // When BPM alarm started
const unsigned long alarmDuration = 20000; // Duration for each alarm = 20 seconds
const unsigned long beepInterval = 3000;   // Buzzer beeps every 3 seconds
unsigned long lastBeepTime = 0;            // Last time buzzer beeped

// LCD display switching
unsigned long lastLCDUpdate = 0;  // Stores the last time the LCD was updated.
int lcdState = 0; 
// Keeps track of what’s currently shown on the LCD:
// 0 = Temperature screen, 1 = BPM/SpO₂ screen
// Every 2 seconds in the loop, this toggles between them.

// -------------------- ALARM FLAGS --------------------
bool fallAlarmActive = false;               // becomes true when a fall is detected
bool timeAlarmActive = false;               // becomes true when the clock reaches 21:00.
bool bpmAlarmActive  = false;               // becomes true when heart rate goes outside safe range.
bool timeAlarmTriggered = false;            // prevents the medicine alarm from being triggered over and over within the same minute. (Resets when the clock moves on.)

// -------------------- MPU6050 FALL DETECTION THRESHOLDS --------------------
float lowerThreshold = 0.5; // Acceleration lower bound
float upperThreshold = 2.0; // Acceleration upper bound

// -------------------- MAX30102 BPM RANGE --------------------
float bpmLowerLimit = 50;  // Safe heart rate lower bound
float bpmUpperLimit = 75;  // Safe heart rate upper bound

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);    // Tells Arduino that the buzzer pin will send signals (not read).
  Wire.begin();                  // Starts the I²C communication bus

  // -------------------- MAX30102 SETUP --------------------
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    // This tries to initialize the MAX30102 sensor using the I²C communication bus (Wire).
    // I2C_SPEED_STANDARD → means it communicates at 100 kHz (standard I²C speed).
    Serial.println("MAX30102 not found!");
    while (1);  // Stop program
  }
  particleSensor.setup();                       // Loads the default configuration of the MAX30102.
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);
  // These functions control the LED brightness inside the MAX30102.
  // 0x0A = 10 in decimal → a very low brightness level.

  // -------------------- DHT11 SETUP --------------------
  dht.begin();  // Starts the DHT11 sensor so you can read temperature and humidity later.

  // -------------------- DS3231 SETUP --------------------
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {  // checks if the RTC’s backup battery ever died or got removed.
    Serial.println("RTC lost power, setting the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // This sets the RTC’s time to the date & time when you compiled the sketch.
  }

  // -------------------- MPU6050 SETUP --------------------
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // -------------------- LCD INIT --------------------
  lcd.begin();          // Initialize LCD
  lcd.backlight();
  //   This is the small LED behind the LCD that makes the text visible.
  // If you don’t call this (or if the backlight jumper on the I²C module is open), the text might be written but you won’t see anything.

  Serial.println("All sensors initialized!");
}

// -------------------- HELPER FUNCTION --------------------
void beepBuzzerNonBlocking() {
  unsigned long currentMillis = millis();  // returns the number of milliseconds since Arduino powered on.

  // Reset alarms if duration expired
  if (fallAlarmActive && currentMillis - fallAlarmStart >= alarmDuration) fallAlarmActive = false;
  if (timeAlarmActive && currentMillis - timeAlarmStart >= alarmDuration) timeAlarmActive = false;
  if (bpmAlarmActive  && currentMillis - bpmAlarmStart  >= alarmDuration) bpmAlarmActive  = false;

  // Buzzer beeping every 3 sec for all active alarms
  if (fallAlarmActive || timeAlarmActive || bpmAlarmActive) {
    if (currentMillis - lastBeepTime >= beepInterval) {
      digitalWrite(buzzerPin, HIGH);  // Turn buzzer ON
      delay(500);                     // Keep buzzer ON for 0.5s
      digitalWrite(buzzerPin, LOW);   // Turn buzzer OFF
      lastBeepTime = currentMillis;   // Update last beep time

      // Serial messages for active alarms
      if (fallAlarmActive) Serial.println("[ALARM] Fall detected!");
      if (timeAlarmActive) Serial.println("[ALARM] Time 21:00 reached!");
      if (bpmAlarmActive)  Serial.println("[ALARM] Heart rate out of range!");
    }
  }
}

// -------------------- MAIN LOOP --------------------
void loop() {
  unsigned long currentMillis = millis();     // Get current time

  // -------------------- DHT11 READ --------------------
  float temp = dht.readTemperature();                   // Read temperature (Celsius)
  float hum  = dht.readHumidity();                      // Read humidity (%)
  if (isnan(temp) || isnan(hum)) {                      // Check for failed read
    Serial.println("Failed to read DHT sensor!");
  } else {
    Serial.print("Temperature: "); Serial.print(temp); Serial.print(" °C, ");
    Serial.print("Humidity: "); Serial.println(hum);
  }

  // -------------------- DS3231 READ (Real Time Clock) --------------------
  DateTime now = rtc.now();        // Read current time
  int hour = now.hour();           // Extract hour
  int minute = now.minute();       // Extract minute
  Serial.print("Time: ");
  Serial.print(hour); Serial.print(":"); Serial.println(minute);

  // Trigger time alarm at 21:00
  if (hour == 21 && minute == 0 && !timeAlarmTriggered) {
     Serial.print("Time:ddd ");
    timeAlarmStart = currentMillis;
    timeAlarmActive = true;
    timeAlarmTriggered = true;   // Prevent multiple triggers
  }
  if (!(hour == 21 && minute == 0)) {
    timeAlarmTriggered = false; // reset for next day
  }

  // -------------------- MAX30102 READ --------------------
  long irValue  = particleSensor.getIR();      // Read IR signal
  long redValue = particleSensor.getRed();     // Read Red signal

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
  int16_t ax, ay, az;                                  // int16_t = 16-bit signed integer (range: -32768 to +32767).
  mpu.getAcceleration(&ax, &ay, &az);                  // Read acceleration
  float Ax = ax / 16384.0;                             // Convert raw → g units
  float Ay = ay / 16384.0;                             // dividing by 16384.0 converts raw numbers into g units.
  float Az = az / 16384.0;
  float accelMagnitude = sqrt(Ax*Ax + Ay*Ay + Az*Az);  // Calculate total acceleration
  Serial.print("Accel Magnitude: "); Serial.println(accelMagnitude);

  if (accelMagnitude < lowerThreshold || accelMagnitude > upperThreshold) {
    fallAlarmStart = currentMillis;
    fallAlarmActive = true;
    Serial.println("[ALARM] Fall detected! Alarm triggered for 20 seconds.");
  }

  // -------------------- BUZZER CONTROL --------------------
  beepBuzzerNonBlocking();  // Handle buzzer alarm

  // -------------------- LCD CONTROL --------------------
  if (timeAlarmActive) {
    // During medicine alarm → show "Med 1" for 20 sec
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Med 1");
  } else {
    // Cycle between Temp and BPM every 2 sec
    if (currentMillis - lastLCDUpdate >= 2000) {
      lcd.clear();
      if (lcdState == 0) {
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(temp);
        lcd.print(" C");
        lcdState = 1;
      } else {
        lcd.setCursor(0, 0);
        lcd.print("BPM: ");
        lcd.print(bpm, 1);
        lcd.setCursor(0, 1);
        lcd.print("SpO2: ");
        lcd.print(spo2, 1);
        lcd.print("%");
        lcdState = 0;
      }
      lastLCDUpdate = currentMillis;   // Update timer
    }
  }

  Serial.println("--------------------------");
  delay(500); // Small delay to reduce noise
}
