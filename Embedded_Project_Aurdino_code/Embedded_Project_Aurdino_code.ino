#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>
#include <time.h>

// Wi-Fi Credentials
#define WIFI_SSID "YOUR_WIFI"
#define WIFI_PASSWORD "PASSWORD"

// Firebase Credentials
#define API_KEY " "
#define DATABASE_URL " "

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// LCD I2C Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use Serial1
#define GPS_RX 4
#define GPS_TX 2

// Sensor Pins
#define FIRE_SENSOR_PIN 35
#define WATER_SENSOR_PIN 34
#define IR_SENSOR_PIN 33   // RIGHT IR sensor
#define IR2_SENSOR_PIN 32  // LEFT IR sensor
#define TRIG_PIN 17
#define ECHO_PIN 16

// Helper variables
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // 5 seconds

// Connect to WiFi
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  Serial.print("Connecting to WiFi");

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(retry % 16, 1);
    lcd.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(2000);
  } else {
    Serial.println("\nWiFi Failed");
    lcd.clear();
    lcd.print("WiFi Failed");
    delay(2000);
  }
}

// NTP Time Setup
void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing time...");
  while (time(nullptr) < 100000) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Time Synced");
}

// Ultrasonic Distance
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

// Setup
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("RTMS Booting...");
  delay(2000);

  pinMode(FIRE_SENSOR_PIN, INPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(IR2_SENSOR_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  connectWiFi();
  setupTime();

  // Firebase config
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = API_KEY;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// Loop
void loop() {
  int fireRaw = analogRead(FIRE_SENSOR_PIN);
  int waterRaw = analogRead(WATER_SENSOR_PIN);

  bool irRightReading = digitalRead(IR_SENSOR_PIN);  // RIGHT sensor
  bool irLeftReading = digitalRead(IR2_SENSOR_PIN);  // LEFT sensor

  bool crackRightDetected = irRightReading;
  bool crackLeftDetected = irLeftReading;

  float distance = getDistance();
  bool fireDetected = fireRaw < 400;
  bool waterDetected = waterRaw > 500;
  bool crackDetected = crackLeftDetected || crackRightDetected;
  bool objectDetected = (distance > 0 && distance < 20);

  // GPS Handling
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lng = gps.location.isValid() ? gps.location.lng() : 0.0;

  // LCD Display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("F:"); lcd.print(fireDetected ? "Y" : "N");
  lcd.setCursor(4, 0);
  lcd.print("W:"); lcd.print(waterDetected ? "Y" : "N");
  lcd.setCursor(8, 0);
  lcd.print("C:"); lcd.print(crackDetected ? "Y" : "N");
  lcd.setCursor(12, 0);
  lcd.print("O:"); lcd.print(objectDetected ? "Y" : "N");
  lcd.setCursor(0, 1);
  lcd.print("GPS:"); lcd.print(gps.location.isValid() ? "OK" : "No");

  if (millis() - lastSendTime > sendInterval) {
    lastSendTime = millis();

    // Update /alerts
    Firebase.RTDB.setBool(&fbdo, "/alerts/fire", fireDetected);
    Firebase.RTDB.setBool(&fbdo, "/alerts/water", waterDetected);
    Firebase.RTDB.setBool(&fbdo, "/alerts/obstacle", objectDetected);
    Firebase.RTDB.setBool(&fbdo, "/alerts/crack", crackDetected);
    Firebase.RTDB.setDouble(&fbdo, "/alerts/gps/lat", lat);
    Firebase.RTDB.setDouble(&fbdo, "/alerts/gps/lng", lng);

    // Update /anomalies
    if (fireDetected || waterDetected || crackDetected || objectDetected) {
      String type = fireDetected ? "fire" :
                    waterDetected ? "water" :
                    crackDetected ? "crack" :
                    objectDetected ? "obstacle" : "unknown";

      String severity = fireDetected ? "High" :
                        waterDetected ? "Medium" :
                        crackDetected ? "Medium" :
                        objectDetected ? "Low" : "Unknown";

      FirebaseJson anomaly;
      anomaly.set("lat", lat);
      anomaly.set("lng", lng);
      anomaly.set("resolved", false);
      anomaly.set("severity", severity);
      anomaly.set("type", type);
      anomaly.set("timestamp_ms", millis());

      // ISO timestamp
      char isoTime[30];
      time_t now = time(nullptr);
      strftime(isoTime, sizeof(isoTime), "%FT%TZ", gmtime(&now));
      anomaly.set("timestamp", isoTime);

      String path = "/anomalies/" + String(millis());
      Firebase.RTDB.setJSON(&fbdo, path.c_str(), &anomaly);
    }

    // Serial Logging
    Serial.println("Firebase Updated:");
    Serial.print("Fire: "); Serial.println(fireDetected);
    Serial.print("Water: "); Serial.println(waterDetected);
    Serial.print("Crack: "); Serial.print(crackDetected ? "YES" : "NO");
    if (crackDetected) {
      Serial.print(" (Left: "); Serial.print(crackLeftDetected ? "Y" : "N");
      Serial.print(", Right: "); Serial.print(crackRightDetected ? "Y" : "N");
      Serial.print(")");
    }
    Serial.println();
    Serial.print("Object: "); Serial.println(objectDetected);
    Serial.print("GPS: "); Serial.print(lat, 6); Serial.print(", ");
    Serial.println(lng, 6);
    Serial.println();
  }

  delay(3000);
}