// ==========================================
// ESP8266 (Wemos D1 R2) Rotary Encoder
// -> Jarak Linear + Arah + Speed + LCD I2C 16x2
// -> Kirim JSON ke Server tiap 5 menit
// -> Multi WiFi roaming (scan RSSI & switch AP)
// -> LCD: W<idx>-<sendCode>
//    - W0  : belum konek wifi / sedang putus
//    - W1..W6: konek ke wifi index sesuai list
//    - sendCode:
//        00..99  = jumlah kirim sukses (2 digit)
//        A       = >=100
//        B       = >=200
//        C       = >=300 ...
//        0z      = >=1000 (freeze, stop naik)
// ==========================================

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================
// KONFIG ENDPOINT SERVER
// ============================
// Pastikan URL TANPA trailing slash setelah .php
const char *serverUrl = "http://103.24.148.59:90/encoder_log.php";
const char *device_id = "esp8266_smk_01";

// ============================
// LIST WIFI (SSID + PASSWORD)
// ============================
struct WifiCred {
  const char *ssid;
  const char *pass;
};

WifiCred wifiList[] = {
  // {"bernot", "bernotdasilva"},
  {"Robot_Resource (Lokal)", "robot@9876"},
  {"PT. Gistex Garmen Indonesia", "GISTEX0102"},
  {"Robotics2", "robot123"},
  {"Robotics3", "robot123"},
  {"Robotics4", "robot123"},
  {"Robotics5", "robot123"},
};

const int WIFI_COUNT = sizeof(wifiList) / sizeof(wifiList[0]);

// ============================
// LCD I2C
// ============================
LiquidCrystal_I2C lcd(0x27, 16, 2);
// SDA = D2 (GPIO4), SCL = D1 (GPIO5)

// ============================
// ENCODER SETUP
// ============================
const int pinEncA = D5; // GPIO14
const int pinEncB = D6; // GPIO12

const float diameter_mm = 50.0;      // roda 5 cm
const int pulsesPerRevolution = 360; // sesuai spek encoder

volatile long encoderCount = 0;

float distancePerPulse_mm = 0;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500; // ms
long lastCountForSpeed = 0;

float totalDistance_m = 0.0;

long currentCountCached = 0;
long deltaCountCached = 0;
float speed_km_per_j_Cached = 0.0;
const char *dirShortCached = "STP";

// ============================
// TIMER KIRIM DATA
// ============================
unsigned long lastSendTime = 0;
const unsigned long sendIntervalMs = 5000UL; // 5 menit

unsigned long lastFailRetryMs = 0;
const unsigned long failRetryIntervalMs = 15000UL; // retry tiap 15 detik kalau gagal

// ===== counter kirim sukses =====
unsigned int sendOkCount = 0;
bool sendCountFrozen = false;

// ============================
// WIFI ROAMING PARAMETER
// ============================
// Scan hanya saat perlu
unsigned long lastScanMs = 0;
const unsigned long minScanIntervalMs = 20000UL; // minimal jeda scan 20 detik

// RSSI threshold
const int rssiBad = -75;         // jika di bawah ini dianggap jelek
const int rssiGood = -67;        // jika di atas ini dianggap aman
const int rssiSwitchMargin = 10; // AP baru harus lebih kuat minimal 10 dB (agar tidak ping-pong)

// Butuh sinyal jelek beberapa detik sebelum scan
unsigned long badRssiSinceMs = 0;
const unsigned long badRssiHoldMs = 5000UL; // 5 detik

// Cooldown setelah switch
unsigned long lastSwitchMs = 0;
const unsigned long switchCooldownMs = 25000UL; // 25 detik

// Rekam status wifi untuk timer tampil (dipakai untuk logika transisi)
bool wifiWasConnected = false;
unsigned long wifiUpSinceMs = 0;
unsigned long wifiDownSinceMs = 0;

// Simpan SSID target yang sedang dipakai
String currentSsid = "";

// ============================
// UTIL: cari password dari SSID
// ============================
bool getCredBySsid(const String &ssid, const char* &outPass) {
  for (int i = 0; i < WIFI_COUNT; i++) {
    if (ssid.equals(wifiList[i].ssid)) {
      outPass = wifiList[i].pass;
      return true;
    }
  }
  return false;
}

// Return 1..WIFI_COUNT jika SSID ada di list, kalau tidak ketemu return 0
int wifiIndexFromSsid(const String &ssid)
{
  for (int i = 0; i < WIFI_COUNT; i++) {
    if (ssid.equals(wifiList[i].ssid)) return i + 1;
  }
  return 0;
}

// Kode kirim:
// 00..99  = 2 digit
// A       = >=100
// B       = >=200
// ...
// 0z      = >=1000 (freeze)
String sendCode()
{
  if (sendCountFrozen || sendOkCount >= 1000) return "0z";

  if (sendOkCount < 100) {
    char buf[3];
    snprintf(buf, sizeof(buf), "%02u", sendOkCount);
    return String(buf);
  }

  unsigned int bucket = (sendOkCount / 100); // 1 untuk 100-199
  char letter = char('A' + (bucket - 1));
  return String(letter);
}

// LCD helper: pendekkan SSID (biar muat)
String shortSsid(const String &s)
{
  if (s.length() <= 6) return s;
  return s.substring(0, 6);
}

// ============================
// INTERRUPT HANDLER ENCODER A
// ============================
void IRAM_ATTR handleEncA()
{
  bool B = digitalRead(pinEncB);
  if (B == LOW) encoderCount++;
  else encoderCount--;
}

// ============================
// WIFI: connect ke SSID tertentu (non-blocking style)
// ============================
void connectToSsid(const String &ssid)
{
  const char *pass = nullptr;
  if (!getCredBySsid(ssid, pass)) {
    Serial.print("SSID tidak ada di list: ");
    Serial.println(ssid);
    return;
  }

  Serial.println();
  Serial.println("=== CONNECT WIFI ===");
  Serial.print("Target SSID: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);

  // Putus halus saja
  WiFi.disconnect(false);
  delay(50);

  WiFi.begin(ssid.c_str(), pass);
  currentSsid = ssid;

  lastSwitchMs = millis();
}

// ============================
// WIFI: konek awal TANPA scan (biar encoder tidak telat)
// ============================
void connectWiFiInitialFast()
{
  Serial.println();
  Serial.println("=== WIFI INITIAL CONNECT (FAST) ===");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  delay(50);

  // coba langsung SSID pertama, nanti roaming akan cari terbaik
  WiFi.begin(wifiList[0].ssid, wifiList[0].pass);
  currentSsid = String(wifiList[0].ssid);
  lastSwitchMs = millis();
}

// ============================
// ROAM: scan -> pilih AP terbaik dari list -> switch bila perlu
// ============================
void roamToBetterAP(int currentRssi)
{
  Serial.println();
  Serial.println("=== ROAM SCAN ===");

  int n = WiFi.scanNetworks(false, true);
  Serial.print("Scan count: ");
  Serial.println(n);

  int bestIdx = -1;
  int bestRssi = -999;

  for (int i = 0; i < n; i++) {
    String ss = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);

    for (int k = 0; k < WIFI_COUNT; k++) {
      if (ss.equals(wifiList[k].ssid)) {
        if (rssi > bestRssi) {
          bestRssi = rssi;
          bestIdx = k;
        }
      }
    }
  }

  if (bestIdx < 0) {
    Serial.println("Tidak ada SSID list yang terdeteksi.");
    return;
  }

  String bestSsid = String(wifiList[bestIdx].ssid);

  Serial.print("Best SSID: ");
  Serial.print(bestSsid);
  Serial.print(" RSSI: ");
  Serial.println(bestRssi);

  bool wifiOK = (WiFi.status() == WL_CONNECTED);
  String nowSsid = wifiOK ? WiFi.SSID() : currentSsid;

  // Jika belum connect, langsung pindah ke best
  if (!wifiOK) {
    connectToSsid(bestSsid);
    return;
  }

  // Kalau sudah connect:
  if (currentRssi >= rssiGood) {
    Serial.println("RSSI sudah bagus, tidak perlu switch.");
    return;
  }

  // Switch jika SSID beda dan jauh lebih kuat
  if (!bestSsid.equals(nowSsid) && (bestRssi >= (currentRssi + rssiSwitchMargin))) {
    Serial.print("Switching AP: ");
    Serial.print(nowSsid);
    Serial.print(" (");
    Serial.print(currentRssi);
    Serial.print(") -> ");
    Serial.print(bestSsid);
    Serial.print(" (");
    Serial.print(bestRssi);
    Serial.println(")");

    connectToSsid(bestSsid);
    return;
  }

  Serial.println("Tidak ada kandidat yang cukup lebih kuat untuk switch.");
}

// ============================
// WIFI SERVICE: monitor + roaming
// ============================
void wifiService()
{
  unsigned long now = millis();
  bool wifiOK = (WiFi.status() == WL_CONNECTED);

  // transisi putus
  if (!wifiOK && wifiWasConnected)
  {
    wifiDownSinceMs = now;
    wifiWasConnected = false;
    badRssiSinceMs = 0;
  }

  // transisi nyambung
  if (wifiOK && !wifiWasConnected)
  {
    wifiUpSinceMs = now;
    wifiWasConnected = true;
    currentSsid = WiFi.SSID();
  }

  // Jika connect, cek RSSI
  if (wifiOK)
  {
    int rssiNow = WiFi.RSSI();

    if (rssiNow < rssiBad) {
      if (badRssiSinceMs == 0) badRssiSinceMs = now;
    } else {
      badRssiSinceMs = 0;
    }

    bool shouldScan = false;
    if (badRssiSinceMs != 0 && (now - badRssiSinceMs >= badRssiHoldMs)) shouldScan = true;
    if (shouldScan && (now - lastScanMs < minScanIntervalMs)) shouldScan = false;
    if (shouldScan && (now - lastSwitchMs < switchCooldownMs)) shouldScan = false;

    if (shouldScan) {
      lastScanMs = now;
      roamToBetterAP(rssiNow);
      badRssiSinceMs = 0;
    }
    return;
  }

  // Jika tidak connect: scan best dengan interval
  if (now - lastScanMs >= minScanIntervalMs) {
    lastScanMs = now;
    roamToBetterAP(-999);
  } else {
    // coba ulang SSID terakhir setiap 5 detik
    if (now - lastSwitchMs >= 5000UL) {
      if (currentSsid.length() > 0) connectToSsid(currentSsid);
      else connectToSsid(String(wifiList[0].ssid));
    }
  }
}

// ============================
// UPDATE LCD: Baris 1 = D + DIR
//             Baris 2 = W<idx>-<sendCode> RSSI
// ============================
void updateLCD()
{
  unsigned long now = millis();
  bool wifiOK = (WiFi.status() == WL_CONNECTED);

  long dist_m_int = (long)(totalDistance_m + 0.5f);

  lcd.clear();

  // ===== BARIS 1 =====
  // contoh: D:123 FWD
  lcd.setCursor(0, 0);
  lcd.print("D:");
  lcd.print(dist_m_int);
  lcd.print(" ");
  lcd.print(dirShortCached);

  // ===== BARIS 2 =====
  // contoh: W3-02 -65
  lcd.setCursor(0, 1);

  if (!wifiOK) {
    lcd.print("W0-");
    lcd.print(sendCode());
    lcd.print(" --");
  } else {
    int widx = wifiIndexFromSsid(WiFi.SSID()); // 1..6, kalau tidak ketemu 0
    lcd.print("W");
    lcd.print(widx);
    lcd.print("-");
    lcd.print(sendCode());
    lcd.print(" ");
    lcd.print(WiFi.RSSI());
  }
}

// ============================
// UPDATE PERHITUNGAN ENCODER + CACHE
// ============================
void updateEncoderStats()
{
  unsigned long now = millis();
  if (now - lastPrintTime < printInterval) return;
  lastPrintTime = now;

  long currentCount;
  noInterrupts();
  currentCount = encoderCount;
  interrupts();

  long deltaCount = currentCount - lastCountForSpeed;
  lastCountForSpeed = currentCount;

  if (deltaCount > 0)
  {
    float distanceInterval_mm_forward = deltaCount * distancePerPulse_mm;
    totalDistance_m += distanceInterval_mm_forward / 1000.0;
  }

  float deltaTime_s = printInterval / 1000.0;
  float distanceInterval_mm = deltaCount * distancePerPulse_mm;
  float speed_mm_per_s = distanceInterval_mm / deltaTime_s;
  float speed_km_per_j = (speed_mm_per_s / 1000.0) * 3.6;

  const char *dirShort;
  if (deltaCount > 0) dirShort = "FWD";
  else if (deltaCount < 0) dirShort = "REV";
  else dirShort = "STP";

  currentCountCached = currentCount;
  deltaCountCached = deltaCount;
  speed_km_per_j_Cached = speed_km_per_j;
  dirShortCached = dirShort;

  // Serial debug ringkas
  Serial.print("CNT:");
  Serial.print(currentCountCached);
  Serial.print(" D:");
  Serial.print(totalDistance_m, 3);
  Serial.print("m V:");
  Serial.print(speed_km_per_j_Cached, 2);
  Serial.print(" ");
  Serial.print(dirShortCached);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" RSSI:");
    Serial.print(WiFi.RSSI());
    Serial.print(" SSID:");
    Serial.print(WiFi.SSID());
  } else {
    Serial.print(" WIFI:DIS");
  }
  Serial.print(" OKsend:");
  Serial.println(sendOkCount);

  updateLCD();
}

// ============================
// KIRIM DATA KE SERVER (return true kalau sukses)
// ============================
bool sendDistanceData()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Send skip, WiFi belum connect.");
    return false;
  }

  float distanceToSend = totalDistance_m;

  String idToSend;

  if (WiFi.status() == WL_CONNECTED) {
    idToSend = WiFi.SSID();          // SSID yang benar-benar terhubung
  } else if (currentSsid.length() > 0) {
    idToSend = currentSsid;          // SSID target terakhir
  } else {
    idToSend = "NO_WIFI";
  }

  // JSON escape sederhana: ganti " menjadi '
  idToSend.replace("\"", "'");

  String payload = "{";
  payload += "\"device_id\":\"" + idToSend + "\",";
  payload += "\"total_distance_m\":" + String(distanceToSend, 3);
  payload += "}";


  Serial.println("POST -> server");
  Serial.println(payload);

  WiFiClient client;
  client.setTimeout(8000);

  HTTPClient http;
  http.setTimeout(8000);
  http.setReuse(false);

  int httpCode = -1;

  if (http.begin(client, serverUrl))
  {
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Connection", "close");
    httpCode = http.POST(payload);

    if (httpCode > 0) {
      Serial.print("HTTP ");
      Serial.println(httpCode);
      Serial.println(http.getString());
    } else {
      Serial.print("HTTP POST gagal: ");
      Serial.println(http.errorToString(httpCode));
    }

    http.end();
  }
  else
  {
    Serial.println("http.begin gagal (URL salah / DNS / port).");
  }

  if (httpCode >= 200 && httpCode <= 299)
  {
    // naikkan counter sukses kirim sampai 1000 lalu freeze
    if (!sendCountFrozen) {
      sendOkCount++;
      if (sendOkCount >= 1000) {
        sendOkCount = 1000;
        sendCountFrozen = true;
      }
    }
    return true;
  }

  return false;
}

// ============================
// SETUP
// ============================
void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);

  distancePerPulse_mm = (3.14159265 * diameter_mm) / pulsesPerRevolution;
  attachInterrupt(digitalPinToInterrupt(pinEncA), handleEncA, RISING);

  Wire.begin(D2, D1);
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Encoder Ready");
  lcd.setCursor(0, 1);
  lcd.print("Start...");
  delay(400);

  // Start WiFi cepat, tanpa scan blocking
  connectWiFiInitialFast();

  lastSendTime = millis();
  lastPrintTime = millis();
  lcd.clear();
}

// ============================
// LOOP
// ============================
void loop()
{
  updateEncoderStats();
  wifiService();

  unsigned long now = millis();

  // Normal send interval
  if (now - lastSendTime >= sendIntervalMs)
  {
    bool ok = sendDistanceData();

    // IMPORTANT: update lastSendTime selalu, biar tidak spam saat gagal
    lastSendTime = now;

    if (!ok) lastFailRetryMs = now;
    else lastFailRetryMs = 0;
  }
  // Retry kalau sebelumnya gagal
  else if (lastFailRetryMs != 0 && (now - lastFailRetryMs >= failRetryIntervalMs))
  {
    bool ok = sendDistanceData();
    lastFailRetryMs = now;
    if (ok) lastFailRetryMs = 0;
  }

  delay(10);
}
