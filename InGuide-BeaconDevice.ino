#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFi.h>

// Define GPS RX/TX pins and serial
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);  // Serial2 on ESP32

const char* SensorID = "1";
const unsigned long readInterval = 10000;

//--------- WiFi------------------------
// const char* ssid = "BAMY_2.4G";
// const char* password = "12345678";
// const char* host = "192.168.1.141";
const char* ssid = "Bay";
const char* password = "password";
const char* host = "172.20.10.2";
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  Serial.println("Start!!!");
  Serial.println("Starting GPS reader...");

  // connect wifi
  wifiConnect();
  server.begin();

  // Start GPS Serial on UART2
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  Serial.println("GPS Serial started");
}

unsigned long lastPrint = 0;

void loop() {
  // Feed TinyGPSPlus continuously
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    gps.encode(c);   // parse data
  }

  // Print every second
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();

    if (gps.location.isValid()) {
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 8);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 8);
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
      Serial.print("Altitude: ");
      Serial.println(gps.altitude.meters());
    } else {
      Serial.println("Waiting for GPS fix...");
      Serial.println(gps.charsProcessed());
    }

    if (gps.charsProcessed() < 10) {
      Serial.println("** No GPS data — check wiring or baud rate **");
    }
  }
}

// --- Connect to WIFI ---
void wifiConnect() {
  delay(100);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("================================");
  Serial.print("IP address: ");
  thisIP = WiFi.localIP().toString();
  Serial.println(WiFi.localIP());
  Serial.print("Sensor ID: ");
  Serial.println(SensorID);
  Serial.println("================================");
  delay(250);
}

void sendURL(double lat, double lng) {
  Serial.print("connecting to ");
  Serial.println(host);

  WiFiClient client;
  const int httpPort = 80;

  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  char latStr[10];
  char lonStr[10];
  dtostrf(lat, 8, 6, latStr);
  dtostrf(lng, 8, 6, lonStr);

  String url = "/beaconLog?";
  url += "sensorID=";
  url += SensorID;
  url += "&lat=";
  url += latStr;
  url += "&lon=";
  url += lonStr;

  Serial.print("Requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  // while (client.available()) {
  //   String line = client.readStringUntil('\r');
  //   Serial.print(line);
  // }
  client.stop();
}
