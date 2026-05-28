#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>  
#include <Preferences.h>
#include <WebServer.h>
#include "index.h"

#define SSID_casa "MIWIFI_CD65-2,4g"
#define PASSWORD_casa "ZTT9QJG6"

const char* ssid = SSID_casa;
const char* password = PASSWORD_casa;

//Contadores para calcular RMS
unsigned long pitch_counter = 0;
unsigned long roll_counter = 0;
unsigned long accelH_counter = 0;
unsigned long accelV_counter = 0;

//Lectura de calibración cada: 5s
unsigned long timer_last_calibration = 0;
unsigned long timer_calibration = 5000;
//Lectura de datos cada: 50ms
unsigned long timer_last_data = 0;
unsigned long timer_data = 50;
//Escritura en terminal cada: 1s
unsigned long timer_last_print = 0;
unsigned long timer_print = 5000;

IPAddress Ip(192, 168, 1, 222);
IPAddress NMask(255, 255, 255, 0);

String header;
String readString; 

WebServer server(80);
WiFiClient fosaclient;
PubSubClient client(fosaclient);

Preferences preferences;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);


//Datos de interes 
struct Data
{
  float pitch_actual, pitch_pico, pitch_rms, pitch_offset,
  roll_actual, roll_pico, roll_rms, roll_offset,
  accelH_actual, accelH_pico, accelH_rms, accelH_offset,
  accelV_actual, accelV_pico, accelV_rms, accelV_offset;
} BNO_data;

//Calibracion del sensor 0-3 (3 = Fully calibrated)
struct Calibration
{
  uint8_t sys, gyro, accel, mag;

} BNO_calibration;
//Configuracion de la calibracion, se pierde al apagar el BNO
struct Configuration
{
  adafruit_bno055_offsets_t e;

} config;
//Datos de comunicacion
struct Settings 
{
  char ip[16]; char gw[16]; char sub[16];
  char web_user[16]; char web_pass[16];
  bool mb_en; int mb_port; bool udp_en;
  char udp_ip[16]; int udp_port; int udp_interval;
  bool mqtt_en; char mqtt_host[16]; int mqtt_port;
  char mqtt_user[16]; char mqtt_pass[16]; int mqtt_interval;

} settings;

void cargarDefaults() 
{
  strcpy(settings.ip, "192.168.95.11");
  strcpy(settings.gw, "192.168.95.1");
  strcpy(settings.sub, "255.255.255.0");
  strcpy(settings.web_user, "admin");
  strcpy(settings.web_pass, "admin");
  settings.mb_en = true;
  settings.mb_port = 502;
  settings.udp_en = false;
  strcpy(settings.udp_ip, "192.168.95.100");
  settings.udp_port = 1234;
  settings.udp_interval = 5;
  settings.mqtt_en = true;
  strcpy(settings.mqtt_host, "192.168.1.141");
  settings.mqtt_port = 1883;
  strcpy(settings.mqtt_user, "admin");
  strcpy(settings.mqtt_pass, "admin");
  settings.mqtt_interval = 5;
}

void get_data()
{
  sensors_event_t orientationData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  //Calculo de pitch
  float pitch_offset_value = orientationData.orientation.y - BNO_data.pitch_offset;
  float pitch_abs_value = abs(pitch_offset_value);
  pitch_counter++;
  BNO_data.pitch_actual = pitch_offset_value;
  BNO_data.pitch_pico = max(pitch_abs_value, BNO_data.pitch_pico);
  BNO_data.pitch_rms -= BNO_data.pitch_rms / pitch_counter;
  BNO_data.pitch_rms += pitch_abs_value / pitch_counter;
  //Calculo de roll
  float roll_offset_value = orientationData.orientation.z - BNO_data.roll_offset;
  float roll_abs_value = abs(roll_offset_value);
  roll_counter++;
  BNO_data.roll_actual = roll_offset_value;
  BNO_data.roll_pico = max(roll_abs_value, BNO_data.roll_pico);
  BNO_data.roll_rms -= BNO_data.roll_rms / roll_counter;
  BNO_data.roll_rms += roll_abs_value / roll_counter;
  //Calculo de aceleración horizontal
  float accelH_offset_value = linearAccelData.acceleration.x - BNO_data.accelH_offset;
  float accelH_abs_value = abs(accelH_offset_value);
  accelH_counter++;
  BNO_data.accelH_actual = accelH_offset_value;
  BNO_data.accelH_pico = max(accelH_abs_value, BNO_data.accelH_pico);
  BNO_data.accelH_rms -= BNO_data.accelH_rms / accelH_counter;
  BNO_data.accelH_rms += accelH_abs_value / accelH_counter;
  //Calculo de aceleración vertical
  float accelV_offset_value = linearAccelData.acceleration.y - BNO_data.accelV_offset;
  float accelV_abs_value = abs(accelV_offset_value);
  accelV_counter++;
  BNO_data.accelV_actual = accelV_offset_value;
  BNO_data.accelV_pico = max(accelV_abs_value, BNO_data.accelV_pico);
  BNO_data.accelV_rms -= BNO_data.accelV_rms / accelV_counter;
  BNO_data.accelV_rms += accelV_abs_value / accelV_counter;
}

void send_data_mqtt()
{
  
}
void get_calibration()
{
  bno.getCalibration(&BNO_calibration.sys, &BNO_calibration.gyro, &BNO_calibration.accel, &BNO_calibration.mag);
}
void print_terminal()
{
  //Serial.print("\n\n\n\n\n\n\n\n");
  Serial.print("\n------------DATA------------");
  Serial.print("\n[ACTUAL]  ");
  Serial.print("Pitch: " + String(BNO_data.pitch_actual) + " || Roll: " + String(BNO_data.roll_actual)
                + " || AccelH: " + String(BNO_data.accelH_actual) + " || AccelV: " + String(BNO_data.accelV_actual));
  Serial.print("\n[PICO]    ");
  Serial.print("Pitch: " + String(BNO_data.pitch_pico) + " || Roll: " + String(BNO_data.roll_pico)
                + " || AccelH: " + String(BNO_data.accelH_pico) + " || AccelV: " + String(BNO_data.accelV_pico));
  Serial.print("\n[RMS]     ");
  Serial.print("Pitch: " + String(BNO_data.pitch_rms) + " || Roll: " + String(BNO_data.roll_rms)
                + " || AccelH: " + String(BNO_data.accelH_rms) + " || AccelV: " + String(BNO_data.accelV_rms));
  Serial.print("\n----------------------------");
  Serial.print("\n--------CALIBRACION---------");
  Serial.print("\nSys: " + String(BNO_calibration.sys) + " || Gyro: " + String(BNO_calibration.gyro)
                + " || Accel: " + String(BNO_calibration.accel) + " || Mag: " + String(BNO_calibration.mag));
  Serial.print("\n----------------------------");
}

void serial_setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);
}

void BNO055_setup()
{
  if(!bno.begin())
  {
    Serial.print("BNO055 not detected");
  }else
  {
    bno.setExtCrystalUse(true);
    Serial.println("\nBNO-055 setup complete");
  }
  
}
void wifi_setup_sta()
{
  WiFi.mode(WIFI_STA);   
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting to Wifi");
  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(50);
  }
  WiFi.config(Ip, Ip, NMask);
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  delay(1000);
}

void wifi_setup_ap()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32||BNO-055", NULL);
  Serial.println("\nSetting Wifi access point");
  
  WiFi.softAPConfig(Ip, Ip, NMask);
  Serial.print("ESP32 access point IP address: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
}
void handleRoot() 
{
  server.send(200, "text/HTML", HTML_CONTENT);
}
void setup()
{
  serial_setup();
  BNO055_setup();
  wifi_setup_sta();
  server.on("/", handleRoot);

  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });


  server.begin();
  Serial.println("HTTP server started");
  delay(5000);
}

void loop()
{
  

  if ((millis() - timer_last_calibration) > timer_calibration) 
  {
    get_calibration();
    timer_last_calibration = millis();
  }

  if ((millis() - timer_last_data) > timer_data) 
  {
    get_data();
    timer_last_data = millis();
  }

  if ((millis() - timer_last_print) > timer_print) 
  {
    print_terminal();
    timer_last_print = millis();
  }
  server.handleClient();
  delay(50);
}