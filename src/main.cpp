#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>  
#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Ethernet.h>      
#include <ModbusIP_ESP8266.h>
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

//Escritura en terminal cada: 2s
unsigned long timer_last_print = 0;
unsigned long timer_print = 2000;


unsigned long timer_pico = 0;
unsigned long timer_mb_tcp = 0;
unsigned long timer_bno055 = 0;


ModbusIP mb;
AsyncWebServer server(80);
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
  IPAddress IP, GW, SUB;
  int mb_tcp_samplerate, bno055_samplerate, mqtt_samplerate;
  int mb_tcp_port, mqtt_port;
  bool mb_tcp_en, mqtt_en;
} settings;

void cargar_default_settings() 
{
  settings.IP = IPAddress(192, 168, 1, 222);
  settings.GW = IPAddress(192, 168, 1, 222);
  settings.SUB = IPAddress(255, 255, 255, 0);
  settings.bno055_samplerate = 25;
  settings.mb_tcp_samplerate = 250;
  settings.mb_tcp_en = true;
  settings.mqtt_en = true;
  settings.mb_tcp_port = 502;
  settings.mqtt_port = 1883;
}

void get_data()
{//falta reset del contador rms para evitar overflow
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
  //
}
void send_float(uint16_t direccion, float valor) {
    union 
    {
      float f;
      uint16_t regs[2];
    } conversor;

    conversor.f = valor;
   
    mb.Hreg(direccion, conversor.regs[0]);
    mb.Hreg(direccion + 1, conversor.regs[1]);
}
void send_data_modbus_tcp()
{
  send_float(0, BNO_data.pitch_actual);
  send_float(2, BNO_data.roll_actual);
  send_float(4, BNO_data.accelH_actual);
  send_float(6, BNO_data.accelV_actual);
  send_float(8, BNO_data.pitch_pico);
  send_float(10, BNO_data.roll_pico);
  send_float(12, BNO_data.accelH_pico);
  send_float(14, BNO_data.accelV_pico);
  send_float(16, BNO_data.pitch_rms);
  send_float(18, BNO_data.roll_rms);
  send_float(20, BNO_data.accelH_rms);
  send_float(22, BNO_data.accelV_rms);
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
  Serial.print("\n--------CALIBRACION---------");
  Serial.print("\nSys: " + String(BNO_calibration.sys) + " || Gyro: " + String(BNO_calibration.gyro)
                + " || Accel: " + String(BNO_calibration.accel) + " || Mag: " + String(BNO_calibration.mag));
  Serial.print("\n--------SAMPLE RATES--------");
  Serial.print("\nBNO-055: " + String(settings.bno055_samplerate) + "ms || Modbus TCP: " + String(settings.mb_tcp_samplerate) + "ms");
  Serial.print("\n----------SETTINGS----------");
  Serial.print("\n----------------------------");
}

void notFound(AsyncWebServerRequest *request) 
{
  request->send(404, "text/plain", "Not found");
}

void serial_setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);
}

void modbus_tcp_setup()
{
  mb.server();
  mb.addHreg(0, 0, 32);
  
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
void nvmemory_setup()
{
  cargar_default_settings();
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
  WiFi.config(settings.IP, settings.GW, settings.SUB);
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void wifi_setup_ap()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32||BNO-055", NULL);
  Serial.println("\nSetting Wifi access point");
  
  WiFi.softAPConfig(settings.IP, settings.GW, settings.SUB);
  Serial.print("ESP32 access point IP address: ");
  Serial.println(WiFi.softAPIP());
}

void webserver_setup()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(200, "text/html", HTML_CONTENT);
  });
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request)
  { 
    if (request->hasParam("pitch")) 
    {
      BNO_data.pitch_offset = (request->getParam("pitch")->value()).toFloat();
    }
    else if (request->hasParam("roll")) 
    {
      BNO_data.roll_offset = (request->getParam("roll")->value()).toFloat();
    }
    else if (request->hasParam("accelH")) 
    {
      BNO_data.accelH_offset = (request->getParam("accelH")->value()).toFloat();
    }
    else if (request->hasParam("accelV")) 
    {
      BNO_data.accelV_offset = (request->getParam("accelV")->value()).toFloat();
    }
    else if (request->hasParam("settings")) 
    {
      if (request->getParam("settings")->value() == "ip")
      {
        settings.IP.fromString(request->getParam("param")->value());
      }
      else if (request->getParam("settings")->value() == "gw")
      {
        settings.GW.fromString(request->getParam("param")->value());
      }
      else if (request->getParam("settings")->value() == "sub")
      {
        settings.SUB.fromString(request->getParam("param")->value());
      }
      else if (request->getParam("settings")->value() == "bno_samplerate")
      {
        settings.bno055_samplerate = (request->getParam("param")->value()).toInt();
      }
      else if (request->getParam("settings")->value() == "mb_tcp_samplerate")
      {
        settings.mb_tcp_samplerate = (request->getParam("param")->value()).toInt();
      }
    }
    request->send(200, "text/html", HTML_CONTENT);
  });
  server.onNotFound(notFound);
  server.begin();
}

void setup()
{
  serial_setup();
  nvmemory_setup();
  BNO055_setup();
  wifi_setup_sta();
  webserver_setup();
  modbus_tcp_setup();
  delay(5000);
}

void loop()
{
  mb.task();
  if ((millis() - timer_last_calibration) > timer_calibration) 
  {
    get_calibration();
    timer_last_calibration = millis();
  }

  if ((millis() - timer_bno055) > settings.bno055_samplerate) 
  {
    get_data();
    timer_bno055 = millis();
  }

  if ((millis() - timer_last_print) > timer_print) 
  {
    print_terminal();
    timer_last_print = millis();
  }
  if ((millis() - timer_mb_tcp) > settings.mb_tcp_samplerate) 
  {
    send_data_modbus_tcp();
    timer_mb_tcp = millis();
  }
  if ((millis() - timer_pico) > settings.mb_tcp_samplerate*2) 
  {
    BNO_data.pitch_pico = BNO_data.pitch_actual;
    BNO_data.roll_pico = BNO_data.roll_actual;
    BNO_data.accelH_pico = BNO_data.accelH_actual;
    BNO_data.accelV_pico = BNO_data.accelV_actual;
    timer_pico = millis();
  }
  delay(10);
}