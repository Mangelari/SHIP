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
#include <math.h>
#include "index.h"

#define SSID_casa "MIWIFI_CD65-2,4g"
#define PASSWORD_casa "ZTT9QJG6"
#define API_DB "qwnsAEun-9u8aHaiPBWL8xFaULYeECKWYag1iMPJo1yy_p7sAtzWhI3L3jKdhpq6cD3yuVmh2HTesmNhpBIVPQ=="

const char* ssid = SSID_casa;
const char* password = PASSWORD_casa;

//Contadores para calcular RMS
float pitch_sum = 0;
float roll_sum = 0;
float accelH_sum = 0;
float accelV_sum = 0;
unsigned long counter = 0;

//Lectura de calibración cada: 5s
unsigned long timer_last_calibration = 0;
unsigned long timer_calibration = 5000;

//Escritura en terminal cada: 2s
unsigned long timer_last_print = 0;
unsigned long timer_print = 2000;


unsigned long timer_pico = 0;
unsigned long timer_mb_tcp = 0;
unsigned long timer_bno055 = 0;
unsigned long timer_mqtt = 0;


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
  accelV_actual, accelV_pico, accelV_rms, accelV_offset,
  msi, msdv;
} BNO_data;

//Calibracion del sensor 0-3 (3 = Fully calibrated)
struct Calibration
{
  uint8_t sys, gyro, accel, mag;

} BNO_calibration;
//Configuracion de la calibracion, se pierde al apagar el BNO
struct Configuration
{
  adafruit_bno055_offsets_t calibration;

} config;
//Datos de comunicacion
struct Settings 
{
  IPAddress IP, GW, SUB;
  char* mqtt_server;
  int mb_tcp_samplerate, bno055_samplerate, mqtt_samplerate;
  int mb_tcp_port, mqtt_port;
  bool mb_tcp_en, mqtt_en;
} settings;

void load_default_settings() 
{
  settings.IP = IPAddress(192, 168, 1, 222);
  settings.GW = IPAddress(192, 168, 1, 222);
  settings.SUB = IPAddress(255, 255, 255, 0);
  settings.mqtt_server = "192.168.1.223";
  settings.bno055_samplerate = 50;
  settings.mb_tcp_samplerate = 250;
  settings.mqtt_samplerate = 1000;
  settings.mb_tcp_en = true;
  settings.mqtt_en = true;
  settings.mb_tcp_port = 502;
  settings.mqtt_port = 1883;
}

void save_settings_nvmemory()
{
  preferences.begin("settings", false);
  preferences.putBytes("IP", &settings.IP, sizeof(settings.IP));
  preferences.putBytes("GW", &settings.GW, sizeof(settings.GW));
  preferences.putBytes("SUB", &settings.SUB, sizeof(settings.SUB));
  preferences.putBytes("BNO055_SR", &settings.bno055_samplerate, sizeof(settings.bno055_samplerate));
  preferences.putBytes("MB_TCP_SR", &settings.mb_tcp_samplerate, sizeof(settings.mb_tcp_samplerate));
  preferences.putBytes("MQTT_SR", &settings.mqtt_samplerate, sizeof(settings.mqtt_samplerate));
  preferences.end();
}
void load_settings_nvmemory()
{
  preferences.begin("settings", false);
  preferences.getBytes("IP", &settings.IP, sizeof(settings.IP));
  preferences.getBytes("GW", &settings.GW, sizeof(settings.GW));
  preferences.getBytes("SUB", &settings.SUB, sizeof(settings.SUB));
  preferences.getBytes("BNO055_SR", &settings.bno055_samplerate, sizeof(settings.bno055_samplerate));
  preferences.getBytes("MB_TCP_SR", &settings.mb_tcp_samplerate, sizeof(settings.mb_tcp_samplerate));
  preferences.getBytes("MQTT_SR", &settings.mqtt_samplerate, sizeof(settings.mqtt_samplerate));
  preferences.end();
}
void save_calibration_nvmemory()
{
  bno.getSensorOffsets(config.calibration);
  preferences.begin("calibration", false);
  preferences.putBytes("CALIBRATION", &config.calibration, sizeof(config.calibration));
  preferences.end();
}

void load_calibration_nvmemory()
{
  preferences.begin("calibration", true);
  preferences.getBytes("CALIBRATION", &config.calibration, sizeof(config.calibration));
  bno.setSensorOffsets(config.calibration);
  preferences.end();
}

void get_data()
{
  sensors_event_t orientationData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  counter++;
  //Calculo de pitch
  float pitch_woffset_value = orientationData.orientation.y - BNO_data.pitch_offset;
  float pitch_abs_value = abs(pitch_woffset_value);
  BNO_data.pitch_actual = pitch_woffset_value;
  if(abs(BNO_data.pitch_pico) < pitch_abs_value) BNO_data.pitch_pico = pitch_woffset_value;
  pitch_sum += pitch_abs_value;
  BNO_data.pitch_rms = pitch_sum / counter;
  //Calculo de roll
  float roll_woffset_value = orientationData.orientation.z - BNO_data.roll_offset;
  float roll_abs_value = abs(roll_woffset_value);
  BNO_data.roll_actual = roll_woffset_value;
  if(abs(BNO_data.roll_pico) < roll_abs_value) BNO_data.roll_pico = roll_woffset_value;
  roll_sum += roll_abs_value;
  BNO_data.roll_rms = roll_sum / counter;
  //Calculo de aceleración horizontal
  float accelH_woffset_value = linearAccelData.acceleration.x - BNO_data.accelH_offset;
  float accelH_abs_value = abs(accelH_woffset_value);
  BNO_data.accelH_actual = accelH_woffset_value;
  if(abs(BNO_data.accelH_pico) < accelH_abs_value) BNO_data.accelH_pico = accelH_woffset_value;
  accelH_sum += accelH_abs_value;
  BNO_data.accelH_rms = accelH_sum / counter;
  //Calculo de aceleración vertical
  float accelV_woffset_value = linearAccelData.acceleration.y - BNO_data.accelV_offset;
  float accelV_abs_value = abs(accelV_woffset_value);
  BNO_data.accelV_actual = accelV_woffset_value;
  if(abs(BNO_data.accelV_pico) < accelV_abs_value) BNO_data.accelV_pico = accelV_woffset_value;
  accelV_sum += accelV_abs_value;
  BNO_data.accelV_rms = accelV_sum / counter;
  BNO_data.msdv = sqrt(pow(BNO_data.accelV_rms, 2) * millis() / 1000);
  BNO_data.msi = 100 * (1 - exp(-pow(BNO_data.msdv/0.2, 2)));
  
}

void get_calibration()
{
  bno.getCalibration(&BNO_calibration.sys, &BNO_calibration.gyro, &BNO_calibration.accel, &BNO_calibration.mag);
}

void print_terminal()
{
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
  Serial.print("\n[MSDV]    "   + String(BNO_data.msdv) );
  Serial.print("\n[MSI]    "   + String(BNO_data.msi) );
  Serial.print("\n--------CALIBRACION---------");
  Serial.print("\nSys: " + String(BNO_calibration.sys) + " || Gyro: " + String(BNO_calibration.gyro)
                + " || Accel: " + String(BNO_calibration.accel) + " || Mag: " + String(BNO_calibration.mag));
  Serial.print("\n--------SAMPLE RATES--------");
  Serial.print("\nBNO-055: " + String(settings.bno055_samplerate) + "ms || Modbus TCP: " + String(settings.mb_tcp_samplerate)
                + "ms ||  MQTT: " + String(settings.mqtt_samplerate) + "ms");
  Serial.print("\n----------SETTINGS----------");
  Serial.print("\nIP: " + settings.IP.toString() + " || GW: " + settings.GW.toString() + " || SUB: " + settings.SUB.toString());
  Serial.print("\n----------------------------");
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void send_data_mqtt()
{
  JsonDocument doc_actual;
  doc_actual["pitch_actual"] = BNO_data.pitch_actual;
  doc_actual["roll_actual"] = BNO_data.roll_actual;
  doc_actual["accelH_actual"] = BNO_data.accelH_actual;
  doc_actual["accelV_actual"] = BNO_data.accelV_actual;
  JsonDocument doc_pico;
  doc_pico["pitch_pico"] = BNO_data.pitch_pico;
  doc_pico["roll_pico"] = BNO_data.roll_pico;
  doc_pico["accelH_pico"] = BNO_data.accelH_pico;
  doc_pico["accelV_pico"] = BNO_data.accelV_pico;
  JsonDocument doc_rms;
  doc_rms["pitch_rms"] = BNO_data.pitch_rms;
  doc_rms["roll_rms"] = BNO_data.roll_rms;
  doc_rms["accelH_rms"] = BNO_data.accelH_rms;
  doc_rms["accelV_rms"] = BNO_data.accelV_rms;
  JsonDocument doc_calibration;
  doc_calibration["sys"] = BNO_calibration.sys;
  doc_calibration["gyro"] = BNO_calibration.gyro;
  doc_calibration["accel"] = BNO_calibration.accel;
  doc_calibration["mag"] = BNO_calibration.mag;
  JsonDocument doc_index;
  doc_index["msdv"] = BNO_data.msdv;
  doc_index["msi"] = BNO_data.msi;
  char doc_actual_buffer[256];
  serializeJson(doc_actual , doc_actual_buffer);
  client.publish("BNO055/actual", doc_actual_buffer);
  char doc_pico_buffer[256];
  serializeJson(doc_pico , doc_pico_buffer);
  client.publish("BNO055/pico", doc_pico_buffer);
  char doc_rms_buffer[256];
  serializeJson(doc_rms , doc_rms_buffer);
  client.publish("BNO055/rms", doc_rms_buffer);
  char doc_calibration_buffer[256];
  serializeJson(doc_calibration , doc_calibration_buffer);
  client.publish("BNO055/calibration", doc_calibration_buffer);
  char doc_index_buffer[256];
  serializeJson(doc_index , doc_index_buffer);
  client.publish("BNO055/index", doc_index_buffer);
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
  send_float(24, BNO_data.msdv);
  send_float(26, BNO_data.msi);
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
void reconnect() 
{
  while (!client.connected()) 
  {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client")) 
    { 
      Serial.println("Connected!");

    }else 
    {
      Serial.print("Failed, rc=");
      Serial.print(client.state()); 
      Serial.println(" retrying in 2 seconds");
      delay(2000); 
    }
  }
}
void mqtt_setup()
{
  Serial.println("\nInitializing MQTT protocol");
  client.setServer(settings.mqtt_server, settings.mqtt_port);
  client.setCallback(callback);
  if (!client.connected()) 
  {
    reconnect();
  }
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
  load_default_settings();
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
  WiFi.softAP("ESP32||BNO-055", "alumnofosa");
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
      BNO_data.pitch_offset += (request->getParam("pitch")->value()).toFloat();
    }
    else if (request->hasParam("roll")) 
    {
      BNO_data.roll_offset += (request->getParam("roll")->value()).toFloat();
    }
    else if (request->hasParam("accelH")) 
    {
      BNO_data.accelH_offset += (request->getParam("accelH")->value()).toFloat();
    }
    else if (request->hasParam("accelV")) 
    {
      BNO_data.accelV_offset += (request->getParam("accelV")->value()).toFloat();
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
      else if (request->getParam("settings")->value() == "mqtt_samplerate")
      {
        settings.mqtt_samplerate = (request->getParam("param")->value()).toInt();
      }
    }
    else if (request->hasParam("save"))
    {
      Serial.println("Saving settings to non-volatile memory");
        save_calibration_nvmemory();
    }
     else if (request->hasParam("save"))
    {
      Serial.println("Saving settings to non-volatile memory");
        load_calibration_nvmemory();
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
  wifi_setup_ap();
  webserver_setup();
  modbus_tcp_setup();
  //mqtt_setup();
  delay(5000);
}

void loop()
{
  mb.task();
  //client.loop();

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
  if ((millis() - timer_mqtt) > settings.mqtt_samplerate) 
  {
    //reconnect();
    timer_mqtt = millis();
    //send_data_mqtt();
  }
  if ((millis() - timer_pico) > settings.mqtt_samplerate*2) 
  {
    BNO_data.pitch_pico = BNO_data.pitch_actual;
    BNO_data.roll_pico = BNO_data.roll_actual;
    BNO_data.accelH_pico = BNO_data.accelH_actual;
    BNO_data.accelV_pico = BNO_data.accelV_actual;
    timer_pico = millis();
  }
  delay(10);
}