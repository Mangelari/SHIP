#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>  
#include <Preferences.h>


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
unsigned long timer_print = 1000;


WiFiClient fosaclient;
PubSubClient client(fosaclient);
Preferences preferences;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);


//Estructura de datos de interes 
struct Data
{
  float pitch_actual, pitch_pico, pitch_rms, pitch_offset,
  roll_actual, roll_pico, roll_rms, roll_offset,
  accelH_actual, accelH_pico, accelH_rms, accelH_offset,
  accelV_actual, accelV_pico, accelV_rms, accelV_offset;
} BNO_data;

//Estructura de calibración
struct Calibration
{
  uint8_t sys, gyro, accel, mag;

} BNO_calibration;
struct Config 
{
  char ip[16]; char gw[16]; char sub[16];
  char web_user[16]; char web_pass[16];
  bool mb_en; int mb_port; bool udp_en;
  char udp_ip[16]; int udp_port; int udp_interval;
  bool mq_en; char mq_host[16]; int mq_port;
  char mq_user[16]; char mq_pass[16]; int mq_interval;

} settings;
void cargar_defaults() 
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
  settings.mq_en = false;
  strcpy(settings.mq_host, "192.168.95.100");
  settings.mq_port = 1883;
  strcpy(settings.mq_user, "admin");
  strcpy(settings.mq_pass, "admin");
  settings.mq_interval = 5;
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
void get_calibration()
{
  bno.getCalibration(&BNO_calibration.sys, &BNO_calibration.gyro, &BNO_calibration.accel, &BNO_calibration.mag);
}
void print_terminal()
{
  Serial.print("\n\n\n\n\n\n\n\n");
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

void setup()
{
  serial_setup();
  BNO055_setup();
  delay(3000);
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
}