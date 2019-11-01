#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
//#include <Adafruit_GPS.h>

Adafruit_BMP280 bmp;
double base_altitude = 0;
double* alti_offset = &base_altitude;
int packetNo = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55);


void setup(void) 
{
  Serial.begin(9600);

  if(!bno.begin()){
    Serial.print("No BNO055 Detected!");
    while(1);
  }

  if (!bmp.begin()) {
    Serial.println("No BMP280 Detected!");
    while (1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  *alti_offset = bmp.readAltitude(1013.25);
  bno.setExtCrystalUse(true);
  
  delay(1000);
}

void loop(void) 
{
  imu::Quaternion quat = bno.getQuat();           // Request quaternion data from BNO055
  imu::Vector<3> vaccel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gvector = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> magneto = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  StaticJsonDocument<414> doc;
  JsonArray header = doc.createNestedArray("header");
  JsonArray alt = doc.createNestedArray("tpa");
  JsonArray imu = doc.createNestedArray("imu");

  header.add(packetNo);
  header.add(millis());

  tpa.add(bmp.readTemperature());       
  tpa.add(bmp.readPressure());
  tap.add((bmp.readAltitude(1013.25))-(*alti_offset+.1));

  imu.add(vaccel.x());
  imu.add(vaccel.y());
  imu.add(vaccel.z());
  imu.add(quat.w());
  imu.add(quat.x());
  imu.add(quat.y());
  imu.add(quat.z());
  imu.add(laccel.x());
  imu.add(laccel.y());
  imu.add(laccel.z());
  imu.add(gvector.x());
  imu.add(gvector.y());
  imu.add(gvector.z());
  imu.add(angular_vel.x());
  imu.add(angular_vel.y());
  imu.add(angular_vel.z());
  imu.add(magneto.x());
  imu.add(magneto.y());
  imu.add(magneto.z());

  serializeJsonPretty(root, Serial);

  packetNo ++;
  delay(90);
}
