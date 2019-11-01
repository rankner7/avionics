#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial1
#define GPSECHO true

uint32_t timer = millis();

Adafruit_GPS GPS(&GPSSerial);

Adafruit_BMP280 bmp;
double base_altitude = 0;
double* alti_offset = &base_altitude;
int packetNo = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(9600);

  GPS.begin(9600);

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

  // You can adjust which sentences to have the module emit, below
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  // Set the update rate
  // 1 Hz update rate
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  delay(1000);
}

void loop(void) 
{
  /*
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }*/
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
  
  if (timer > millis()) timer = millis();

  // approximately every 1 / 10 second or so, print out the current stats
  if (millis() - timer > 100) {
    timer = millis();
    GPS.parse(GPS.lastNEMA())
    gps.add(GPS.fix);
    gps.add(GPS.longitude);
    gps.add(GPS.latitude);
  }

  serializeJson(doc, Serial);
  Serial.println("");

  packetNo ++;
  delay(90);
}
