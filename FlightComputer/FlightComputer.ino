
// For Adafruit Feather M0 (SAMD21)
// Requires https://adafruit.github.io/arduino-board-index/package_adafruit_index.json in Settings > Additional Board Manager URLs

#include <Adafruit_GPS.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_ADS1X15.h>
#include <SD.h>

#include "Average.hpp"

// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define SDCardSelect 4

static constexpr uint32_t LOOP_PERIOD_MICROSECONDS { 5000 };
static constexpr uint32_t LOOPS_PER_1HZ { 200 };
static constexpr uint32_t LOOPS_PER_40HZ { 5 };
static constexpr uint32_t GPS_MESSAGE_WAIT_MILLISECONDS { 250 };
static constexpr int IMU_LOGGING_DIGITS { 8 };
static constexpr adsGain_t ADC_GAIN { GAIN_TWO };
static constexpr size_t ADC_CHANNEL_COUNT { 4 };
static constexpr int ADC_LOGGING_DIGITS { 8 };

// Connect to the GPS on the hardware port
Adafruit_GPS gps {&GPSSerial};
Adafruit_LSM9DS1 lsm {};
Adafruit_BMP3XX bmp {};
Adafruit_ADS1115 giraffe {};
Adafruit_ADS1115 zebra {};

sensors_event_t accel {};
sensors_event_t mag {};
sensors_event_t gyro {};
sensors_event_t temp {};

File gpsLog;
File log1Hz;
File imuLog;

uint32_t lastLoop { 0 };
uint32_t lastParsedMessage { 0 };
bool newMessage { false };

Average<float> giraffeAverages[ADC_CHANNEL_COUNT];
Average<float> zebraAverages[ADC_CHANNEL_COUNT];

void error() {
  while (true) {
    Serial.println("Error");
    delay(1000);
  }
}

void setup() {
  Serial.begin(9600);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB
  // }

  if (!SD.begin(SDCardSelect)) {
    Serial.println("Card init. failed!");
    error();
  }
  char filename[32];
  strcpy(filename, "/LOGS0000");
  for (uint8_t i = 0; i < 10000; i++) {
    filename[5] = '0' + i/1000;
    filename[6] = '0' + (i/100)%10;
    filename[7] = '0' + (i/10)%10;
    filename[8] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename)) {
      break;
    }
  }
  if (!SD.mkdir(filename)) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error();
  }

  strcpy(&filename[9], "/GPS.CSV");
  gpsLog = SD.open(filename, FILE_WRITE);
  if( ! gpsLog ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error();
  }

  strcpy(&filename[9], "/1HZ.CSV");
  log1Hz = SD.open(filename, FILE_WRITE);
  if( ! log1Hz ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error();
  }

  strcpy(&filename[9], "/IMU.CSV");
  imuLog = SD.open(filename, FILE_WRITE);
  if( ! imuLog ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error();
  }

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  gps.begin(9600);

  // Get all the data 5 times per second
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  
  if (!bmp.begin_I2C())
  {
    Serial.print("Failed to start BMP reading");
  }
  if (!lsm.begin())
  {
    Serial.print("Failed to start LSM reading");
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  giraffe.begin(0x48); // ADDR = GND
  zebra.begin(0x49); // ADDR = VIN

  giraffe.setGain(ADC_GAIN);
  zebra.setGain(ADC_GAIN);

  delay(1000);

  gpsLog.print("hour,");
  gpsLog.print("minute,");
  gpsLog.print("seconds,");
  gpsLog.print("latitude_fixed,");
  gpsLog.print("longitude_fixed,");
  gpsLog.print("altitude,");
  gpsLog.print("speed,");
  gpsLog.print("fixquality,");
  gpsLog.print("satellites");
  gpsLog.println();

  log1Hz.print("time,");
  log1Hz.print("x_magnetometer,");
  log1Hz.print("y_magnetometer,");
  log1Hz.print("z_magnetometer,");
  log1Hz.print("temperature,");
  log1Hz.print("pressure,");
  log1Hz.print("giraffe_0,");
  log1Hz.print("giraffe_1,");
  log1Hz.print("giraffe_2,");
  log1Hz.print("giraffe_3,");
  log1Hz.print("zebra_0,");
  log1Hz.print("zebra_1,");
  log1Hz.print("zebra_2,");
  log1Hz.print("zebra_3");
  log1Hz.println();

  imuLog.print("time,");
  imuLog.print("x_accel,");
  imuLog.print("y_accel,");
  imuLog.print("z_accel,");
  imuLog.print("x_gyro,");
  imuLog.print("y_gyro,");
  imuLog.print("z_gyro");
  imuLog.println();

  lastLoop = micros();
}

char const* getTimeString() {
  static char time[128];
  if (!gps.fix)
  {
    time[0] = '\0';
    return time;
  }
  uint32_t offset = static_cast<uint32_t>(gps.secondsSinceTime() * 1000.0);
  uint32_t milliseconds = static_cast<uint32_t>(gps.milliseconds) + offset;
  uint32_t seconds = static_cast<uint32_t>(gps.seconds) + (milliseconds / 1000);
  milliseconds = milliseconds % 1000;
  uint32_t minutes = static_cast<uint32_t>(gps.minute) + (seconds / 60);
  seconds = seconds % 60;
  uint32_t hour = static_cast<uint32_t>(gps.hour) + (minutes / 60);
  minutes = minutes % 60;
  uint32_t day = static_cast<uint32_t>(gps.day) + (hour / 24);
  hour = hour % 24;
  uint32_t month = static_cast<uint32_t>(gps.month);
  uint32_t year = static_cast<uint32_t>(gps.year)+2000;
  sprintf(time, "%04d-%02d-%02dT%02d:%02d:%02d.%03d", year, month, day, hour, minutes, seconds, milliseconds);
  return time;
}

void loopGps() {
  // Read data from the GPS - this is necessary
  (void)gps.read();

  // If we got something from the GPS parse it out
  if (gps.newNMEAreceived()) {
    if (!gps.parse(gps.lastNMEA())) {
      return;
    }
    lastParsedMessage = millis();
    newMessage = true;
  }

  if ((millis() - lastParsedMessage > GPS_MESSAGE_WAIT_MILLISECONDS) && newMessage){
    newMessage = false;

    gpsLog.print(gps.hour); gpsLog.print(",");
    gpsLog.print(gps.minute); gpsLog.print(",");
    gpsLog.print(gps.seconds); gpsLog.print(",");
    gpsLog.print(gps.latitude_fixed); gpsLog.print(",");
    gpsLog.print(gps.longitude_fixed); gpsLog.print(",");
    gpsLog.print(gps.altitude, 2); gpsLog.print(",");
    gpsLog.print(gps.speed, 2); gpsLog.print(",");
    gpsLog.print(gps.fixquality); gpsLog.print(",");
    gpsLog.print(gps.satellites); gpsLog.println();
    gpsLog.flush();
  }
}

void loopImu() {
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  imuLog.print(getTimeString()); imuLog.print(",");
  imuLog.print(accel.acceleration.x, IMU_LOGGING_DIGITS); imuLog.print(",");
  imuLog.print(accel.acceleration.y, IMU_LOGGING_DIGITS); imuLog.print(",");
  imuLog.print(accel.acceleration.z, IMU_LOGGING_DIGITS); imuLog.print(",");
  imuLog.print(gyro.gyro.x, IMU_LOGGING_DIGITS); imuLog.print(",");
  imuLog.print(gyro.gyro.y, IMU_LOGGING_DIGITS); imuLog.print(",");
  imuLog.print(gyro.gyro.z, IMU_LOGGING_DIGITS); imuLog.println();
}

void loop40Hz() {
  static size_t loopCounter { 0 };

  giraffeAverages[loopCounter].insert(giraffe.computeVolts(giraffe.readADC_SingleEnded(static_cast<uint8_t>(loopCounter))));
  zebraAverages[loopCounter].insert(zebra.computeVolts(zebra.readADC_SingleEnded(static_cast<uint8_t>(loopCounter))));

  loopCounter = (loopCounter + 1) % 4;
}

void loop1Hz() {
  bmp.performReading();

  log1Hz.print(getTimeString()); log1Hz.print(",");
  log1Hz.print(mag.magnetic.x, IMU_LOGGING_DIGITS); log1Hz.print(",");
  log1Hz.print(mag.magnetic.y, IMU_LOGGING_DIGITS); log1Hz.print(",");
  log1Hz.print(mag.magnetic.z, IMU_LOGGING_DIGITS); log1Hz.print(",");
  log1Hz.print(bmp.temperature); log1Hz.print(",");
  log1Hz.print(bmp.pressure); log1Hz.print(",");
  for (size_t i { 0 }; i < ADC_CHANNEL_COUNT; i++) {
    log1Hz.print(giraffeAverages[i].get(), ADC_LOGGING_DIGITS);
    log1Hz.print(",");
  }
  for (size_t i { 0 }; i < ADC_CHANNEL_COUNT; i++) {
    log1Hz.print(zebraAverages[i].get(), ADC_LOGGING_DIGITS);
    if (i != (ADC_CHANNEL_COUNT - 1)) {
      log1Hz.print(",");
    } else {
      log1Hz.println();
    }
  }

  log1Hz.flush();
  imuLog.flush();
}

void loop() {
  static uint32_t loopCounter { 0 };

  loopImu();
  loopGps();
  if ((loopCounter % LOOPS_PER_40HZ) == (LOOPS_PER_40HZ - 1)) {
    loop40Hz();
  }
  if ((loopCounter % LOOPS_PER_1HZ) == (LOOPS_PER_1HZ - 1)) {
    loop1Hz();
  }

  loopCounter = (loopCounter + 1) % LOOPS_PER_1HZ;
  lastLoop += LOOP_PERIOD_MICROSECONDS;
  while (static_cast<int32_t>(micros() - lastLoop) < 0) {
    // Busy loop until it is time for the next cycle
  }
}