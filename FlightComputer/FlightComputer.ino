
// For Adafruit Feather M0 (SAMD21)
// Requires https://adafruit.github.io/arduino-board-index/package_adafruit_index.json in Settings > Additional Board Manager URLs

#include <Adafruit_GPS.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_ADS1X15.h>
#include <MS5611.h>
#include <SD.h>
#include <cmath>

#include "Average.hpp"

// Uncomment this to enable calibration mode
#define THERMISTOR_CALIBRATION

// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define SDCardSelect 4

static constexpr uint32_t LOOP_PERIOD_MICROSECONDS { 5000 };
static constexpr uint32_t LOOPS_PER_1HZ { 200 };
static constexpr uint32_t LOOPS_PER_100HZ { 2 };
static constexpr uint32_t GPS_MESSAGE_WAIT_MILLISECONDS { 250 };
static constexpr int IMU_LOGGING_DIGITS { 8 };
static constexpr adsGain_t ADC_GAIN { GAIN_TWO };
static constexpr size_t ANALOG_COUNT { 10 };
static constexpr int ADC_LOGGING_DIGITS { 8 };
static constexpr int MS5611_LOGGING_DIGITS { 8 };
static constexpr uint32_t LED_PIN { 5 };
static constexpr int ADC_BITS { 12 };
static constexpr double KELVIN_CELSIUS_OFFSET { 273.15 };
static constexpr double ADC_VOLTAGE { 3.3 };
static constexpr double ADC_DIVISOR { static_cast<double>((1 << ADC_BITS) - 1) };
static constexpr double THERMISTOR_RESISTOR_VALUE {100000};
static constexpr double THERMISTOR_HIGH_VOLTAGE {3.3};
static constexpr double STEINHART_HART_CONSTANTS[ANALOG_COUNT][3] {
  //  A               B               C
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 },
  { 1.122919524e-3, 2.357963107e-4, 0.7559000491e-7 }
};

// Connect to the GPS on the hardware port
Adafruit_GPS gps { &GPSSerial };
Adafruit_LSM9DS1 lsm {};
MS5611 ms5611 { 0x77 };
Adafruit_ADS1115 zebra {};

Average<double> analogAverages[ANALOG_COUNT];

sensors_event_t accel {};
sensors_event_t mag {};
sensors_event_t gyro {};
sensors_event_t temp {};

File gpsLog;
File log1Hz;
File imuLog;
File diagnosticLog;

uint32_t lastLoop { 0 };
uint32_t lastParsedMessage { 0 };
uint32_t loopCounter { 0 };
bool newMessage { false };

void halt(char const* message, char const* message2) {
  if (diagnosticLog) {
    diagnosticLog.print(message);
    diagnosticLog.println(message2);
    diagnosticLog.flush();
  }
  while (true) {
    Serial.print(message);
    Serial.println(message2);
    delay(1000);
  }
}
void halt(char const* message) {
  halt("", message);
}

void setup() {
  Serial.begin(9600);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB
  // }

  if (!SD.begin(SDCardSelect)) {
    halt("SD Card initialization failed!");
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
    halt("Couldn't create ", filename);
  }

  strcpy(&filename[9], "/DIA.LOG");
  diagnosticLog = SD.open(filename, FILE_WRITE);

  strcpy(&filename[9], "/GPS.CSV");
  gpsLog = SD.open(filename, FILE_WRITE);
  if( ! gpsLog ) {
    halt("Couldn't create ", filename);
  }

  strcpy(&filename[9], "/1HZ.CSV");
  log1Hz = SD.open(filename, FILE_WRITE);
  if( ! log1Hz ) {
    halt("Couldn't create ", filename);
  }

  strcpy(&filename[9], "/IMU.CSV");
  imuLog = SD.open(filename, FILE_WRITE);
  if( ! imuLog ) {
    halt("Couldn't create ", filename);
  }

  diagnosticLog.println("Attempting to Initialize Hardware");
  diagnosticLog.flush();
  Serial.println("Attempting to Initialize Hardware");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  if (!gps.begin(9600)) {
    halt("Failed to start GPS");
  }

  // Get all the data 5 times per second
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);

  if (!lsm.begin())
  {
    halt("Failed to start LSM");
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  if (!zebra.begin(0x49)) { // ADDR = VIN
    halt("Failed to start Zebra");
  }
  zebra.setGain(ADC_GAIN);

  if (!ms5611.begin()){
    halt("Failed to start MS5611");
  }

  pinMode(LED_PIN, OUTPUT); // Enable LED output pin
  analogReadResolution(ADC_BITS); // Set adc resolution

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
  log1Hz.print("ms5611_temperature,");
  log1Hz.print("ms5611_pressure");
  for (size_t i { 0 }; i < ANALOG_COUNT; i++) {
#ifdef THERMISTOR_CALIBRATION
    log1Hz.print(",voltage_");
    log1Hz.print(i);
    log1Hz.print(",resistance_");
    log1Hz.print(i);
#endif
    log1Hz.print(",temperature_");
    log1Hz.print(i);
  }
  log1Hz.println();

  imuLog.print("time,");
  imuLog.print("x_accel,");
  imuLog.print("y_accel,");
  imuLog.print("z_accel,");
  imuLog.print("x_gyro,");
  imuLog.print("y_gyro,");
  imuLog.print("z_gyro");
  imuLog.println();

  diagnosticLog.println("Initialized Successfully");
  diagnosticLog.flush();
  Serial.println("Initialized Successfully");

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

double readAnalogPin(uint32_t pin) {
  return static_cast<double>(analogRead(pin)) / ADC_DIVISOR * ADC_VOLTAGE;
}

double readZebraChannel(uint8_t channel) {
  return static_cast<double>(zebra.computeVolts(zebra.readADC_SingleEnded(channel)));
}

double readAnalog(size_t channel) {
  switch (channel) {
    case 0:
      return readAnalogPin(A5);
    case 1:
      return readAnalogPin(A4);
    case 2:
      return readAnalogPin(A3);
    case 3:
      return readAnalogPin(A2);
    case 4:
      return readAnalogPin(A1);
    case 5:
      return readAnalogPin(A0);
    case 6:
      return readZebraChannel(3);
    case 7:
      return readZebraChannel(2);
    case 8:
      return readZebraChannel(1);
    case 9:
      return readZebraChannel(0);
    default:
      return NAN;
  }
  return NAN;
}

double steinhart_hart(double resistance, double a, double b, double c) {
  double lnResistance { log(resistance) };
  return 1.0 / (a + b * lnResistance + c * lnResistance * lnResistance * lnResistance);
}

double voltage_to_resistance(double voltage) {
  return THERMISTOR_RESISTOR_VALUE * (THERMISTOR_HIGH_VOLTAGE - voltage) / voltage;
}

void loop100Hz() {
  static size_t loopCounter { 0 };

  analogAverages[loopCounter].insert(readAnalog(loopCounter));

  loopCounter = (loopCounter + 1) % 10;
}

void loop1Hz() {
  static bool blinkState { false };

  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState ? HIGH : LOW);

  ms5611.read();

  log1Hz.print(getTimeString()); log1Hz.print(",");
  log1Hz.print(mag.magnetic.x, IMU_LOGGING_DIGITS); log1Hz.print(",");
  log1Hz.print(mag.magnetic.y, IMU_LOGGING_DIGITS); log1Hz.print(",");
  log1Hz.print(mag.magnetic.z, IMU_LOGGING_DIGITS); log1Hz.print(",");
  log1Hz.print(ms5611.getTemperature(), MS5611_LOGGING_DIGITS); log1Hz.print(",");
  log1Hz.print(ms5611.getPressure(), MS5611_LOGGING_DIGITS);
  for (size_t i { 0 }; i < ANALOG_COUNT; i++) {
    log1Hz.print(",");
    double voltage { analogAverages[i].get() };
    double resistance { voltage_to_resistance(voltage) };
    double temperature { steinhart_hart(resistance, STEINHART_HART_CONSTANTS[i][0], STEINHART_HART_CONSTANTS[i][1], STEINHART_HART_CONSTANTS[i][2]) - KELVIN_CELSIUS_OFFSET };
#ifdef THERMISTOR_CALIBRATION
    log1Hz.print(voltage, ADC_LOGGING_DIGITS);
    log1Hz.print(",");
    log1Hz.print(resistance, ADC_LOGGING_DIGITS);
    log1Hz.print(",");
#endif
    log1Hz.print(temperature, ADC_LOGGING_DIGITS);
  }
  log1Hz.println();

  log1Hz.flush();
  imuLog.flush();
}

void loop() {
  loopImu();
  loopGps();
  if (loopCounter % LOOPS_PER_100HZ == 0) { // Run when 100hz rolls over
    loop100Hz();
  }
  if (loopCounter == 0) { // Run every rollover at 1hz
    loop1Hz();
  }

  loopCounter = (loopCounter + 1) % LOOPS_PER_1HZ; //Set loopcount to roll over at 1hz
  lastLoop += LOOP_PERIOD_MICROSECONDS;
  while (static_cast<int32_t>(micros() - lastLoop) < 0) {
    // Busy loop until it is time for the next cycle
  }
}