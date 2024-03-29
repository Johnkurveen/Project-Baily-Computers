
// For Adafruit Feather M0 (SAMD21)
// Requires https://adafruit.github.io/arduino-board-index/package_adafruit_index.json in Settings > Additional Board Manager URLs

#include <Adafruit_GPS.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_ADS1X15.h>
#include <MS5611.h>
#include <SD.h>
#include <cmath>

#include "Average.hpp"
#include "BufferedFile.hpp"

// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define SDCardSelect 4

using Decimal = float;

static constexpr uint32_t I2C_CLOCK { 400000 }; // Baseline 100kHz
static constexpr uint32_t LOOP_PERIOD_MICROSECONDS { 5000 };
static constexpr uint32_t LOOPS_PER_1HZ { 200 };
static constexpr uint32_t LOOPS_PER_100HZ { 2 };
static constexpr adsGain_t ADC_GAIN { GAIN_ONE };
static constexpr size_t ANALOG_COUNT { 10 };
static constexpr uint32_t LED_PIN { 5 };
static constexpr int ADC_BITS { 12 };
static constexpr Decimal KELVIN_CELSIUS_OFFSET { 273.15 };
static constexpr Decimal ADC_VOLTAGE { 3.3 };
static constexpr Decimal ADC_DIVISOR { static_cast<Decimal>((1 << ADC_BITS) - 1) };
static constexpr Decimal THERMISTOR_RESISTOR_VALUE {100000};
static constexpr Decimal THERMISTOR_HIGH_VOLTAGE {3.3};

// Connect to the GPS on the hardware port
Adafruit_GPS gps { &GPSSerial };
Adafruit_LSM9DS1 lsm {};
MS5611 ms5611 { 0x77 };
Adafruit_ADS1115 zebra {};

Average<Decimal> analogAverages[ANALOG_COUNT];

BufferedFile<4096> logFile;
File diagnosticLog;

uint32_t lastLoop { 0 };

void halt(char const* message, char const* message2) {
  if (diagnosticLog) {
    diagnosticLog.print(message);
    diagnosticLog.println(message2);
    diagnosticLog.close();
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
  Serial.begin(921600);
  Wire.setClock(I2C_CLOCK);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB
  // }

  if (!SD.begin(SDCardSelect)) {
    halt("SD Card initialization failed!");
  }

  char filename[32];
  strcpy(filename, "/LOGS0000");
  for (uint16_t i = 0; i < 10000; i++) {
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

  strcpy(&filename[9], "/LOG.DAT");
  logFile = { filename };
  if( ! logFile ) {
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

  diagnosticLog.println("Initialized Successfully");
  diagnosticLog.close();
  Serial.println("Initialized Successfully");

  lastLoop = micros();
}

void logTime() {
  static bool everFixed { false };
  everFixed |= gps.fix;

  uint32_t offset = static_cast<uint32_t>(gps.secondsSinceTime() * 1000);
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
  uint32_t year = static_cast<uint32_t>(gps.year);
  if (!everFixed)
  {
    milliseconds = 0;
    seconds = 0;
    minutes = 0;
    hour = 0;
    day = 0;
    month = 0;
    year = 0;
  }
  logFile.push<uint8_t>(static_cast<uint8_t>(year));
  logFile.push<uint8_t>(static_cast<uint8_t>(month));
  logFile.push<uint8_t>(static_cast<uint8_t>(day));
  logFile.push<uint8_t>(static_cast<uint8_t>(hour));
  logFile.push<uint8_t>(static_cast<uint8_t>(minutes));
  logFile.push<uint8_t>(static_cast<uint8_t>(seconds));
  logFile.push<uint16_t>(static_cast<uint16_t>(milliseconds));
}

void loopGps() {
  // This takes no time usually
  // If an NMEA is received this takes ~17000us
  static uint32_t lastMessageTime {};
  static constexpr int32_t MILLIS_TO_WAIT { 1000 };
  static bool needsToHandleMessage {false};
  static bool everFixed { false };

  // Read data from the GPS - this is necessary
  (void)gps.read();

  // If we got something from the GPS parse it out
  if (gps.newNMEAreceived()) {
    if (!gps.parse(gps.lastNMEA())) {
      return;
    }
    needsToHandleMessage = true;
    lastMessageTime = millis();
  }

  if (needsToHandleMessage && (static_cast<int32_t>(millis() - lastMessageTime) > MILLIS_TO_WAIT)) {
    needsToHandleMessage = false;

    if (!everFixed && gps.fix) {
      everFixed = true;
      Serial.println("GPS Fix");
    }

    logFile.push<uint8_t>(0x10); // GPS Data
    logFile.push<uint8_t>(gps.hour);
    logFile.push<uint8_t>(gps.minute);
    logFile.push<uint8_t>(gps.seconds);
    logFile.push<int32_t>(gps.latitude_fixed);
    logFile.push<int32_t>(gps.longitude_fixed);
    logFile.push<float>(gps.altitude);
    logFile.push<float>(gps.speed);
    logFile.push<uint8_t>(gps.fixquality);
    logFile.push<uint8_t>(gps.satellites);

    logFile.flush();
  } else {
    logFile.doWrite();
  }
}

void loopImu() {
  // This takes ~2000us
  sensors_event_t accel {};
  sensors_event_t gyro {};
  lsm.getAccel().getEvent(&accel);
  lsm.getGyro().getEvent(&gyro);

  logFile.push<uint8_t>(0x20); // IMU Data
  logTime();
  logFile.push<float>(accel.acceleration.x);
  logFile.push<float>(accel.acceleration.y);
  logFile.push<float>(accel.acceleration.z);
  logFile.push<float>(gyro.gyro.x);
  logFile.push<float>(gyro.gyro.y);
  logFile.push<float>(gyro.gyro.z);
}

Decimal readAnalogPin(uint32_t pin) {
  return static_cast<Decimal>(analogRead(pin)) / ADC_DIVISOR * ADC_VOLTAGE;
}

void prepareZebraChannel(uint8_t channel) {
  zebra.startADCReading(MUX_BY_CHANNEL[channel], false);
}

Decimal readZebraChannel(uint8_t channel) {
  (void)channel;
  while (!zebra.conversionComplete()) {
    // Intentionally left blank
  }
  return static_cast<Decimal>(zebra.computeVolts(zebra.getLastConversionResults()));
}

Decimal readAnalog(size_t channel) {
  Decimal result { NAN };
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
      prepareZebraChannel(3);
      return readAnalogPin(A0);
    case 6:
      result = readZebraChannel(3);
      prepareZebraChannel(2);
      break;
    case 7:
      result = readZebraChannel(2);
      prepareZebraChannel(1);
      break;
    case 8:
      result = readZebraChannel(1);
      prepareZebraChannel(0);
      break;
    case 9:
      return readZebraChannel(0);
    default:
      return NAN;
  }
  return result;
}

Decimal steinhart_hart(Decimal resistance, Decimal a, Decimal b, Decimal c) {
  // return resistance;
  Decimal lnResistance { log(resistance) };
  return Decimal {1} / (a + b * lnResistance + c * lnResistance * lnResistance * lnResistance);
}

Decimal voltage_to_resistance(Decimal voltage) {
  // return voltage;
  return THERMISTOR_RESISTOR_VALUE * (THERMISTOR_HIGH_VOLTAGE - voltage) / voltage;
}

void loop100Hz() {
  static size_t loopCounter { 0 };

  // ~2100us worst case
  analogAverages[loopCounter].insert(readAnalog(loopCounter));

  loopCounter = (loopCounter + 1) % 10;
}

void loop196() {
  // ~2900us
  (void)ms5611.read();
}

void loop198() {
  static bool blinkState { false };
  sensors_event_t mag {};

  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState ? HIGH : LOW);

  // ~925us
  lsm.getMag().getEvent(&mag);

  // ~500us
  logFile.push<uint8_t>(0x30); // 1Hz Data
  logTime();
  logFile.push<float>(mag.magnetic.x);
  logFile.push<float>(mag.magnetic.y);
  logFile.push<float>(mag.magnetic.z);
  logFile.push<float>(ms5611.getTemperature());
  logFile.push<float>(ms5611.getPressure());
  logFile.push<float>(analogAverages[0].get());
  logFile.push<float>(analogAverages[1].get());
  logFile.push<float>(analogAverages[2].get());
  logFile.push<float>(analogAverages[3].get());
  logFile.push<float>(analogAverages[4].get());
  logFile.push<float>(analogAverages[5].get());
  logFile.push<float>(analogAverages[6].get());
  logFile.push<float>(analogAverages[7].get());
  logFile.push<float>(analogAverages[8].get());
  logFile.push<float>(analogAverages[9].get());
}

void loop() {
  static uint32_t loopCounter { 0 };

  // We have 5000us to work with

  loopImu(); // This takes ~2000us

  if ((loopCounter % LOOPS_PER_100HZ) == (LOOPS_PER_100HZ - 1)) {
    // This runs every odd loop and takes 2000us
    loop100Hz();
  } else {
    // No more execution may happen on an odd loop
    if (loopCounter == 196) {
      loop196(); // This takes ~3000us - no more execution may happen (~5000 total)
    } else if (loopCounter == 198) {
      loop198(); // This takes ~1500us - no more execution may happen (~3500 total)
    } else {
      loopGps(); // This is incredibly variable - only do it on cycles when we don't need all the time
    }
  }

  loopCounter = (loopCounter + 1) % LOOPS_PER_1HZ;
  lastLoop += LOOP_PERIOD_MICROSECONDS; // Increment this so that we'll try and catch up if we've fallen behind
  while (static_cast<int32_t>(micros() - lastLoop) < 0) {
    // Busy loop until it is time for the next cycle
  }
}