
// For Adafruit Feather M0 (SAMD21)
// Requires https://adafruit.github.io/arduino-board-index/package_adafruit_index.json in Settings > Additional Board Manager URLs

#include <Adafruit_GPS.h>
#include <SD.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define SDCardSelect 4

// Connect to the GPS on the hardware port
Adafruit_GPS gps(&GPSSerial);
File logfile;

uint32_t lastParsedMessage = 0;
bool newMessage { false };

static constexpr uint32_t messageDelay { 250 };

void error() {
  while (true) {
    Serial.println("Error");
    delay(1000);
  }
}

void setup()
{
  Serial.begin(9600);

  if (!SD.begin(SDCardSelect)) {
    Serial.println("Card init. failed!");
    error();
  }
  char filename[15];
  strcpy(filename, "/GPS0000.CSV");
  for (uint8_t i = 0; i < 10000; i++) {
    filename[4] = '0' + i/1000;
    filename[5] = '0' + (i/100)%10;
    filename[6] = '0' + (i/10)%10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error();
  }

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  gps.begin(9600);

  // Get all the data 5 times per second
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);

  delay(1000);

  logfile.print("hour,");
  logfile.print("minute,");
  logfile.print("seconds,");
  logfile.print("latitude_fixed,");
  logfile.print("longitude_fixed,");
  logfile.print("altitude,");
  logfile.print("speed,");
  logfile.print("fixquality");
  logfile.println();
}

void loop()
{
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

  if ((millis() - lastParsedMessage > messageDelay) && newMessage){
    newMessage = false;

    logfile.print(gps.hour);logfile.print(",");
    logfile.print(gps.minute);logfile.print(",");
    logfile.print(gps.seconds);logfile.print(",");
    logfile.print(gps.latitude_fixed);logfile.print(",");
    logfile.print(gps.longitude_fixed);logfile.print(",");
    logfile.print(gps.altitude, 2);logfile.print(",");
    logfile.print(gps.speed, 2);logfile.print(",");
    logfile.print(gps.fixquality);logfile.println();
    logfile.flush();
  }
}