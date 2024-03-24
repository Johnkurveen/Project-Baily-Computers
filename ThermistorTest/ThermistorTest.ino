
#include <math.h>

static constexpr int ANALOG_PIN { 0 };

static double ADC_VOLTAGE { 5.0 };
static double ADC_MAX_VALUE { 1023.0 };
static double INPUT_VOLTAGE { 5.0 };
static double RESISTOR_VALUE { 100000 };

static double A { 1.122919524e-3 };
static double B { 2.357963107e-4 };
static double C { 0.7559000491e-7 };

void setup() {
    Serial.begin(9600);
    while (!Serial) {
      // Loop intentionally left blank
    }
}

double steinhart_hart(double resistance, double a, double b, double c) {
  double lnResistance { log(resistance) };
  return 1.0 / (a + b * lnResistance + c * lnResistance * lnResistance * lnResistance);
}

void loop() {
  double measuredVoltage { static_cast<double>(analogRead(ANALOG_PIN))/ADC_MAX_VALUE * ADC_VOLTAGE };

  // Input Voltage ~<X Resistance>~ measuredVoltage ~<RESISTOR_VALUE>~ 0.0V

  // According to the internet:
  // measuredVoltage = Input Voltage * (RESISTOR_VALUE) / (X + RESISTOR_VALUE)

  // Therefore:
  // measuredVoltage * (X + RESISTOR_VALUE) = Input Voltage * (RESISTOR_VALUE)
  // measuredVoltage * X + measuredVoltage * RESISTOR_VALUE = Input Voltage * RESISTOR_VALUE
  // measuredVoltage * X = Input Voltage * RESISTOR_VALUE - measuredVoltage * RESISTOR_VALUE
  // measuredVoltage * X = RESISTOR_VALUE * (Input Voltage - measuredVoltage)
  // X = RESISTOR_VALUE * (Input Voltage - measuredVoltage) / measuredVoltage

  double resistance = RESISTOR_VALUE * (INPUT_VOLTAGE - measuredVoltage) / measuredVoltage;
  double temperature = steinhart_hart(resistance, A, B, C);

  Serial.print(measuredVoltage);
  Serial.print(",");
  Serial.print(resistance);
  Serial.print(",");
  Serial.print(temperature);
  Serial.println();

  delay(100);
}
