#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

const int ppgPin = A0;
const int gsrPin = A1;

void setup() {
  Serial.begin(9600);

  if (tsl.begin()) {
    Serial.println("Sensor PPG ditemukan!");
  } else {
    Serial.println("Tidak dapat menemukan sensor PPG");
    while (1);
  }
}

void loop() {
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t red, ir;
  tsl.getLuminosity(&red, &ir);
  Serial.print("Data HRV (PPG): ");
  Serial.println(ir);

  int ppgValue = analogRead(ppgPin);
  int heartRate = map(ppgValue, 0, 1023, 40, 200);
  Serial.print("Heart Rate (HR): ");
  Serial.print(heartRate);
  Serial.println(" bpm");

  int edaValue = analogRead(gsrPin);
  Serial.print("Data EDA (GSR): ");
  Serial.println(edaValue);

  delay(1000);
}
