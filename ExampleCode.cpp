#include <ArduinoBLE.h>
#include <RunningAverage.h>
#include <Wire.h>
#include <math.h>

RunningAverage edaRunningAverage(10);  // Running Average for EDA/GSR values

BLEService biometricsService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE Biometrics Service

BLEIntCharacteristic HRCharacteristic("00002A37-0000-1000-8000-00805F9B34FB", BLERead | BLENotify);
BLEIntCharacteristic SDNNCharacteristic("00002A38-0000-1000-8000-00805F9B34FB", BLERead | BLENotify);
BLEIntCharacteristic RMSSDCharacteristic("00002A39-0000-1000-8000-00805F9B34FB", BLERead | BLENotify);
BLEIntCharacteristic MeanCharacteristic("00002A40-0000-1000-8000-00805F9B34FB", BLERead | BLENotify);
BLEIntCharacteristic StdDevCharacteristic("00002A41-0000-1000-8000-00805F9B34FB", BLERead | BLENotify);
BLEIntCharacteristic HRVCharacteristic("00002A42-0000-1000-8000-00805F9B34FB", BLERead | BLENotify);

const int REPORTING_PERIOD_MS = 1000;
const int HRV_CALCULATION_PERIOD_MS = 20000;  // Calculate HRV every minute
uint32_t tsLastReport = 0;
uint32_t tsLastHRVCalculation = 0;
uint8_t c;
int numRRIntervals = 0;
long sumRRIntervals = 0;
long sumRRIntervalsSquared = 0;
bool hrvCalculationStarted = false;
int sensorValue;
const int GSR = A0;      // Define the GSR pin
const int TOUCH_SENSOR = A1;  // Define the touch sensor pin
const unsigned long MODE_SWITCH_INTERVAL = 300000;  // 5 minutes in milliseconds
bool highMode = true;
unsigned long lastTouchTime = 0;
bool touchSensorPressed = false;

void setup() {
  Serial.begin(9600);
  pinMode(GSR, INPUT);
  pinMode(TOUCH_SENSOR, INPUT_PULLUP);  // Set the touch sensor pin as INPUT_PULLUP
  delay(1000);

  Wire.begin();

  if (!BLE.begin()) {
    Serial.println("Starting Bluetooth® Low Energy failed!");
  }

  BLE.setLocalName("Starstec");
  BLE.setAdvertisedService(biometricsService);

  biometricsService.addCharacteristic(HRCharacteristic);
  biometricsService.addCharacteristic(SDNNCharacteristic);
  biometricsService.addCharacteristic(RMSSDCharacteristic);
  biometricsService.addCharacteristic(MeanCharacteristic);
  biometricsService.addCharacteristic(StdDevCharacteristic);
  biometricsService.addCharacteristic(HRVCharacteristic);

  BLE.addService(biometricsService);

  BLE.advertise();
  Serial.println("BLE Biometrics Peripheral, waiting for connections....");
}

void loop() {
  BLEDevice central = BLE.central();

  // Check if the touch sensor is touched
  int touchValue = digitalRead(TOUCH_SENSOR);
  if (touchValue == HIGH) {
    lastTouchTime = millis();
    if (!touchSensorPressed) {
      touchSensorPressed = true;
      highMode = !highMode; // Switch mode
      if (highMode) {
        Serial.println("Switched to High Mode");
      } else {
        Serial.println("Switched to Low Mode");
      }
    }
  } else {
    touchSensorPressed = false;
  }

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Wire.requestFrom(0xA0 >> 1, 1);
        while (Wire.available()) {
          c = Wire.read();
        }
        Serial.print("Heart Rate: ");
        Serial.println(c);

        if (c > 40) {
          hrvCalculationStarted = true;
        }

        if (hrvCalculationStarted && c > 60) {
          long rrInterval = 60000 / c;
          numRRIntervals++;
          sumRRIntervals += rrInterval;
          sumRRIntervalsSquared += rrInterval * rrInterval;
        }

        tsLastReport = millis();
      }

      if (hrvCalculationStarted && millis() - tsLastHRVCalculation > HRV_CALCULATION_PERIOD_MS && numRRIntervals > 0) {
        double meanRRInterval = (double)sumRRIntervals / numRRIntervals;
        double variance = (double)sumRRIntervalsSquared / numRRIntervals - meanRRInterval * meanRRInterval;
        double hrv = sqrt(variance);

        double rmssd = sqrt((double)sumRRIntervalsSquared / (numRRIntervals - 1));

        HRCharacteristic.writeValue(c);
        SDNNCharacteristic.writeValue(int(hrv));
        RMSSDCharacteristic.writeValue(int(rmssd));
        MeanCharacteristic.writeValue(int(meanRRInterval));
        StdDevCharacteristic.writeValue(int(sqrt(variance)));
        HRVCharacteristic.writeValue(int(hrv));

        numRRIntervals = 0;
        sumRRIntervals = 0;
        sumRRIntervalsSquared = 0;
        tsLastHRVCalculation = millis();    
      }

      sensorValue = analogRead(GSR);
      edaRunningAverage.addValue(sensorValue);

      double edaMean = edaRunningAverage.getAverage();
      double edaStdDev = calculateStandardDeviation(edaRunningAverage);

      // Send EDA Mean and Std Dev to Serial Monitor
      Serial.print("EDA Mean: ");
      Serial.println(edaMean);
      Serial.print("EDA Std Dev: ");
      Serial.println(edaStdDev);

      delay(1000);
    }

    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

double calculateStandardDeviation(RunningAverage& ra) {
  double mean = ra.getAverage();
  double varianceSum = 0.0;
  for (int i = 0; i < ra.getSize(); i++) {
    double diff = ra.getElement(i) - mean;
    varianceSum += diff * diff;
  }
  double variance = varianceSum / ra.getSize();
  return sqrt(variance);
}
