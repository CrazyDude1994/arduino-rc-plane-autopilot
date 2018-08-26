#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include <L3G.h>
#include <math.h>
#include "TimerOne.h"

#define SampleRate 10000

ADXL345 accel;
L3G gyro;
unsigned long pT;

int16_t ax, ay, az;
double roll, pitch, accelRoll, accelPitch;
double gX, gY, gZ = 0;
double offsetRoll, offsetPitch = 0;

bool readGyro;
int lastWriteMillis;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(38400);
  Serial.println("Initializing I2C devices...");
  accel.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  enableBt();
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
  }
  gyro.enableDefault();

  readGyro = false;
  lastWriteMillis = millis();
  pT = 0;
  Timer1.initialize(SampleRate);
  Timer1.attachInterrupt( timerIsr );
}

void timerIsr()
{
  readGyro = true;
}

void loop() {
  btLoop();

  if (readGyro && gyro.init()) {
    String command = String();
    while (Serial.available()) {
      command = command + (char) Serial.read();
    }

    readGyro = false;
    unsigned long cT = micros();
    gyro.read();
    unsigned long dT = cT - pT;
    pT = cT;

    accel.getAcceleration(&ax, &ay, &az);
    accelRoll = (atan2(az, ax) * 4068) / 71 + 90;
    accelPitch = (atan2(az, ay) * 4068) / 71 + 90;

    roll = 0.95 * (roll + 0.00875 * gyro.g.x * (dT / 1000000.0)) + 0.05 * accelRoll;
    pitch = 0.95 * (pitch + 0.00875 * gyro.g.y * (dT / 1000000.0)) + 0.05 * accelPitch;

    if (millis() - lastWriteMillis > 66) {
      Serial.print(roll - offsetRoll);
      Serial.print(" ");
      Serial.println(pitch - offsetPitch);
      lastWriteMillis = millis();
    }
  }
}
