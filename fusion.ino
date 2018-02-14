#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include <L3G.h>
#include <math.h>
#include "TimerOne.h"

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
#define SampleRate 10000

ADXL345 accel;
L3G gyro;
unsigned long pT;

int16_t ax, ay, az;
double roll, pitch;
double gX, gY, gZ = 0;

bool readGyro;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(38400);
  Serial.println("Initializing I2C devices...");
  accel.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();

  readGyro = false;
  pT = 0;
  Timer1.initialize(SampleRate);
  Timer1.attachInterrupt( timerIsr );
}

void timerIsr()
{
  readGyro = true;
}

void loop() {
  //  Serial.print("Pitch: ");
  //  Serial.println(pitch);

  if (readGyro) {
    readGyro = false;
    unsigned long cT = micros();
    gyro.read();
    unsigned long dT = cT - pT;
    pT = cT;

    accel.getAcceleration(&ax, &ay, &az);
    roll = (atan2(az, ax) * 4068) / 71 + 90;
    pitch = (atan2(az, ay) * 4068) / 71 + 90;
    Serial.print(roll);
    Serial.print(" ");

    gX = gX + 0.00875 * gyro.g.x * (dT / 1000000.0);
    gY = gY + 0.00875 * gyro.g.y * (dT / 1000000.0);
    gZ = gZ + 0.00875 * gyro.g.z * (dT / 1000000.0);

    roll = (gY * 0.70) + (roll * 0.3);
    
    Serial.print(gY);
    Serial.print(" ");
    Serial.println(roll);
  }
  //  Serial.print("G ");
  //  Serial.print("X: ");
  //  Serial.print((int)gyro.g.x);
  //  Serial.print(" Y: ");
  //  Serial.print((int)gyro.g.y);
  //  Serial.print(" Z: ");
  //  Serial.println((int)gyro.g.z);
}
