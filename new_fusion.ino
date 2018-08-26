#include <Wire.h>
#include <L3G.h>
#include "I2Cdev.h"
#include "ADXL345.h"
#include <SFE_BMP180.h>
#include <HMC5883L.h>
#include <MSP.h>

MSP msp;
SFE_BMP180 pressure;
HMC5883L compass;
L3G gyro;
ADXL345 accel;

int16_t accel_x, accel_y, accel_z = 0;
double accel_pitch, accel_roll = 0;
double accel_pitch_last, accel_roll_last = 0;
double baseline;
double altitude;

bool initialized = false;

struct Angle {
  double pitch, roll, yaw;
};

long dtime = 0;
long lastTime = millis();

Angle angle;

msp_attitude_t attitude;
msp_altitude_t alt;
msp_status_t current_status;

void setup() {
  Serial1.begin(115200);
  msp.begin(Serial1);
  Serial.begin(38400);
  Wire.begin();

  compass = HMC5883L(); // Construct a new HMC5883 compass.

  if (!pressure.begin()) {
    Serial.println("Failed to init pressure");
    while (1);
  }

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  accel.initialize();
  baseline = getPressure();
  compass.SetScale(1.3); // Set the scale of the compass.compass.SetScale(1.3); // Set the scale of the compass.
  compass.SetMeasurementMode(Measurement_Continuous);
}

void loop() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  gyro.read();
  accel.getAcceleration(&accel_x, &accel_y, &accel_z);

  dtime = lastTime - millis();
  lastTime = millis();

  double accel_roll_current = atan2(accel_z, accel_x) * 180 / M_PI;
  double accel_pitch_current = atan2(accel_z, accel_y) * 180 / M_PI;

  double heading = atan2(scaled.ZAxis, scaled.XAxis) * 180 / M_PI;

  angle.pitch = angle.pitch + (gyro.g.x * 0.00875) * dtime / 1000;
  angle.roll = angle.roll - (gyro.g.y * 0.00875) * dtime / 1000;
  angle.yaw = angle.yaw - (gyro.g.z * 0.00875) * dtime / 1000;

  accel_pitch_current = fmod(accel_pitch_current + 360, 360);
  accel_roll_current = fmod(accel_roll_current + 360, 360);

  accel_pitch += accel_pitch_current - accel_pitch_last;
  accel_roll += accel_roll_current - accel_roll_last;

  if (abs(accel_pitch - angle.pitch) > 180) {
    accel_pitch = angle.pitch;
  }

  if (abs(accel_roll - angle.roll) > 180) {
    Serial.println("Gimbal lock detected");
    accel_roll = angle.roll;
  }

  accel_pitch_last = accel_pitch_current;
  accel_roll_last = accel_roll_current;

  if (!initialized) {
    initialized = true;
    angle.pitch = accel_pitch;
    angle.roll = accel_roll;
  }

  double magnitude = sqrt(pow(accel_x * 0.0039, 2) + pow(accel_y * 0.0039, 2) + pow(accel_z * 0.0039, 2));

  if (magnitude < 1.1 && magnitude > 0.9) {
    angle.pitch = (angle.pitch * 0.98) + (accel_pitch * 0.02);
    angle.roll = (angle.roll * 0.98) + (accel_roll * 0.02);
  }

  double current_pressure = getPressure();
  altitude = pressure.altitude(current_pressure, baseline);

  Serial.print(angle.roll);
  Serial.print(" ");
  Serial.print(angle.pitch);
  Serial.print(" ");
  Serial.print(angle.yaw);
  Serial.print(" ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.println(altitude);

  attitude.roll = angle.roll * 10;
  attitude.pitch = angle.pitch * 10;
  attitude.yaw = angle.yaw;
  alt.estimatedActualPosition = altitude * 10;
  current_status.sensor = MSP_STATUS_SENSOR_BARO | MSP_STATUS_SENSOR_ACC | MSP_STATUS_SENSOR_BARO | MSP_STATUS_SENSOR_GPS;
  msp.send(MSP_ATTITUDE, &attitude, sizeof(attitude));
  msp.send(MSP_ALTITUDE, &alt, sizeof(alt));
  msp.send(MSP_STATUS, &current_status, sizeof(current_status));

  delay(50);
}

double getPressure()
{
  char status;
  double T, P, p0, a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
