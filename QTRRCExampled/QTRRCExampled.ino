/**
 * @file main.ino
 * @brief Example code for using QTRSensors with Arduino to detect line position.
 * @author Briyith Guacas (briyithguacas@unicauca.edu.co)
 *         Karol Palechor (karolpalechor@unicauca.edu.co)
 *         Johana Puerres (johanapuerres@unicauca.edu.co)
 * @date [12-05-2024]
 * 
 */

#include <QTRSensors.h>

QTRSensors qtr; /**< Object for handling QTR sensors */

const uint8_t SensorCount = 8; /**< Number of sensors */
uint16_t sensorValues[SensorCount]; /**< Array to store sensor values */

/**
 * @brief Setup function for initializing the system.
 */
void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){14, 27, 26, 25, 33, 32, 19, 21}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // Calibrate sensors
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate calibration is done

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

/**
 * @brief Loop function for continuous operation.
 */
void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, followed by the line position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}
