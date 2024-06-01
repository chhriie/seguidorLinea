/**
 * @file main.ino
 * @brief Control de un robot seguidor de línea utilizando sensores QTR.
 * @author Briyith Guacas (briyithguacas@unicauca.edu.co)
 *         Karol Palechor (karolpalechor@unicauca.edu.co)
 *         Johana Puerres (johanapuerres@unicauca.edu.co)
 * @date [23-05-2024]
 * 
 * Documentación con Doxygen para el código de control de un robot seguidor de línea.
 */
#include <QTRSensors.h>

// Constantes de control PID y velocidad máxima
#define Kp 0.6 /**< Coeficiente proporcional */
#define Ki 0.01 /**< Coeficiente integral */
#define Kd 0.2 /**< Coeficiente derivativo */
#define MaxSpeed 100 /**< Velocidad máxima */
#define BaseSpeed 70 /**< Velocidad base */
#define speedturn 50 /**< Velocidad de giro */
#define rightMotor1 A1 /**< Pin de control del motor derecho 1 */
#define rightMotor2 A2 /**< Pin de control del motor derecho 2 */
#define rightMotorPWM 10 /**< Pin PWM del motor derecho */
#define leftMotor1 A4 /**< Pin de control del motor izquierdo 1 */
#define leftMotor2 A5 /**< Pin de control del motor izquierdo 2 */
#define leftMotorPWM 11 /**< Pin PWM del motor izquierdo */

QTRSensors qtr; /**< Objeto para manejar los sensores QTR */
const uint8_t SensorCount = 8; /**< Cantidad de sensores */
uint16_t sensorValues[SensorCount]; /**< Array para almacenar los valores de los sensores */

/**
 * @brief Inicialización del sistema.
 */
void setup() {
    // Configuración de sensores QTR
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){36, 38, 40, 42, 44, 46, 48, 50}, SensorCount);
    qtr.setEmitterPin(2);

    // Configuración de pines de motores
    pinMode(rightMotor1, OUTPUT);
    pinMode(rightMotor2, OUTPUT);
    pinMode(rightMotorPWM, OUTPUT);
    pinMode(leftMotor1, OUTPUT);
    pinMode(leftMotor2, OUTPUT);
    pinMode(leftMotorPWM, OUTPUT);

    delay(2000);
    Serial.begin(115200);
    Serial.println();

    // Calibración de los sensores
    for (int i = 0; i < 100; i++) {
        if (i < 25 || i >= 75) {
            move(1, 70, 1);
            move(0, 70, 0);
        } else {
            move(1, 70, 0);
            move(0, 70, 1);
        }
        qtr.calibrate();
        delay(20);
    }

    wait();
    delay(2000);
}

int lastError = 0; /**< Último error de seguimiento */
uint16_t position = qtr.readLineBlack(sensorValues); /**< Posición actual del robot */

/**
 * @brief Bucle principal de control.
 */
void loop() {
    position = qtr.readLineBlack(sensorValues);

    if (position > 6500) {
        move(1, speedturn, 1);
        move(0, speedturn, 0);
        return;
    }

    if (position < 500) {
        move(1, speedturn, 0);
        move(0, speedturn, 1);
        return;
    }

    int error = position - 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;

    if (rightMotorSpeed > MaxSpeed) {
        rightMotorSpeed = MaxSpeed;
    }
    if (leftMotorSpeed > MaxSpeed) {
        leftMotorSpeed = MaxSpeed;
    }
    if (rightMotorSpeed < 0) {
        rightMotorSpeed = 0;
    }
    if (leftMotorSpeed < 0) {
        leftMotorSpeed = 0;
    }

    move(1, rightMotorSpeed, 1);
    move(0, leftMotorSpeed, 1);
}

/**
 * @brief Espera deteniendo los motores.
 */
void wait() {
    analogWrite(leftMotorPWM, 0);
    analogWrite(rightMotorPWM, 0);
}

/**
 * @brief Control de movimiento de los motores.
 * @param motor Motor a controlar (0 para izquierdo, 1 para derecho).
 * @param speed Velocidad del motor.
 * @param direction Dirección del movimiento (0 para atrás, 1 para adelante).
 */
void move(int motor, int speed, int direction) {
    boolean inPin1;
    boolean inPin2;

    if (direction == 1) {
        inPin1 = HIGH;
        inPin2 = LOW;
    }
    if (direction == 0) {
        inPin1 = LOW;
        inPin2 = HIGH;
    }

    if (motor == 0) {
        digitalWrite(leftMotor1, inPin1);
        digitalWrite(leftMotor2, inPin2);
        analogWrite(leftMotorPWM, speed);
    }

    if (motor == 1) {
        digitalWrite(rightMotor1, inPin1);
        digitalWrite(rightMotor2, inPin2);
        analogWrite(rightMotorPWM, speed);
    }
}

/**
 * @brief Calibra los sensores QTR.
 */
void sensor_calibrate() {
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
    }

    digitalWrite(LED_BUILTIN, LOW);
    //Serial.begin(9600);

    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
    }

    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
    }

    Serial.println();
}
