/**
 * @file main.ino
 * @brief Control de motores utilizando el driver L298N.
 * @author Briyith Guacas (briyithguacas@unicauca.edu.co)
 *         Karol Palechor (karolpalechor@unicauca.edu.co)
 *         Johana Puerres (johanapuerres@unicauca.edu.co)
 * @date [17-05-2024]
 * 
 * Documentación con Doxygen para el control de motores utilizando el driver L298N.
 */

// *** DECLARAMOS LAS VARIABLES QUE VAMOS A UTILIZAR ***

// Variables que indican los pines para el motor A, por comodidad a la hora de conectar, le vamos a poner los mismos nombres que tiene la placa del L298N
int IN1 = 5; /**< Pin IN1 para el motor A */
int IN2 = 4; /**< Pin IN2 para el motor A */

// Variables que indican los pines para el motor B, por comodidad a la hora de conectar, le vamos a poner los mismos nombres que tiene la placa del L298N
int IN3 = 3; /**< Pin IN3 para el motor B */
int IN4 = 2; /**< Pin IN4 para el motor B */

/**
 * @brief Inicialización del sistema.
 */
void setup() {
    // Los tres pines que controlan el motor A
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Los tres pines que controlan el motor B
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

/**
 * @brief Bucle principal de control.
 */
void loop() {
    Adelante();
    delay(2000);
    Parar();
    delay(2000);
    Atras();
    delay(2000);
    Parar();
    delay(2000);
    izquierda();
    delay(2000);
    Parar();
    derecha();
    delay(2000);
    Parar();
}

/**
 * @brief Función para mover hacia adelante.
 */
void Adelante() {
    // Para controlar el motor A
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    // Para controlar el motor B
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

/**
 * @brief Función para mover hacia atrás.
 */
void Atras() {
    // Para controlar el motor A
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    // Para controlar el motor B
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

/**
 * @brief Función para detener ambos motores.
 */
void Parar() {
    // Dirección motor A
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    // Dirección motor B
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

/**
 * @brief Función para girar hacia la izquierda.
 */
void izquierda() {
    // Dirección motor A
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    // Dirección motor B
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

/**
 * @brief Función para girar hacia la derecha.
 */
void derecha() {
    // Dirección motor A
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    // Dirección motor B
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
