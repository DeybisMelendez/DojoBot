#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// Variables compartidas
volatile int joy1Y = 0;
volatile int joy2Y = 0;
volatile bool isTackling = false;
volatile int valorPWM;

// Parámetros PWM del ESC
const int freq = 50;
const int ledChannel = 0;
const int resolution = 12;
const int PULSE_MIN = 1000;
const int PULSE_VELOCIDAD = 1300;
const int PULSE_MAX = 2000;

// Conversión de microsegundos a duty cycle
int microSegundosADutyCycle(int microsegundos)
{
  return (int)((float)microsegundos / 20000.0 * (1 << resolution));
}

// Estructura de datos ESP-NOW
typedef struct
{
  bool btn1;
  bool btn2;
  bool btn3;
  bool btn4;
  int joy1;
  int joy2;
} ControlData;

ControlData incomingData;
Servo servoMotor;

// Pines
const int SERVO_PIN = 25;
#define ESC_PIN 13
#define DIR_L_PIN 19
#define STEP_L_PIN 18
#define DIR_R_PIN 23
#define STEP_R_PIN 22
#define ENABLE_STEPPER_L 21
#define ENABLE_STEPPER_R 5

// Control del motor brushless (ESC)
void setBrushlessSpeed(int microseconds)
{
  microseconds = constrain(microseconds, PULSE_MIN, PULSE_MAX);
  valorPWM = microSegundosADutyCycle(microseconds);
  ledcWrite(ledChannel, valorPWM);
}

// Movimiento del servo (pala)
void moveServo()
{
  servoMotor.write(180);
  delay(500);
  servoMotor.write(0);
  delay(500);
}

// Embestida (placaje)
void tackle()
{
  isTackling = true;

  digitalWrite(DIR_L_PIN, HIGH);
  digitalWrite(DIR_R_PIN, HIGH);

  // Movimiento hacia adelante durante la embestida
  for (int i = 0; i < 300; i++)
  {
    digitalWrite(STEP_L_PIN, HIGH);
    digitalWrite(STEP_R_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_L_PIN, LOW);
    digitalWrite(STEP_R_PIN, LOW);
    delayMicroseconds(500);
  }

  moveServo();
  isTackling = false;
}

// Callback de recepción ESP-NOW
void onDataReceive(const uint8_t *mac, const uint8_t *incomingDataPtr, int len)
{
  if (len == sizeof(ControlData))
  {
    memcpy((void *)&incomingData, incomingDataPtr, sizeof(incomingData));

    joy1Y = incomingData.joy1;
    joy2Y = incomingData.joy2;

    // Dirección de motores
    digitalWrite(DIR_L_PIN, (joy1Y > 0) ? HIGH : LOW);
    digitalWrite(DIR_R_PIN, (joy2Y > 0) ? HIGH : LOW);

    // Control del brushless
    if (incomingData.btn1)
      setBrushlessSpeed(PULSE_VELOCIDAD);

    if (incomingData.btn2)
      setBrushlessSpeed(PULSE_MIN);

    // Acciones
    if (incomingData.btn3)
      tackle();

    if (incomingData.btn4)
      moveServo();

    /*Serial.printf("Data received -> Buttons: %d %d %d %d | Joysticks: %d | %d\n",
                  incomingData.btn1, incomingData.btn2, incomingData.btn3,
                  incomingData.btn4, joy1Y, joy2Y);*/
  }
}

// Setup
void setup()
{
  Serial.begin(115200);
  // Serial.println("DojoBot iniciado");

  // Configurar PWM del ESC
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ESC_PIN, ledChannel);

  // Calibración del ESC
  setBrushlessSpeed(PULSE_MAX);
  delay(3000);
  setBrushlessSpeed(PULSE_MIN);
  delay(1000);

  // Servo
  servoMotor.attach(SERVO_PIN);

  // Pines motores
  pinMode(DIR_L_PIN, OUTPUT);
  pinMode(STEP_L_PIN, OUTPUT);
  pinMode(DIR_R_PIN, OUTPUT);
  pinMode(STEP_R_PIN, OUTPUT);
  pinMode(ENABLE_STEPPER_L, OUTPUT);
  pinMode(ENABLE_STEPPER_R, OUTPUT);

  // 🔥 Mantener steppers siempre activos
  digitalWrite(ENABLE_STEPPER_L, LOW);
  digitalWrite(ENABLE_STEPPER_R, LOW);

  // Inicializar WiFi y ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    // Serial.println("❌ Error inicializando ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataReceive);
  // Serial.println("✅ ESP-NOW listo, esperando controlador...");
}

// Loop principal
void loop()
{
  if (!isTackling)
  {
    int joy1 = joy1Y;
    int joy2 = joy2Y;

    // Si no hay entrada en ninguno, no mover
    if (joy1 == 0 && joy2 == 0)
      return;

    // Mapeo de velocidad → delay
    int delayL = map(abs(joy1), 0, 100, 1000, 300);
    int delayR = map(abs(joy2), 0, 100, 1000, 300);

    // Motor izquierdo
    if (joy1 != 0)
    {
      digitalWrite(STEP_L_PIN, HIGH);
      delayMicroseconds(delayL);
      digitalWrite(STEP_L_PIN, LOW);
      delayMicroseconds(delayL);
    }

    // Motor derecho
    if (joy2 != 0)
    {
      digitalWrite(STEP_R_PIN, HIGH);
      delayMicroseconds(delayR);
      digitalWrite(STEP_R_PIN, LOW);
      delayMicroseconds(delayR);
    }
  }
}
