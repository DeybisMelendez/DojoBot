#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
// #include <AccelStepper.h>

int joy1Y = 0;
int joy2Y = 0;
bool isTackling = false;

const int freq = 50;
const int ledChannel = 0;
const int resolution = 12;

// Pulso en microsegundos (μs)
const int PULSE_MIN = 1000;       // Apagado/Mínimo
const int PULSE_VELOCIDAD = 1300; // Velocidad de prueba
const int PULSE_MAX = 2000;       // Máximo (para Calibración)

int microSegundosADutyCycle(int microsegundos)
{
  return (int)((float)microsegundos / 20000.0 * (1 << resolution));
}

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

// Estado actual del motor (inicialmente apagado)
int valorPWM = microSegundosADutyCycle(PULSE_MIN);

const int SERVO_PIN = 27;
#define ESC_PIN 13
#define DIR_L_PIN 19
#define STEP_L_PIN 18
#define DIR_R_PIN 5
#define STEP_R_PIN 17
#define ENABLE_STEPPERS 16

const int STEPS_PER_REV = 200;

void moveServo()
{
  servoMotor.write(180);
  delay(500);
  servoMotor.write(0);
  delay(500);
}

void tackle()
{
  isTackling = true;
  digitalWrite(DIR_L_PIN, HIGH);
  digitalWrite(DIR_R_PIN, HIGH);
  digitalWrite(STEP_L_PIN, HIGH);
  digitalWrite(STEP_R_PIN, HIGH);
  delay(400);
  moveServo();
  isTackling = false;
}

void onDataReceive(const uint8_t *mac, const uint8_t *incomingDataPtr, int len)
{
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));

  joy1Y = incomingData.joy1;
  joy2Y = incomingData.joy2;
  if (joy1Y > 0)

  {
    digitalWrite(DIR_L_PIN, HIGH);
  }
  else if (joy1Y < 0)
  {
    digitalWrite(DIR_L_PIN, LOW);
  }

  if (joy2Y > 0)
  {
    digitalWrite(DIR_R_PIN, HIGH);
  }
  else if (joy2Y < 0)
  {
    digitalWrite(DIR_R_PIN, LOW);
  }

  if (incomingData.btn1) // Encender Brushless
  {
    valorPWM = microSegundosADutyCycle(PULSE_VELOCIDAD);
  }
  if (incomingData.btn2) // Apagar Brushless
  {
    valorPWM = microSegundosADutyCycle(PULSE_MIN);
  }

  if (incomingData.btn3)
  {
    tackle();
  }

  if (incomingData.btn4)
  {
    moveServo();
  }

  ledcWrite(ledChannel, valorPWM);

  Serial.println("Data received:");
  Serial.print("  Buttons: ");
  Serial.print(incomingData.btn1);
  Serial.print(", ");
  Serial.print(incomingData.btn2);
  Serial.print(", ");
  Serial.print(incomingData.btn3);
  Serial.print(", ");
  Serial.println(incomingData.btn4);
  Serial.print("  Joysticks: ");
  Serial.print(incomingData.joy1);
  Serial.print(" | ");
  Serial.println(incomingData.joy2);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("DojoBot");
  // Configurar el PWM del ESP32 para el ESC
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ESC_PIN, ledChannel);
  // Calibración
  ledcWrite(ledChannel, microSegundosADutyCycle(PULSE_MAX));
  delay(3000);
  ledcWrite(ledChannel, microSegundosADutyCycle(PULSE_MIN));
  delay(1000);

  servoMotor.attach(SERVO_PIN);

  pinMode(DIR_L_PIN, OUTPUT);
  pinMode(STEP_L_PIN, OUTPUT);
  pinMode(DIR_R_PIN, OUTPUT);
  pinMode(STEP_R_PIN, OUTPUT);
  pinMode(ENABLE_STEPPERS, OUTPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataReceive);
  Serial.println("ESP-NOW initialized, waiting for controller...");
}

void loop()
{
  if (!isTackling)
  {
    if (joy1Y == 0)
    {
      digitalWrite(STEP_L_PIN, LOW);
    }
    else
    {
      digitalWrite(STEP_L_PIN, HIGH);
      delayMicroseconds(800 - joy1Y * 2);
    }

    if (joy2Y == 0)
    {
      digitalWrite(STEP_R_PIN, LOW);
    }
    else
    {
      digitalWrite(STEP_R_PIN, HIGH);
      delayMicroseconds(800 - joy2Y * 2);
    }
  }
}
