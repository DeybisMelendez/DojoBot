#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

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

void onDataReceive(const uint8_t *mac, const uint8_t *incomingDataPtr, int len)
{
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
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
  Serial.println();
  Serial.println("DojoBot");

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
  delay(100);
}
