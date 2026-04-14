#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "hal/comms/lora.h"

// ESP32 Feather V2 Corrected SPI Pins (UNCHANGED)
#define SCK_PIN   5
#define MOSI_PIN  19 
#define MISO_PIN  21 

// SX1276 Control Pins (UNCHANGED)
#define CS_PIN    27
#define RST_PIN   32
#define IRQ_PIN   14

#define LORA_FREQ 915E6 
#define SYNC_WORD 0xF3

uint32_t telemetry_seq = 0;

// FreeRTOS Mutex for protecting the LoRa SPI bus
SemaphoreHandle_t xLoRaMutex;

// ----------------------------------------------------------------
// TASK 1: Periodic Telemetry
// ----------------------------------------------------------------
void vTelemetryTask(void *pvParameters) {
  // FreeRTOS tasks must run in an infinite loop
  for (;;) {
    // Wait exactly 5000 ticks (milliseconds)
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Attempt to take the Mutex (Wait forever if necessary)
    if (xSemaphoreTake(xLoRaMutex, portMAX_DELAY) == pdTRUE) {
      Serial.printf("[AUTO TX] Sending Telemetry [SEQ: %d]\n", telemetry_seq);

      LoRa.beginPacket();
      LoRa.print("TELEMETRY | SEQ: ");
      LoRa.print(telemetry_seq);
      LoRa.endPacket(); 

      telemetry_seq++;

      // We are done with the SPI bus. Give the Mutex back so the other task can use it.
      xSemaphoreGive(xLoRaMutex);
    }
  }
}

// ----------------------------------------------------------------
// TASK 2: Serial Command Monitor
// ----------------------------------------------------------------
void vSerialTask(void *pvParameters) {
  for (;;) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      if (command.length() > 0) {
        // Attempt to take the Mutex before touching the LoRa module
        if (xSemaphoreTake(xLoRaMutex, portMAX_DELAY) == pdTRUE) {
          Serial.print("[MANUAL TX] Sending Command: ");
          Serial.println(command);

          LoRa.beginPacket();
          LoRa.print("CMD: ");
          LoRa.print(command);
          LoRa.endPacket(); 

          // Give the Mutex back
          xSemaphoreGive(xLoRaMutex);
        }
      }
    }
    
    // Crucial: A brief delay prevents this while-loop from hogging the CPU and triggering the Watchdog Timer
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

// ----------------------------------------------------------------
// SETUP: Initialization & Task Spawning
// ----------------------------------------------------------------
void LoRa_Init() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n[SYSTEM] ESP32 Feather FreeRTOS TX Node Booting...");

  // 1. Initialize Hardware (UNCHANGED)
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
  LoRa.setSPI(SPI);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[FATAL] LoRa transceiver not detected. Halt.");
    while (true); 
  }

  LoRa.setSpreadingFactor(7); 
  LoRa.setSyncWord(SYNC_WORD);

  // 2. Create the Mutex
  xLoRaMutex = xSemaphoreCreateMutex();
  if (xLoRaMutex == NULL) {
    Serial.println("[FATAL] FreeRTOS Mutex creation failed. Halt.");
    while (true);
  }

  // 3. Spawn the Tasks
  // xTaskCreate( FunctionName, "TaskName", StackSize, Parameters, Priority, TaskHandle );
  
  xTaskCreate(
    vTelemetryTask,   
    "TelemetryTask",  
    4096,             // Stack size (4KB is plenty for simple SPI comms)
    NULL,             
    1,                // Priority (1 = Low)
    NULL              
  );

  xTaskCreate(
    vSerialTask,      
    "SerialTask",     
    4096,             
    NULL,             
    2,                // Priority (2 = Higher than Telemetry, UI input feels instantly responsive)
    NULL              
  );

  Serial.println("[SYSTEM] FreeRTOS Tasks Spawned. Ready.");
}
