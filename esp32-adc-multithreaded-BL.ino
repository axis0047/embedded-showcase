#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_task_wdt.h"
#include <SoftwareSerial.h>

// Define the Task Handles
TaskHandle_t Task1;
TaskHandle_t Task2;

// ADC pin
const int adcPin = 34;

// Interrupt pins
const int interruptPin = 13;
const int interruptPin2 = 12;

// Define HC-05 Bluetooth module pins
const int bluetoothTx = 1;  
const int bluetoothRx = 3;

// Shared variable for ADC value
volatile int adcValue = -1;
volatile int touch = 0;

// Shared array for ADC values
const int arrayLength = 100;
int adcValues[arrayLength];
volatile int adcIndex = 0;
int touchValues[arrayLength];
volatile int touchIndex = 0;

// Create Bluetooth serial connection
SoftwareSerial bluetooth(bluetoothRx, bluetoothTx);

// Mutex for ADC values array
SemaphoreHandle_t adcMutex;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterruptTouch() {
  touch = !touch;
}

void IRAM_ATTR handleInterrupt() {
  // Read the ADC value
  int value = analogRead(adcPin);
  // Update shared variable safely
  portENTER_CRITICAL_ISR(&mux);
  adcValue = value;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  bluetooth.begin(9600);
  delay(1000);

  adcMutex = xSemaphoreCreateMutex();

  // Enable the task watchdog timer for the main loop task (loopTask) with a timeout of 10 seconds
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 10000, // Timeout in milliseconds
    .trigger_panic = true // Panic on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); // Add the current task (loopTask) to the TWDT

  memset(adcValues, -1, sizeof(adcValues));

  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);

  // Initialize the ADC
  analogReadResolution(12); // 12-bit resolution
  analogSetPinAttenuation(adcPin, ADC_11db);

  // Attach the interrupt to the pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), handleInterruptTouch, CHANGE);

  // Create Task1 on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    4096,        /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(500);

  // Create Task2 on core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
    4096,        /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500);
}

// Task1code: handles printing ADC values when array is full
void Task1code(void * pvParameters) {
  esp_task_wdt_add(NULL); // Add this task to the TWDT
  for (;;) {
    if (xSemaphoreTake(adcMutex, portMAX_DELAY) == pdTRUE) {
      if (adcValues[arrayLength - 1] > -1) {
        // Array is full, print the values
        for (int i = 0; i < arrayLength; i++) {
          Serial.print(adcValues[i]);
          Serial.print(" ");
          bluetooth.print(adcValues[i]);
          bluetooth.print(" ");
        }
        Serial.println();
        for (int i = 0; i < arrayLength; i++) {
          Serial.print(touchValues[i]);
          Serial.print(" ");
          bluetooth.print(touchValues[i]);
          bluetooth.print(" ");
        }
        Serial.println();
        bluetooth.println();
        // Clear the array
        memset(adcValues, -1, sizeof(adcValues));
        memset(touchValues, 0, sizeof(touchValues));
        adcIndex = 0;
        touchIndex = 0;
      }
      // Release mutex after modifying shared resource
      xSemaphoreGive(adcMutex);
    }
    // Reset the watchdog timer for this task
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1)); // Adjust delay as necessary
  }
}

// Task2code: handles storing ADC values in the array
void Task2code(void * pvParameters) {
  esp_task_wdt_add(NULL); // Add this task to the TWDT
  for (;;) {
    if (xSemaphoreTake(adcMutex, portMAX_DELAY) == pdTRUE) {
      if (adcValue >= 0) {
        if (adcIndex < arrayLength) {
          adcValues[adcIndex] = adcValue;
          adcValue = -1; // Reset value after reading
          adcIndex++;
          touchValues[touchIndex] = touch;
          touchIndex++;
        }
      }
      xSemaphoreGive(adcMutex); // Always release the mutex
    }
    // Reset the watchdog timer for this task
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1)); // Adjust delay as necessary
  }
}

void loop() {
  // Empty, everything is handled by tasks
  esp_task_wdt_reset(); // Reset the watchdog timer for this task
  vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as necessary
}