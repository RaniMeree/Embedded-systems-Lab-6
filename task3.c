//*****************************************************************************
//
// task3.c - Sensor Reading with Harmonic Periods and Gatekeeper Task
//
// This program demonstrates:
// 1. Periodic sensor reading tasks with harmonic periods
// 2. Use of queues to send data between tasks
// 3. Gatekeeper task pattern for centralized data processing
//
// HARMONIC PERIODS (as specified in PDF):
// - Microphone:      Tmic = 5ms   (reads every 5ms)
// - Joystick:        Tjoy = 10ms  (reads every 10ms)
// - Accelerometer:   Tacc = 20ms  (reads every 20ms)
// - Gatekeeper:      Tgate = 40ms (processes every 40ms)
//
// In 40ms, gatekeeper receives:
// - 8 readings from Microphone (40/5 = 8)
// - 4 readings from Joystick (40/10 = 4)
// - 2 readings from Accelerometer (40/20 = 2)
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "uartstdio.h"

// Include fake sensor functions
extern void initialize_fake_sensors(void);
extern void read_accel(uint16_t *sensor);
extern void read_joystick(uint16_t *sensor);
extern void read_microphone(float *sensor);

//*****************************************************************************
// Constants
//*****************************************************************************
#define MIC_PERIOD_MS       5
#define JOYSTICK_PERIOD_MS  10
#define ACCEL_PERIOD_MS     20
#define GATEKEEPER_PERIOD_MS 40

#define QUEUE_LENGTH        10

// Expected readings per gatekeeper cycle
#define MIC_READINGS_PER_CYCLE      8  // 40ms / 5ms = 8
#define JOYSTICK_READINGS_PER_CYCLE 4  // 40ms / 10ms = 4
#define ACCEL_READINGS_PER_CYCLE    2  // 40ms / 20ms = 2

//*****************************************************************************
// Data Structures
//*****************************************************************************

// Structure to hold sensor reading with identifier
typedef struct {
    uint8_t sensorType;  // 0=mic, 1=joystick, 2=accel
    union {
        float micValue;
        uint16_t joystickValues[2];  // X, Y
        uint16_t accelValues[3];     // X, Y, Z
    } data;
} SensorReading_t;

//*****************************************************************************
// Global Variables
//*****************************************************************************
static QueueHandle_t sensorQueue;

//*****************************************************************************
// Microphone Task - Reads every 5ms
//*****************************************************************************
void MicrophoneTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    SensorReading_t reading;
    float micData[1];
    
    // Initialize the last wake time
    xLastWakeTime = xTaskGetTickCount();
    
    reading.sensorType = 0; // Microphone
    
    while(1)
    {
        // Read microphone sensor
        read_microphone(micData);
        reading.data.micValue = micData[0];
        
        // Send to queue (non-blocking to prevent task blocking)
        if (xQueueSend(sensorQueue, &reading, 0) != pdPASS)
        {
            // Queue full - data lost (in real system, would handle this)
        }
        
        // Wait for next period (5ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_PERIOD_MS));
    }
}

//*****************************************************************************
// Joystick Task - Reads every 10ms
//*****************************************************************************
void JoystickTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    SensorReading_t reading;
    uint16_t joyData[2];
    
    // Initialize the last wake time
    xLastWakeTime = xTaskGetTickCount();
    
    reading.sensorType = 1; // Joystick
    
    while(1)
    {
        // Read joystick sensor
        read_joystick(joyData);
        reading.data.joystickValues[0] = joyData[0];
        reading.data.joystickValues[1] = joyData[1];
        
        // Send to queue
        if (xQueueSend(sensorQueue, &reading, 0) != pdPASS)
        {
            // Queue full - data lost
        }
        
        // Wait for next period (10ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(JOYSTICK_PERIOD_MS));
    }
}

//*****************************************************************************
// Accelerometer Task - Reads every 20ms
//*****************************************************************************
void AccelerometerTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    SensorReading_t reading;
    uint16_t accelData[3];
    
    // Initialize the last wake time
    xLastWakeTime = xTaskGetTickCount();
    
    reading.sensorType = 2; // Accelerometer
    
    while(1)
    {
        // Read accelerometer sensor
        read_accel(accelData);
        reading.data.accelValues[0] = accelData[0];
        reading.data.accelValues[1] = accelData[1];
        reading.data.accelValues[2] = accelData[2];
        
        // Send to queue
        if (xQueueSend(sensorQueue, &reading, 0) != pdPASS)
        {
            // Queue full - data lost
        }
        
        // Wait for next period (20ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ACCEL_PERIOD_MS));
    }
}

//*****************************************************************************
// Gatekeeper Task - Processes data every 40ms
// Collects all sensor readings and calculates averages
//*****************************************************************************
void GatekeeperTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    SensorReading_t reading;
    
    // Accumulators for averaging
    float micSum = 0;
    uint32_t joyXSum = 0, joyYSum = 0;
    uint32_t accelXSum = 0, accelYSum = 0, accelZSum = 0;
    
    // Counters for each sensor type
    int micCount = 0;
    int joyCount = 0;
    int accelCount = 0;
    
    // Initialize the last wake time
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // Wait for the gatekeeper period to elapse
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(GATEKEEPER_PERIOD_MS));
        
        // Small delay to allow sensor tasks to post their readings
        vTaskDelay(pdMS_TO_TICKS(2));
        
        // Reset accumulators at start of period
        micSum = 0;
        joyXSum = 0; joyYSum = 0;
        accelXSum = 0; accelYSum = 0; accelZSum = 0;
        micCount = 0; joyCount = 0; accelCount = 0;
        
        // Collect all readings that accumulated during the last period (non-blocking)
        while(xQueueReceive(sensorQueue, &reading, 0) == pdPASS)
        {
            switch(reading.sensorType)
            {
                case 0: // Microphone
                    micSum += reading.data.micValue;
                    micCount++;
                    break;
                    
                case 1: // Joystick
                    joyXSum += reading.data.joystickValues[0];
                    joyYSum += reading.data.joystickValues[1];
                    joyCount++;
                    break;
                    
                case 2: // Accelerometer
                    accelXSum += reading.data.accelValues[0];
                    accelYSum += reading.data.accelValues[1];
                    accelZSum += reading.data.accelValues[2];
                    accelCount++;
                    break;
            }
        }
        
        // Calculate and display averages
        UARTprintf("\r\n========== Sensor Averages (40ms period) ==========\r\n");
        
        if (micCount > 0)
        {
            float micAvg = micSum / micCount;
            // Convert float to integer parts for printing (TI UARTprintf limitation)
            int micInt = (int)micAvg;
            int micFrac = (int)((micAvg - micInt) * 100);
            UARTprintf("Microphone: %d.%02d (from %d readings)\r\n", micInt, micFrac, micCount);
        }
        else
        {
            UARTprintf("Microphone: No data\r\n");
        }
        
        if (joyCount > 0)
        {
            uint16_t joyXAvg = joyXSum / joyCount;
            uint16_t joyYAvg = joyYSum / joyCount;
            UARTprintf("Joystick: X=%d, Y=%d (from %d readings)\r\n", 
                       joyXAvg, joyYAvg, joyCount);
        }
        else
        {
            UARTprintf("Joystick: No data\r\n");
        }
        
        if (accelCount > 0)
        {
            uint16_t accelXAvg = accelXSum / accelCount;
            uint16_t accelYAvg = accelYSum / accelCount;
            uint16_t accelZAvg = accelZSum / accelCount;
            UARTprintf("Accelerometer: X=%d, Y=%d, Z=%d (from %d readings)\r\n", 
                       accelXAvg, accelYAvg, accelZAvg, accelCount);
        }
        else
        {
            UARTprintf("Accelerometer: No data\r\n");
        }
        
        UARTprintf("Expected: Mic=%d, Joy=%d, Accel=%d\r\n",
                   MIC_READINGS_PER_CYCLE, JOYSTICK_READINGS_PER_CYCLE, 
                   ACCEL_READINGS_PER_CYCLE);
        UARTprintf("===================================================\r\n");
        
        // Wait for next gatekeeper period (40ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(GATEKEEPER_PERIOD_MS));
    }
}

//*****************************************************************************
// Hardware Initialization for Task 3
//*****************************************************************************
void Task3_HardwareInit(void)
{
    // Initialize UART for output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    // Configure UART pins
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Initialize UART for console I/O
    UARTStdioConfig(0, 115200, 120000000);
    
    // Initialize fake sensors
    initialize_fake_sensors();
}

//*****************************************************************************
// Task Initialization
//*****************************************************************************
void Task3_Init(void)
{
    // Initialize hardware
    Task3_HardwareInit();
    
    // Create queue for sensor data
    sensorQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorReading_t));
    
    // Print welcome message
    UARTprintf("\r\n");
    UARTprintf("========================================\r\n");
    UARTprintf("Sensor Reading Task - Task 3\r\n");
    UARTprintf("========================================\r\n");
    UARTprintf("Harmonic Periods:\r\n");
    UARTprintf("  Microphone:    %d ms\r\n", MIC_PERIOD_MS);
    UARTprintf("  Joystick:      %d ms\r\n", JOYSTICK_PERIOD_MS);
    UARTprintf("  Accelerometer: %d ms\r\n", ACCEL_PERIOD_MS);
    UARTprintf("  Gatekeeper:    %d ms\r\n", GATEKEEPER_PERIOD_MS);
    UARTprintf("========================================\r\n\r\n");
    
    // Create sensor tasks with appropriate priorities
    // Higher frequency = higher priority (rate-monotonic scheduling)
    xTaskCreate(MicrophoneTask,
                "Microphone",
                128,
                NULL,
                4,  // Highest priority (5ms period)
                NULL);
    
    xTaskCreate(JoystickTask,
                "Joystick",
                128,
                NULL,
                3,  // High priority (10ms period)
                NULL);
    
    xTaskCreate(AccelerometerTask,
                "Accelerometer",
                128,
                NULL,
                2,  // Medium priority (20ms period)
                NULL);
    
    xTaskCreate(GatekeeperTask,
                "Gatekeeper",
                256,
                NULL,
                1,  // Lowest priority (40ms period)
                NULL);
}

//*****************************************************************************
// IMPLEMENTATION NOTES:
//
// 1. HARMONIC PERIODS:
//    - All periods are multiples of the smallest period (5ms)
//    - This ensures predictable timing relationships
//    - Gatekeeper period contains exact multiples of sensor periods
//
// 2. RATE-MONOTONIC SCHEDULING:
//    - Tasks with shorter periods get higher priorities
//    - Microphone (5ms) has highest priority
//    - Gatekeeper (40ms) has lowest priority
//
// 3. QUEUE USAGE:
//    - All sensor tasks send data to one queue
//    - Gatekeeper task receives from queue
//    - Queue provides decoupling between producers and consumer
//
// 4. GATEKEEPER PATTERN:
//    - Centralized task for processing sensor data
//    - Calculates averages over 40ms window
//    - Provides consistent interface to sensor data
//
// 5. TIMING:
//    - vTaskDelayUntil ensures periodic execution
//    - More accurate than vTaskDelay
//    - Accounts for task execution time
//
// 6. DATA STRUCTURE:
//    - Union saves memory (only one sensor type active at a time)
//    - sensorType field identifies which union member is valid
//*****************************************************************************
