

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "uartstdio.h"


#define MIC_PERIOD_MS       5
#define JOYSTICK_PERIOD_MS  10
#define ACCEL_PERIOD_MS     20
#define GATEKEEPER_PERIOD_MS 40  // LCM of all periods
#define QUEUE_LENGTH        14

// Expected readings per gatekeeper cycle
#define MIC_READINGS_PER_CYCLE      8  // 40ms / 5ms = 8
#define JOYSTICK_READINGS_PER_CYCLE 4  // 40ms / 10ms = 4
#define ACCEL_READINGS_PER_CYCLE    2  // 40ms / 20ms = 2


// Structures for each sensor type
typedef struct {
    float micValue;
} MicReading_t;

typedef struct {
    uint16_t x;
    uint16_t y;
} JoystickReading_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} AccelReading_t;

// Three separate queues - one for each sensor
QueueHandle_t micQueue;
QueueHandle_t joystickQueue;
QueueHandle_t accelQueue;


void ReadJoystick(uint16_t *x, uint16_t *y)
{
    uint32_t adcValues[2];
    
    ADCProcessorTrigger(ADC0_BASE, 0);
    while(!ADCIntStatus(ADC0_BASE, 0, false));
    ADCIntClear(ADC0_BASE, 0);
    ADCSequenceDataGet(ADC0_BASE, 0, adcValues);
    
    *y = adcValues[0];
    *x = adcValues[1];
}

void ReadMicrophone(float *value)
{
    uint32_t adcValue;
    
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false));
    ADCIntClear(ADC0_BASE, 1);
    ADCSequenceDataGet(ADC0_BASE, 1, &adcValue);
    
    // Debug: Store raw ADC value for debugging (0-4095)
    // The microphone has DC bias at ~1.65V (ADC value ~2048)
    *value = (adcValue * 3.3) / 4095.0;  // Fixed: 4095 not 4096
}

void ReadAccelerometer(uint16_t *x, uint16_t *y, uint16_t *z)
{
    uint32_t adcValues[3];
        ADCProcessorTrigger(ADC0_BASE, 2);
    while(!ADCIntStatus(ADC0_BASE, 2, false));
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, adcValues);
    
    *z = adcValues[0];
    *y = adcValues[1];
    *x = adcValues[2];
}


void MicrophoneTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    MicReading_t reading;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // Read microphone from ADC
        ReadMicrophone(&reading.micValue);
        
        xQueueSend(micQueue, &reading, 0);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_PERIOD_MS));
    }
}


void JoystickTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    JoystickReading_t reading;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // Read joystick from ADC
        ReadJoystick(&reading.x, &reading.y);
        
        xQueueSend(joystickQueue, &reading, 0);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(JOYSTICK_PERIOD_MS));
    }
}


void AccelerometerTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    AccelReading_t reading;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // Read accelerometer
        ReadAccelerometer(&reading.x, &reading.y, &reading.z);
        
        xQueueSend(accelQueue, &reading, 0);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ACCEL_PERIOD_MS));
    }
}


void GatekeeperTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    MicReading_t micReading;
    JoystickReading_t joyReading;
    AccelReading_t accelReading;
    
    float micMin = 3.3, micMax = 0.0;  // Track min/max for peak detection
    uint32_t joyXSum = 0, joyYSum = 0;
    uint32_t accelXSum = 0, accelYSum = 0, accelZSum = 0;
    int micCount = 0, joyCount = 0, accelCount = 0;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // Clear screen at the beginning
        UARTprintf("\033[2J\033[H");
        
        // Reset counters
        joyXSum = 0; joyYSum = 0;
        accelXSum = 0; accelYSum = 0; accelZSum = 0;
        micCount = 0; joyCount = 0; accelCount = 0;
        micMin = 3.3; micMax = 0.0;
        
        // Collect all readings from microphone queue
        while(xQueueReceive(micQueue, &micReading, 0) == pdPASS)
        {
            // Track min and max for peak-to-peak calculation
            if (micReading.micValue < micMin) micMin = micReading.micValue;
            if (micReading.micValue > micMax) micMax = micReading.micValue;
            micCount++;
        }
        
        // Collect all readings from joystick queue
        while(xQueueReceive(joystickQueue, &joyReading, 0) == pdPASS)
        {
            joyXSum += joyReading.x;
            joyYSum += joyReading.y;
            joyCount++;
        }
        
        // Collect all readings from accelerometer queue
        while(xQueueReceive(accelQueue, &accelReading, 0) == pdPASS)
        {
            accelXSum += accelReading.x;
            accelYSum += accelReading.y;
            accelZSum += accelReading.z;
            accelCount++;
        }
        
        // Print averages

        UARTprintf("\r\n========== Average Sensor Data ==========\r\n");
        
        if (micCount > 0)
        {
            float peakToPeak = micMax - micMin;  // Amplitude of sound

            // Convert amplitude to decibels
            // Reference: use a small value to avoid log(0), typical mic reference
            float refVoltage = 0.001;  // 1 mV reference
            float dB;

            if (peakToPeak > 0.0001) {  // Avoid log of very small numbers
                dB = 20.0 * log10f(peakToPeak / refVoltage);
            } else {
                dB = 0.0;  // Silence
            }

            int dbWhole = (int)dB;
            int dbFrac = (int)((dB - dbWhole) * 10);
            if (dbFrac < 0) dbFrac = -dbFrac;  // Handle negative fractions

            UARTprintf("Mic: %d.%01d dB | samples: %d\r\n", dbWhole, dbFrac, micCount);
        }
        
        if (joyCount > 0)
        {
            UARTprintf("Joy: X=%d, Y=%d (%d)\r\n", 
                       joyXSum/joyCount, joyYSum/joyCount, joyCount);
        }
        
        if (accelCount > 0)
        {
            UARTprintf("Accel: X=%d, Y=%d, Z=%d (%d)\r\n", 
                       accelXSum/accelCount, accelYSum/accelCount, 
                       accelZSum/accelCount, accelCount);
        }
        
        UARTprintf("=================================\r\n");
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(GATEKEEPER_PERIOD_MS));



    }
    //UARTprintf("\033[2J\033[H");



}

// Hardware Initialization for Task 3
void Task3_HardwareInit(void)

{
    // Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    // Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    
    // Configure UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 120000000);
    
    // Configure ADC pins
    //GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
    //GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    
    //Patrik
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0  | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    // Sequencer 0: Joystick (2 channels)
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
    
    // Sequencer 1: Microphone (1 channel)
    ADCSequenceDisable(ADC0_BASE, 1);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);
    
    // Sequencer 2: Accelerometer (3 channels)
    ADCSequenceDisable(ADC0_BASE, 2);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 2);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
}


void Task3_Init(void)
{
    Task3_HardwareInit();
    
    // Create 3 separate queues - one for each sensor
    micQueue = xQueueCreate(QUEUE_LENGTH, sizeof(MicReading_t));
    joystickQueue = xQueueCreate(QUEUE_LENGTH, sizeof(JoystickReading_t));
    accelQueue = xQueueCreate(QUEUE_LENGTH, sizeof(AccelReading_t));
    
    UARTprintf("\r\n========================================\r\n");
    UARTprintf("Task 3 - Sensor Reading (3 Queues)\r\n");
    UARTprintf("========================================\r\n");
    
    xTaskCreate(MicrophoneTask, "Mic", 128, NULL, 2, NULL);
    xTaskCreate(JoystickTask, "Joy", 128, NULL, 2, NULL);
    xTaskCreate(AccelerometerTask, "Accel", 128, NULL, 2, NULL);
    xTaskCreate(GatekeeperTask, "Gate", 256, NULL, 1, NULL);
}

