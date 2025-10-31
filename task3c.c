

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
#define GATEKEEPER_PERIOD_MS 40
#define QUEUE_LENGTH        14


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
static QueueHandle_t micQueue;
static QueueHandle_t joystickQueue;
static QueueHandle_t accelQueue;


static void ReadJoystick(uint16_t *x, uint16_t *y)
{
    uint32_t adcValues[2];
    
    ADCProcessorTrigger(ADC0_BASE, 0);
    while(!ADCIntStatus(ADC0_BASE, 0, false));
    ADCIntClear(ADC0_BASE, 0);
    ADCSequenceDataGet(ADC0_BASE, 0, adcValues);
    
    *y = adcValues[0];
    *x = adcValues[1];
}

static void ReadMicrophone(float *value)
{
    uint32_t adcValue;
    
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false));
    ADCIntClear(ADC0_BASE, 1);
    ADCSequenceDataGet(ADC0_BASE, 1, &adcValue);
    
    *value = (adcValue * 3.3) / 4095.0;
}

static void ReadAccelerometer(uint16_t *x, uint16_t *y, uint16_t *z)
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


static void MicrophoneTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    MicReading_t reading;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        ReadMicrophone(&reading.micValue);
        xQueueSend(micQueue, &reading, 0);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_PERIOD_MS));
    }
}


static void JoystickTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    JoystickReading_t reading;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        ReadJoystick(&reading.x, &reading.y);
        xQueueSend(joystickQueue, &reading, 0);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(JOYSTICK_PERIOD_MS));
    }
}


static void AccelerometerTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    AccelReading_t reading;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        ReadAccelerometer(&reading.x, &reading.y, &reading.z);
        xQueueSend(accelQueue, &reading, 0);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ACCEL_PERIOD_MS));
    }
}


static void GatekeeperTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    MicReading_t micReading;
    JoystickReading_t joyReading;
    AccelReading_t accelReading;
    
    float micMin, micMax;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // Clear screen
        UARTprintf("\033[2J\033[H");
        UARTprintf("========== Sensor Readings ==========\r\n\r\n");
        
        // Print microphone readings in dB
        UARTprintf("Microphone:\r\n");
        micMin = 3.3;
        micMax = 0.0;
        
        while(xQueueReceive(micQueue, &micReading, 0) == pdPASS)
        {
            // Track min and max for peak-to-peak calculation
            if (micReading.micValue < micMin) micMin = micReading.micValue;
            if (micReading.micValue > micMax) micMax = micReading.micValue;
            
            // Convert individual reading to dB
            float refVoltage = 0.001;  // 1 mV reference
            float dB;
            
            if (micReading.micValue > 0.0001) {
                dB = 20.0 * log10f(micReading.micValue / refVoltage);
            } else {
                dB = 0.0;
            }
            
            int dbWhole = (int)dB;
            int dbFrac = (int)((dB - dbWhole) * 10);
            if (dbFrac < 0) dbFrac = -dbFrac;
            
            UARTprintf("  %d.%01d dB\r\n", dbWhole, dbFrac);
        }
        
        // Also show peak-to-peak dB
        float peakToPeak = micMax - micMin;
        float dBPeak;
        
        if (peakToPeak > 0.0001) {
            dBPeak = 20.0 * log10f(peakToPeak / 0.001);
        } else {
            dBPeak = 0.0;
        }
        
        int dbpWhole = (int)dBPeak;
        int dbpFrac = (int)((dBPeak - dbpWhole) * 10);
        if (dbpFrac < 0) dbpFrac = -dbpFrac;
        
        UARTprintf("  -> Peak-to-Peak: %d.%01d dB\r\n", dbpWhole, dbpFrac);
        
        // Print joystick readings
        UARTprintf("\r\nJoystick:\r\n");
        while(xQueueReceive(joystickQueue, &joyReading, 0) == pdPASS)
        {
            UARTprintf("  X=%d, Y=%d\r\n", joyReading.x, joyReading.y);
        }
        
        // Print accelerometer readings
        UARTprintf("\r\nAccelerometer:\r\n");
        while(xQueueReceive(accelQueue, &accelReading, 0) == pdPASS)
        {
            UARTprintf("  X=%d, Y=%d, Z=%d\r\n", 
                       accelReading.x, accelReading.y, accelReading.z);
        }
        
        UARTprintf("\r\n=====================================\r\n");
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(GATEKEEPER_PERIOD_MS));
    }
}

// Hardware Initialization
static void Task3C_HardwareInit(void)
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


void Task3C_Init(void)
{
    Task3C_HardwareInit();
    
    // Create 3 separate queues - one for each sensor
    micQueue = xQueueCreate(QUEUE_LENGTH, sizeof(MicReading_t));
    joystickQueue = xQueueCreate(QUEUE_LENGTH, sizeof(JoystickReading_t));
    accelQueue = xQueueCreate(QUEUE_LENGTH, sizeof(AccelReading_t));
    
    UARTprintf("\r\n========================================\r\n");
    UARTprintf("Task 3C - Individual Sensor Readings\r\n");
    UARTprintf("========================================\r\n");
    
    xTaskCreate(MicrophoneTask, "Mic", 128, NULL, 2, NULL);
    xTaskCreate(JoystickTask, "Joy", 128, NULL, 2, NULL);
    xTaskCreate(AccelerometerTask, "Accel", 128, NULL, 2, NULL);
    xTaskCreate(GatekeeperTask, "Gate", 256, NULL, 1, NULL);
}

