

#include <stdint.h>
#include <stdbool.h>
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
#define QUEUE_LENGTH        10

// Expected readings per gatekeeper cycle
#define MIC_READINGS_PER_CYCLE      8  // 40ms / 5ms = 8
#define JOYSTICK_READINGS_PER_CYCLE 4  // 40ms / 10ms = 4
#define ACCEL_READINGS_PER_CYCLE    2  // 40ms / 20ms = 2


// Structure to hold sensor reading with identifier
typedef struct {
    uint8_t sensorType;  // 0=mic, 1=joystick, 2=accel
    union {float micValue; uint16_t joystickValues[2]; uint16_t accelValues[3];} data;
} SensorReading_t;

static QueueHandle_t sensorQueue;


void ReadJoystick(uint16_t *x, uint16_t *y)
{
    uint32_t adcValues[2];
    
    ADCProcessorTrigger(ADC0_BASE, 0);
    while(!ADCIntStatus(ADC0_BASE, 0, false));
    ADCIntClear(ADC0_BASE, 0);
    ADCSequenceDataGet(ADC0_BASE, 0, adcValues);
    
    *x = adcValues[0];
    *y = adcValues[1];
}

void ReadMicrophone(float *value)
{
    uint32_t adcValue;
    
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false));
    ADCIntClear(ADC0_BASE, 1);
    ADCSequenceDataGet(ADC0_BASE, 1, &adcValue);
    
    *value = (adcValue * 3.3) / 4096.0;
}

void ReadAccelerometer(uint16_t *x, uint16_t *y, uint16_t *z)
{
    uint32_t adcValues[3];
    
    ADCProcessorTrigger(ADC0_BASE, 2);
    while(!ADCIntStatus(ADC0_BASE, 2, false));
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, adcValues);
    
    *x = adcValues[0];
    *y = adcValues[1];
    *z = adcValues[2];
}


void MicrophoneTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    SensorReading_t reading;
    float micValue;
    
    xLastWakeTime = xTaskGetTickCount();
    reading.sensorType = 0;
    
    while(1)
    {
        // Read microphone from ADC
        ReadMicrophone(&micValue);
        reading.data.micValue = micValue;
        
        xQueueSend(sensorQueue, &reading, 0);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_PERIOD_MS));
    }
}


void JoystickTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    SensorReading_t reading;
    uint16_t x, y;
    
    xLastWakeTime = xTaskGetTickCount();
    reading.sensorType = 1;
    
    while(1)
    {
        // Read joystick from ADC
        ReadJoystick(&x, &y);
        reading.data.joystickValues[0] = x;
        reading.data.joystickValues[1] = y;
        
        xQueueSend(sensorQueue, &reading, 0);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(JOYSTICK_PERIOD_MS));
    }
}


void AccelerometerTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    SensorReading_t reading;
    uint16_t x, y, z;
    
    xLastWakeTime = xTaskGetTickCount();
    reading.sensorType = 2;
    
    while(1)
    {
        // Read accelerometer
        ReadAccelerometer(&x, &y, &z);
        reading.data.accelValues[0] = x;
        reading.data.accelValues[1] = y;
        reading.data.accelValues[2] = z;
        
        xQueueSend(sensorQueue, &reading, 0);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ACCEL_PERIOD_MS));
    }
}


void GatekeeperTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    SensorReading_t reading;
    
    float micSum = 0;
    uint32_t joyXSum = 0, joyYSum = 0;
    uint32_t accelXSum = 0, accelYSum = 0, accelZSum = 0;
    int micCount = 0, joyCount = 0, accelCount = 0;
    
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(GATEKEEPER_PERIOD_MS));
        
        // Reset counters
        micSum = 0; joyXSum = 0; joyYSum = 0;
        accelXSum = 0; accelYSum = 0; accelZSum = 0;
        micCount = 0; joyCount = 0; accelCount = 0;
        
        // Collect all readings from queue
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
        
        // Print averages
        UARTprintf("\r\n========== Sensor Data ==========\r\n");
        
        if (micCount > 0)
        {
            float avg = micSum / micCount;
            int whole = (int)avg;
            int frac = (int)((avg - whole) * 100);
            UARTprintf("Mic: %d.%02d V (%d)\r\n", whole, frac, micCount);
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
    }
}

//*****************************************************************************
// Hardware Initialization for Task 3
//*****************************************************************************
void Task3_HardwareInit(void)
{
    // Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    
    // Configure UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 120000000);
    
    // Configure ADC pins
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_4);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    
    // Sequencer 0: Joystick (2 channels)
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
    
    // Sequencer 1: Microphone (1 channel)
    ADCSequenceDisable(ADC0_BASE, 1);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);
    
    // Sequencer 2: Accelerometer (3 channels)
    ADCSequenceDisable(ADC0_BASE, 2);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 2);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH15);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH14);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH13 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
}


void Task3_Init(void)
{
    Task3_HardwareInit();
    
    sensorQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorReading_t));
    
    UARTprintf("\r\n========================================\r\n");
    UARTprintf("Task 3 - Sensor Reading\r\n");
    UARTprintf("========================================\r\n");
    
    xTaskCreate(MicrophoneTask, "Mic", 128, NULL, 4, NULL);
    xTaskCreate(JoystickTask, "Joy", 128, NULL, 3, NULL);
    xTaskCreate(AccelerometerTask, "Accel", 128, NULL, 2, NULL);
    xTaskCreate(GatekeeperTask, "Gate", 256, NULL, 1, NULL);
}

