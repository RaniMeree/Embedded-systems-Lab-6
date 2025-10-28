

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uartstdio.h"


#define BUFFER_SIZE 10 //Buffer size



static int8_t buffer[BUFFER_SIZE];  // Use signed to show negative values as bugs!
static int writeIndex = 0;
static int readIndex = 0;

// Task handles for sleep/wakeup simulation
static TaskHandle_t producerHandle = NULL;
static TaskHandle_t consumerHandle = NULL;


// Print buffer state for visualization
static void printBufferState(void)
{
    int i;
    UARTprintf("  Buffer: [");
    
    for (i = 0; i < BUFFER_SIZE; i++)
    {
        if (i > 0) UARTprintf(" ");
        UARTprintf("%3d", buffer[i]);
    }
    UARTprintf("]\n");
    
    UARTprintf("          ");
    for (i = 0; i < BUFFER_SIZE; i++)
    {
        if (i == readIndex && i == writeIndex)
            UARTprintf(" R/W");
        else if (i == readIndex)
            UARTprintf("  R ");
        else if (i == writeIndex)
            UARTprintf("  W ");
        else
            UARTprintf("    ");
    }
    UARTprintf("\n");
    UARTprintf("  readIndex=%d, writeIndex=%d\r\n\r\n", 
               readIndex, writeIndex);
}


// Producer: Add byte to buffer 
static void putByteIntoBuffer(void)
{
    // PROBLEM: Not atomic! Consumer might modify same slot
    buffer[writeIndex] = buffer[writeIndex] + 1;
    
    UARTprintf("  [PRODUCER] Added 1 to buffer[%d] (now = %d)\r\n",
               writeIndex, buffer[writeIndex]);
    writeIndex = (writeIndex + 1) % BUFFER_SIZE;



    // Show buffer state
    printBufferState();
}

// Consumer: Remove byte from buffer 
static void removeByteFromBuffer(void)
{
    // PROBLEM: Not atomic! Producer might modify same slot
    buffer[readIndex] = buffer[readIndex] - 1;
    
    UARTprintf("  [CONSUMER]  Subtracted 1 from buffer[%d] (now = %d)\r\n",
               readIndex, buffer[readIndex]);
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    
    // Show buffer state
    printBufferState();
}


static void ProducerTask(void *pvParameters)
{
    while(1)
    {
        // PROBLEM: No mutual exclusion!
        putByteIntoBuffer();
        
        // Small delay to simulate production time
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

//*****************************************************************************
// Consumer Task - Has race conditions!
//*****************************************************************************
static void ConsumerTask(void *pvParameters)
{
    while(1)
    {
        // PROBLEM: No mutual exclusion!
        removeByteFromBuffer();
        
        // Small delay to simulate consumption time
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void Task1a_HardwareInit(void)
{
    // Enable UART0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    // Configure GPIO Pins for UART mode
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Initialize UART for console I/O
    UARTStdioConfig(0, 115200, 120000000);
}


void Task1a_Init(void)
{
    // Initialize UART
    Task1a_HardwareInit();
    
    // Print header
    UARTprintf("\r\n");
    UARTprintf("========================================\r\n");
    UARTprintf("Task 1a: Producer-Consumer WITHOUT Semaphores\r\n");
    UARTprintf("========================================\r\n");
    UARTprintf("WATCH FOR PROBLEMS:\r\n");
    UARTprintf("- Buffer slots going NEGATIVE (< 0)\r\n");
    UARTprintf("- Buffer slots exceeding 1 (> 1)\r\n");
    UARTprintf("- These prove race conditions!\r\n");
    UARTprintf("========================================\r\n\r\n");
    
    // Create producer task
    xTaskCreate(ProducerTask,           
                "Producer",              
                256,                     
                NULL,                    
                2,                       
                &producerHandle);        
    
    // Create consumer task
    xTaskCreate(ConsumerTask,           
                "Consumer",              
                256,                    
                NULL,                    
                2,                       
                &consumerHandle);       
}
