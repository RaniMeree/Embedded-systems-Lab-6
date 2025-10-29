

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
static int byteCount = 0;  // Track how many items in buffer

// Task handles - NEEDED for sleep/wakeup!
static TaskHandle_t producerHandle = NULL;
static TaskHandle_t consumerHandle = NULL;



static void sleep(void)
{
    vTaskSuspend(NULL);  // Suspend yourself (NULL = current task)
}

static void wakeup(TaskHandle_t taskHandle)
{
    vTaskResume(taskHandle);  // Resume the specified task
}


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
    
    UARTprintf("] byteCount=%d\n", byteCount);
    
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
    buffer[readIndex] = buffer[readIndex] - 1;
    
    UARTprintf("  [CONSUMER] Subtracted 1 from buffer[%d] (now = %d)\r\n",
               readIndex, buffer[readIndex]);
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    
    // Show buffer state
    printBufferState();
}


static void ProducerTask(void *pvParameters)
{
    while(1)
    {
        // PROBLEM 1: race condition!
        // Consumer can change byteCount between checking and sleeping
        if (byteCount == BUFFER_SIZE)
        {
            UARTprintf("  [PRODUCER] Buffer FULL! Going to sleep...\r\n\r\n");
            sleep();
            UARTprintf("  [PRODUCER] Woken up!\r\n");
        }
        
        putByteIntoBuffer();
        
        // PROBLEM 2: byteCount is not ptotected
        // Consumer might also access byteCount at same time
        byteCount = byteCount + 1;
        
        // If buffer was empty and then has 1 byte the consumer wakeup
        if (byteCount == 1)
        {
            UARTprintf("  [PRODUCER] Waking up consumer!\r\n\r\n");
            wakeup(consumerHandle);
        }
        
        // Small delay to simulate production time
//        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


static void ConsumerTask(void *pvParameters)
{
    while(1)
    {
        // PROBLEM 1: Race condition!
        // Producer might change bytecount between checking and sleeping
        if (byteCount == 0)
        {
            UARTprintf("  [CONSUMER] Buffer EMPTY! Going to sleep...\r\n\r\n");
            sleep();
            UARTprintf("  [CONSUMER] Woken up!\r\n");
        }
        
        removeByteFromBuffer();
        
        // PROBLEM 2: byteCount is not protected! 
        // Producer might also access byteCount at same time 
        byteCount = byteCount - 1;
        
        // If buffer was full and then has a space the producer wakes up
        if (byteCount == BUFFER_SIZE - 1)
        {
            UARTprintf("  [CONSUMER] Waking up producer!\r\n\r\n");
            wakeup(producerHandle);
        }
        
        // Small delay to simulate consumption time
//        vTaskDelay(pdMS_TO_TICKS(100));
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
    UARTprintf("WARNING: THIS HAS RACE CONDITIONS!\r\n");
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
