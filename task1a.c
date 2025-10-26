
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
#define NUM_BYTES_TO_PROCESS 50

// Circular buffer for bytes
static uint8_t buffer[BUFFER_SIZE];
static int writeIndex = 0;
static int readIndex = 0;

// Shared buffer
static volatile int byteCount = 0;

// Task handles for sleep/wakeup simulation
static TaskHandle_t producerHandle = NULL;
static TaskHandle_t consumerHandle = NULL;

// Statistics
static int bytesProduced = 0;
static int bytesConsumed = 0;


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
    UARTprintf("  byteCount=%d, readIndex=%d, writeIndex=%d\r\n\r\n", 
               byteCount, readIndex, writeIndex);
}

// Simulate producing a byte (returns incremental values)
static uint8_t produceByte(void)
{
    static uint8_t value = 0;
    return value++;
}

// Simulate consuming a byte (just increments counter)
static void consumeByte(uint8_t byte)
{
    // In real application, would process the byte
    (void)byte; // Suppress unused variable warning
}


// Put byte into buffer 
static void putByteIntoBuffer(uint8_t byte)
{
    buffer[writeIndex] = byte;
    UARTprintf("  [PRODUCER] Put byte %d into buffer[%d], byteCount=%d\r\n", 
               byte, writeIndex, byteCount);
    writeIndex = (writeIndex + 1) % BUFFER_SIZE;
    
    // Show buffer state
    printBufferState();
}

// Remove byte from buffer 
static uint8_t removeByteFromBuffer(void)
{
    uint8_t byte = buffer[readIndex];
    UARTprintf("  [CONSUMER] Got byte %d from buffer[%d], byteCount=%d\r\n", 
               byte, readIndex, byteCount);
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    
    // Show buffer state
    printBufferState();
    
    return byte;
}


//*****************************************************************************
// Producer Task - Has race conditions!
//*****************************************************************************
static void ProducerTask(void *pvParameters)
{
    uint8_t byte;
    
    while(1)
    {
        // Produce a byte
        byte = produceByte();
        
        // PROBLEM: This check is not atomic! Consumer might change byteCount here
        if (byteCount == BUFFER_SIZE) 
        {
            // Buffer is full, sleep
            UARTprintf("[PRODUCER] Buffer FULL! Going to sleep... byteCount=%d\r\n", byteCount);
            // PROBLEM: If consumer removes item between the check and sleep,
            // we might sleep forever (lost wakeup problem)
            vTaskSuspend(NULL);  // Sleep
            UARTprintf("[PRODUCER] Woke up! byteCount=%d\r\n", byteCount);
        }
        
        // Put byte into buffer - PROBLEM: No mutual exclusion!
        putByteIntoBuffer(byte);
        
        // PROBLEM: byteCount++ is not atomic! Can be interrupted
        byteCount = byteCount + 1;
        bytesProduced++;
        
        // If buffer was empty, wake up consumer
        // PROBLEM: If consumer checks byteCount == 0 and we call wakeup
        // before consumer calls sleep, the wakeup is lost!
        if (byteCount == 1) 
        {
            UARTprintf("[PRODUCER] Waking up CONSUMER (byteCount was 0, now 1)\r\n");
            vTaskResume(consumerHandle);  // Wakeup consumer
        }
        
        // Small delay to simulate production time
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Stop after producing enough bytes
        if (bytesProduced >= NUM_BYTES_TO_PROCESS)
        {
            UARTprintf("\r\n[PRODUCER] Produced %d bytes. Pausing...\r\n\r\n", bytesProduced);
            vTaskDelay(pdMS_TO_TICKS(1000));
            bytesProduced = 0;
        }
    }
}

//*****************************************************************************
// Consumer Task - Has race conditions!
//*****************************************************************************
static void ConsumerTask(void *pvParameters)
{
    uint8_t byte;
    
    while(1)
    {
        // PROBLEM: This check is not atomic! Producer might change byteCount here
        if (byteCount == 0) 
        {
            // Buffer is empty, sleep
            UARTprintf("[CONSUMER] Buffer EMPTY! Going to sleep... byteCount=%d\r\n", byteCount);
            // PROBLEM: Lost wakeup problem
            vTaskSuspend(NULL);  // Sleep
            UARTprintf("[CONSUMER] Woke up! byteCount=%d\r\n", byteCount);
        }
        
        // Remove byte from buffer - PROBLEM: No mutual exclusion!
        byte = removeByteFromBuffer();
        
        // PROBLEM: byteCount-- is not atomic!
        byteCount = byteCount - 1;
        bytesConsumed++;
        
        // If buffer was full, wake up producer
        if (byteCount == BUFFER_SIZE - 1) 
        {
            UARTprintf("[CONSUMER] Waking up PRODUCER (byteCount was %d, now %d)\r\n", 
                       BUFFER_SIZE, byteCount);
            vTaskResume(producerHandle);  // Wakeup producer
        }
        
        // Consume the byte
        consumeByte(byte);
        
        // Small delay to simulate consumption time
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

//*****************************************************************************
// Hardware Initialization for Task 1a
//*****************************************************************************
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

//*****************************************************************************
// Task Initialization
//*****************************************************************************
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
    UARTprintf("- Race conditions on byteCount\r\n");
    UARTprintf("- Lost wakeup signals\r\n");
    UARTprintf("- Buffer corruption\r\n");
    UARTprintf("========================================\r\n\r\n");
    
    // Create producer task
    xTaskCreate(ProducerTask,           // Task function
                "Producer",              // Task name
                256,                     // Stack size
                NULL,                    // Parameters
                2,                       // Priority
                &producerHandle);        // Task handle
    
    // Create consumer task
    xTaskCreate(ConsumerTask,           // Task function
                "Consumer",              // Task name
                256,                     // Stack size
                NULL,                    // Parameters
                2,                       // Priority (same as producer - more contention)
                &consumerHandle);        // Task handle
}

//*****************************************************************************
// PROBLEMS SUMMARY:
//
// 1. RACE CONDITION: byteCount is accessed by both tasks without protection
//    - Both increment/decrement operations are not atomic
//    - Can lead to incorrect count values
//
// 2. LOST WAKEUP: If wakeup() is called before sleep(), the wakeup is lost
//    - Producer might call wakeup before consumer calls sleep
//    - Consumer will then sleep forever
//
// 3. BUFFER CORRUPTION: No mutual exclusion for buffer access
//    - Producer and consumer can access buffer simultaneously
//    - Can corrupt data
//
// 4. NO SYNCHRONIZATION: Checks and actions are not atomic
//    - Time gap between checking condition and acting on it
//    - Other task can change state in between
//
// SOLUTION: Use semaphores (see task1b.c)
//*****************************************************************************
