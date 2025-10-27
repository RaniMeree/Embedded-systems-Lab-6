

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uartstdio.h"


#define BUFFER_SIZE 10

static int8_t buffer[BUFFER_SIZE];  // Use signed to show it stays at valid values (0 or 1)
static int writeIndex = 0;
static int readIndex = 0;

static SemaphoreHandle_t emptySlots;  // Counting: tracks available slots for producer
static SemaphoreHandle_t fullSlots;   // Counting: tracks filled slots for consumer
static SemaphoreHandle_t mutex;       // Binary: mutual exclusion for buffer access
static SemaphoreHandle_t uartMutex;   


// Print buffer state 
static void printBufferState(void)
{
    int i;
    
    if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
    
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
    
    if (uartMutex) xSemaphoreGive(uartMutex);
}

// Producer: Add byte to buffer - PROTECTED VERSION
static void putByteIntoBuffer(void)
{
    // Increment the buffer slot
    buffer[writeIndex] = buffer[writeIndex] + 1;
    
    if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
    UARTprintf("  [PRODUCER] Added 1 to buffer[%d] (now = %d)\r\n", 
               writeIndex, buffer[writeIndex]);
    if (uartMutex) xSemaphoreGive(uartMutex);
    
    writeIndex = (writeIndex + 1) % BUFFER_SIZE;
    
    // Show buffer state
    printBufferState();
}

// Consumer: Remove byte from buffer - PROTECTED VERSION
static void removeByteFromBuffer(void)
{
    // Decrement the buffer slot
    buffer[readIndex] = buffer[readIndex] - 1;
    
    if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
    UARTprintf("  [CONSUMER] Subtracted 1 from buffer[%d] (now = %d)\r\n", 
               readIndex, buffer[readIndex]);
    if (uartMutex) xSemaphoreGive(uartMutex);
    
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    
    // Show buffer state
    printBufferState();
}

//*****************************************************************************
// Producer Task - Now with proper synchronization!
//*****************************************************************************
static void ProducerTask_Semaphore(void *pvParameters)
{
    while(1)
    {
        // SOLUTION 1: Wait for an empty slot (counting semaphore)
        // This prevents buffer overflow - blocks if buffer is full
        xSemaphoreTake(emptySlots, portMAX_DELAY);
        
        // SOLUTION 2: Take mutex for exclusive access (binary semaphore)
        // This prevents race conditions on buffer modifications
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        // Add byte to buffer - now protected!
        putByteIntoBuffer();
        
        // SOLUTION 3: Release mutex
        xSemaphoreGive(mutex);
        
        // SOLUTION 4: Signal one more full slot (counting semaphore)
        // This wakes up consumer if waiting
        xSemaphoreGive(fullSlots);
        
        // Small delay to simulate production time
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void ConsumerTask_Semaphore(void *pvParameters)
{
    while(1)
    {
        // Wait for a full slot (counting semaphore). This prevents buffer underflow - blocks if buffer is empty
        xSemaphoreTake(fullSlots, portMAX_DELAY);
        
        // Take mutex for exclusive access (binary semaphore). This prevents race conditions on buffer modifications
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        // Remove byte from buffer - now protected!
        removeByteFromBuffer();
        
        // Release mutex
        xSemaphoreGive(mutex);
        
        // Signal one more empty slot (counting semaphore) to wake up producer if waiting
        xSemaphoreGive(emptySlots);
        
        // Small delay to simulate consumption time
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}


void Task1b_HardwareInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTStdioConfig(0, 115200, 120000000);
}


void Task1b_Init(void)
{
    // Initialize UART
    Task1b_HardwareInit();
    
    // UART mutex 
    uartMutex = xSemaphoreCreateBinary();
    xSemaphoreGive(uartMutex);
    
    // Print header
    UARTprintf("\r\n");
    UARTprintf("========================================\r\n");
    UARTprintf("Task 1b: Producer-Consumer WITH Semaphores\r\n");
    UARTprintf("========================================\r\n");

    
    // Create counting semaphore for empty slots.
    // Initial value = BUFFER_SIZE (all slots are empty initially)
    // Max value = BUFFER_SIZE
    emptySlots = xSemaphoreCreateCounting(BUFFER_SIZE, BUFFER_SIZE);
    
    // Create counting semaphore for full slots
    // Initial value = 0 (no slots are full initially)
    // Max value = BUFFER_SIZE
    fullSlots = xSemaphoreCreateCounting(BUFFER_SIZE, 0);
    
    // Create binary semaphore that acts as a mutex to protect critical sections
    mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(mutex); // Initialize as available
    
    UARTprintf("Semaphores created successfully!\r\n");
    UARTprintf("- emptySlots: %d\r\n", BUFFER_SIZE);
    UARTprintf("- fullSlots: 0\r\n");
    UARTprintf("- mutex: available\r\n");
    UARTprintf("Starting tasks...\r\n\r\n");
    
    // Create producer task
    xTaskCreate(ProducerTask_Semaphore, 
                "Producer_Sem",          
                256,                     
                NULL,                    
                2,                       
                NULL);                   
    
    // Create consumer task
    xTaskCreate(ConsumerTask_Semaphore, 
                "Consumer_Sem",          
                256,                     
                NULL,                    
                2,                       
                NULL);                   
}