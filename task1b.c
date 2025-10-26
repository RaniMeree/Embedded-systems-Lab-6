//*****************************************************************************
//
// task1b.c - Producer-Consumer Problem WITH Semaphores
// This implementation solves all the problems from task1a using semaphores
//
// SOLUTIONS IMPLEMENTED:
// 1. Counting Semaphore (emptySlots) - tracks empty slots in buffer
// 2. Counting Semaphore (fullSlots) - tracks filled slots in buffer
// 3. Binary Semaphore (mutex) - provides mutual exclusion for buffer access
//
// HOW IT SOLVES THE PROBLEMS:
// - No race condition: mutex protects shared variables
// - No lost wakeup: semaphores properly signal between tasks
// - No buffer corruption: mutex ensures exclusive access
// - Atomic operations: semaphore operations are atomic
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
#include "semphr.h"
#include "uartstdio.h"

//*****************************************************************************
// Global Variables and Constants
//*****************************************************************************
#define BUFFER_SIZE 10
#define NUM_BYTES_TO_PROCESS 50

// Circular buffer for bytes
static uint8_t buffer[BUFFER_SIZE];
static int writeIndex = 0;
static int readIndex = 0;

// Semaphores - THE SOLUTION!
static SemaphoreHandle_t emptySlots;  // Counting semaphore for empty slots
static SemaphoreHandle_t fullSlots;   // Counting semaphore for full slots
static SemaphoreHandle_t mutex;       // Binary semaphore for mutual exclusion
static SemaphoreHandle_t uartMutex;   // Binary semaphore for UART output (for clean demo)

// Statistics (now protected by mutex)
static int bytesProduced = 0;
static int bytesConsumed = 0;

//*****************************************************************************
// Helper Functions
//*****************************************************************************

// Simulate producing a byte
static uint8_t produceByte(void)
{
    static uint8_t value = 0;
    return value++;
}

// Put byte into buffer (called within mutex protection)
static void putByteIntoBuffer(uint8_t byte)
{
    buffer[writeIndex] = byte;
    
    if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
    UARTprintf("  [PRODUCER] Put byte %d into buffer[%d]\r\n", byte, writeIndex);
    if (uartMutex) xSemaphoreGive(uartMutex);
    
    writeIndex = (writeIndex + 1) % BUFFER_SIZE;
    
    // Toggle LED to show activity
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 
                 GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0) ^ GPIO_PIN_0);
}

// Remove byte from buffer (called within mutex protection)
static uint8_t removeByteFromBuffer(void)
{
    uint8_t byte = buffer[readIndex];
    
    if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
    UARTprintf("  [CONSUMER] Got byte %d from buffer[%d]\r\n", byte, readIndex);
    if (uartMutex) xSemaphoreGive(uartMutex);
    
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    
    // Toggle LED to show activity
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 
                 GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1) ^ GPIO_PIN_1);
    
    return byte;
}

// Simulate consuming a byte
static void consumeByte(uint8_t byte)
{
    // In real application, would process the byte
    (void)byte; // Suppress unused variable warning
}

//*****************************************************************************
// Producer Task - Now with proper synchronization!
//*****************************************************************************
static void ProducerTask_Semaphore(void *pvParameters)
{
    uint8_t byte;
    
    while(1)
    {
        // Produce a byte
        byte = produceByte();
        
        // SOLUTION: Wait for an empty slot
        // This blocks if buffer is full, automatically handles the waiting
        if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
        UARTprintf("[PRODUCER] Waiting for empty slot...\r\n");
        if (uartMutex) xSemaphoreGive(uartMutex);
        
        xSemaphoreTake(emptySlots, portMAX_DELAY);
        
        if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
        UARTprintf("[PRODUCER] Got empty slot! Producing byte...\r\n");
        if (uartMutex) xSemaphoreGive(uartMutex);
        
        // SOLUTION: Take mutex to get exclusive access to buffer
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        // Put byte into buffer - now protected by mutex!
        putByteIntoBuffer(byte);
        bytesProduced++;
        
        // SOLUTION: Release mutex
        xSemaphoreGive(mutex);
        
        // SOLUTION: Signal that there is one more full slot
        // This will wake up consumer if it's waiting
        xSemaphoreGive(fullSlots);
        
        if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
        UARTprintf("[PRODUCER] Signaled full slot (produced=%d)\r\n", bytesProduced);
        if (uartMutex) xSemaphoreGive(uartMutex);
        
        // Small delay to simulate production time
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Reset counter for demonstration
        if (bytesProduced >= NUM_BYTES_TO_PROCESS)
        {
            if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
            UARTprintf("\r\n[PRODUCER] Produced %d bytes. Pausing...\r\n\r\n", bytesProduced);
            if (uartMutex) xSemaphoreGive(uartMutex);
            
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            xSemaphoreTake(mutex, portMAX_DELAY);
            bytesProduced = 0;
            xSemaphoreGive(mutex);
        }
    }
}

//*****************************************************************************
// Consumer Task - Now with proper synchronization!
//*****************************************************************************
static void ConsumerTask_Semaphore(void *pvParameters)
{
    uint8_t byte;
    
    while(1)
    {
        // SOLUTION: Wait for a full slot
        // This blocks if buffer is empty, automatically handles the waiting
        if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
        UARTprintf("[CONSUMER] Waiting for full slot...\r\n");
        if (uartMutex) xSemaphoreGive(uartMutex);
        
        xSemaphoreTake(fullSlots, portMAX_DELAY);
        
        if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
        UARTprintf("[CONSUMER] Got full slot! Consuming byte...\r\n");
        if (uartMutex) xSemaphoreGive(uartMutex);
        
        // SOLUTION: Take mutex to get exclusive access to buffer
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        // Remove byte from buffer - now protected by mutex!
        byte = removeByteFromBuffer();
        bytesConsumed++;
        
        // SOLUTION: Release mutex
        xSemaphoreGive(mutex);
        
        // SOLUTION: Signal that there is one more empty slot
        // This will wake up producer if it's waiting
        xSemaphoreGive(emptySlots);
        
        if (uartMutex) xSemaphoreTake(uartMutex, portMAX_DELAY);
        UARTprintf("[CONSUMER] Signaled empty slot (consumed=%d)\r\n", bytesConsumed);
        if (uartMutex) xSemaphoreGive(uartMutex);
        
        // Consume the byte
        consumeByte(byte);
        
        // Small delay to simulate consumption time
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

//*****************************************************************************
// Hardware Initialization for Task 1b
//*****************************************************************************
void Task1b_HardwareInit(void)
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
void Task1b_Init(void)
{
    // Initialize UART
    Task1b_HardwareInit();
    
    // Create UART mutex for clean output (for demonstration purposes)
    uartMutex = xSemaphoreCreateBinary();
    xSemaphoreGive(uartMutex);
    
    // Print header
    UARTprintf("\r\n");
    UARTprintf("========================================\r\n");
    UARTprintf("Task 1b: Producer-Consumer WITH Semaphores\r\n");
    UARTprintf("========================================\r\n");
    UARTprintf("USING:\r\n");
    UARTprintf("- Counting semaphore (emptySlots)\r\n");
    UARTprintf("- Counting semaphore (fullSlots)\r\n");
    UARTprintf("- Binary semaphore (mutex)\r\n");
    UARTprintf("========================================\r\n\r\n");
    
    // Create counting semaphore for empty slots
    // Initial value = BUFFER_SIZE (all slots are empty initially)
    // Max value = BUFFER_SIZE
    emptySlots = xSemaphoreCreateCounting(BUFFER_SIZE, BUFFER_SIZE);
    
    // Create counting semaphore for full slots
    // Initial value = 0 (no slots are full initially)
    // Max value = BUFFER_SIZE
    fullSlots = xSemaphoreCreateCounting(BUFFER_SIZE, 0);
    
    // Create binary semaphore for mutual exclusion
    // Acts as a mutex to protect critical sections
    mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(mutex); // Initialize as available
    
    UARTprintf("Semaphores created successfully!\r\n");
    UARTprintf("Starting tasks...\r\n\r\n");
    
    // Create producer task
    xTaskCreate(ProducerTask_Semaphore, // Task function
                "Producer_Sem",          // Task name
                256,                     // Stack size
                NULL,                    // Parameters
                2,                       // Priority
                NULL);                   // Task handle
    
    // Create consumer task
    xTaskCreate(ConsumerTask_Semaphore, // Task function
                "Consumer_Sem",          // Task name
                256,                     // Stack size
                NULL,                    // Parameters
                2,                       // Priority
                NULL);                   // Task handle
}

//*****************************************************************************
// SOLUTIONS EXPLANATION:
//
// 1. COUNTING SEMAPHORE (emptySlots):
//    - Initialized to BUFFER_SIZE (all slots empty)
//    - Producer takes (decrements) before adding item
//    - Consumer gives (increments) after removing item
//    - Automatically blocks producer when buffer is full
//
// 2. COUNTING SEMAPHORE (fullSlots):
//    - Initialized to 0 (no slots full)
//    - Producer gives (increments) after adding item
//    - Consumer takes (decrements) before removing item
//    - Automatically blocks consumer when buffer is empty
//
// 3. BINARY SEMAPHORE (mutex):
//    - Provides mutual exclusion for buffer access
//    - Only one task can access buffer at a time
//    - Prevents race conditions on shared variables
//
// WHY THIS WORKS:
// - Semaphore operations are atomic (no interruption)
// - No lost wakeup: semaphores maintain count
// - No race conditions: mutex protects critical sections
// - Proper synchronization: counting semaphores handle full/empty states
//
// COMPARISON TO task1a:
// - task1a: Uses manual sleep/wakeup → lost wakeup problem
// - task1b: Uses semaphores → no lost wakeup
// - task1a: No protection on byteCount → race condition
// - task1b: Semaphores track count → no race condition
// - task1a: No mutual exclusion → buffer corruption
// - task1b: Mutex ensures exclusive access → no corruption
//*****************************************************************************
