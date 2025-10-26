//*****************************************************************************
//
// Lab 6 - FreeRTOS Main Program
// Embedded Systems Course - Student Level Implementation
//
// This program demonstrates three different tasks:
// - Task 1a: Producer-Consumer WITHOUT semaphores (shows problems)
// - Task 1b: Producer-Consumer WITH semaphores (solves problems)
// - Task 2:  UART Echo with character count display
// - Task 3:  Sensor reading with harmonic periods and gatekeeper
//
// To select which task to run, uncomment ONE of the #define statements below
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "FreeRTOS.h"
#include "task.h"

// Include task headers
#include "task1a.h"  // Producer-Consumer without semaphores
#include "task1b.h"  // Producer-Consumer with semaphores
#include "task2.h"   // UART Echo program
#include "task3.h"   // Sensor reading with gatekeeper

//*****************************************************************************
// Task Selection - Uncomment ONE of these to select which task to run
//*****************************************************************************
#define RUN_TASK1A   // Producer-Consumer WITHOUT semaphores (has problems)
//#define RUN_TASK1B   // Producer-Consumer WITH semaphores (solution)
//#define RUN_TASK2    // UART Echo with character count
//#define RUN_TASK3    // Sensor reading with gatekeeper

//*****************************************************************************
// Hardware Initialization
// Sets up system clock, GPIO for LEDs and buttons
//*****************************************************************************
void HardwareInit(void)
{
    // Set the system clock to 120 MHz
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                            SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Enable GPIO ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // LEDs on Port N
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // LEDs on Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);  // Buttons on Port J
    
    // Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));

    // Configure LED pins as outputs
    // PN0, PN1 - User LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // PF0, PF4 - Additional LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    
    // Turn off all LEDs initially
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);

    // Configure button pins as inputs with pull-up resistors
    // PJ0 - Left button, PJ1 - Right button
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

//*****************************************************************************
// Main Function
// Initializes hardware and starts the selected task
//*****************************************************************************
int main(void)
{
    // Initialize hardware
    HardwareInit();
    
    // Initialize the selected task
    #if defined(RUN_TASK1A)
        // Task 1a: Producer-Consumer WITHOUT semaphores
        // This demonstrates the classic problems:
        // - Race conditions
        // - Lost wakeup problem
        // - Buffer corruption
        Task1a_Init();
        
    #elif defined(RUN_TASK1B)
        // Task 1b: Producer-Consumer WITH semaphores
        // This solves all the problems from Task 1a using:
        // - Counting semaphores (for empty/full slots)
        // - Binary semaphore (mutex for mutual exclusion)
        Task1b_Init();
        
    #elif defined(RUN_TASK2)
        // Task 2: UART Echo Program
        // Features:
        // - Displays last 15 characters in rolling window
        // - Button press shows total character count
        // - Count display lasts 10 seconds
        Task2_Init();
        
    #elif defined(RUN_TASK3)
        // Task 3: Sensor Reading with Gatekeeper
        // Features:
        // - 3 sensor tasks with harmonic periods (5ms, 10ms, 20ms)
        // - Gatekeeper task processes data every 40ms
        // - Calculates averages from all sensor readings
        Task3_Init();
        
    #else
        #error "No task selected! Please define RUN_TASK1A, RUN_TASK1B, RUN_TASK2, or RUN_TASK3"
    #endif
    
    // Start the FreeRTOS scheduler
    // After this point, the scheduler takes control
    vTaskStartScheduler();
    
    // Should never reach here (scheduler runs forever)
    // If we do reach here, there was insufficient heap memory
    while(1)
    {
        // Error: Insufficient heap memory for FreeRTOS
        // Check configTOTAL_HEAP_SIZE in FreeRTOSConfig.h
    }
}

//*****************************************************************************
// TASK DESCRIPTIONS FOR YOUR REPORT:
//
// TASK 1A - Producer-Consumer WITHOUT Semaphores:
// -------------------------------------------------
// PROBLEMS:
// 1. Race Condition: The byteCount variable is shared between producer and
//    consumer without protection. Both tasks increment/decrement it, but
//    these operations are not atomic, leading to incorrect values.
//
// 2. Lost Wakeup: The sleep/wakeup mechanism has a timing problem. If a
//    wakeup signal is sent before the task calls sleep, the signal is lost,
//    and the task may sleep forever.
//
// 3. Buffer Corruption: Producer and consumer can access the buffer
//    simultaneously without synchronization, potentially corrupting data.
//
// 4. Non-Atomic Operations: Checking conditions and acting on them are
//    separate operations, creating a window where another task can change
//    the state, causing inconsistencies.
//
//
// TASK 1B - Producer-Consumer WITH Semaphores:
// ----------------------------------------------
// SOLUTIONS:
// 1. Counting Semaphore (emptySlots): Tracks the number of empty buffer
//    slots. Producer waits (takes) on this before adding data. Consumer
//    signals (gives) this after removing data. Initialized to BUFFER_SIZE.
//
// 2. Counting Semaphore (fullSlots): Tracks the number of full buffer slots.
//    Producer signals (gives) this after adding data. Consumer waits (takes)
//    on this before removing data. Initialized to 0.
//
// 3. Binary Semaphore (mutex): Provides mutual exclusion for buffer access.
//    Ensures only one task accesses the buffer at a time, preventing race
//    conditions and corruption.
//
// WHY IT WORKS:
// - Semaphore operations are atomic (cannot be interrupted)
// - Semaphores maintain count (no lost signals)
// - Automatic blocking and unblocking of tasks
// - Guaranteed mutual exclusion with mutex
//
//
// TASK 2 - UART Echo Program:
// ----------------------------
// IMPLEMENTATION:
// 1. Circular Buffer: Stores the last 15 characters efficiently using
//    modulo arithmetic to wrap around.
//
// 2. Three Tasks:
//    - UART Receive Task: Continuously reads characters and updates buffer
//    - Display Update Task: Refreshes the display periodically
//    - Button Monitor Task: Detects button presses and manages display mode
//
// 3. Display Modes:
//    - Normal: Shows rolling 15-character window
//    - Count: Shows total character count for 10 seconds after button press
//
// 4. Synchronization: Binary semaphore (mutex) protects shared variables
//    (buffer, character count) from race conditions.
//
//
// TASK 3 - Sensor Reading with Gatekeeper:
// -----------------------------------------
// IMPLEMENTATION:
// 1. Harmonic Periods: All sensor periods are multiples of 5ms:
//    - Microphone: 5ms
//    - Joystick: 10ms (2 × 5ms)
//    - Accelerometer: 20ms (4 × 5ms)
//    - Gatekeeper: 40ms (8 × 5ms)
//
// 2. Rate-Monotonic Scheduling: Tasks with shorter periods get higher
//    priorities to meet their deadlines.
//
// 3. Queue Communication: Sensor tasks send readings to a queue, and the
//    gatekeeper task receives them. This decouples producers from consumer.
//
// 4. Gatekeeper Pattern: One centralized task processes all sensor data,
//    calculating averages over a 40ms window. In each cycle:
//    - 8 microphone readings
//    - 4 joystick readings
//    - 2 accelerometer readings
//
// 5. Periodic Execution: vTaskDelayUntil() ensures accurate periodic
//    execution, accounting for task execution time.
//
//*****************************************************************************

