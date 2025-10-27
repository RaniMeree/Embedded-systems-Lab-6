

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uartstdio.h"


#define DISPLAY_WINDOW_SIZE 15
#define COUNT_DISPLAY_TIME 10000  // 10 seconds in milliseconds


static char displayBuffer[DISPLAY_WINDOW_SIZE];
static int bufferIndex = 0;
static int totalCharCount = 0;

// Display mode control
static volatile bool showCount = false;
static SemaphoreHandle_t displayMutex;


void UARTReceiveTask(void *pvParameters)
{
    char ch;
    
    while(1)
    {
        // Check if character is available 
        if (UARTCharsAvail(UART0_BASE))
        {
            // Read one character
            ch = UARTCharGetNonBlocking(UART0_BASE);
            
            // Take mutex to update shared variables
            xSemaphoreTake(displayMutex, portMAX_DELAY);
            
            // Add to circular buffer
            displayBuffer[bufferIndex] = ch;
            bufferIndex = (bufferIndex + 1) % DISPLAY_WINDOW_SIZE;
            
            // Increment total count
            totalCharCount++;
            
            xSemaphoreGive(displayMutex);
        }
        
        // Small delay to prevent hogging CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Display charachters and count
void DisplayUpdateTask(void *pvParameters)
{
    int i;
    int idx;
    int startIdx;
    
    while(1)
    {
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        
        if (!showCount)
        {
            // Normal mode: Display last 15 characters
            UARTprintf("\033[2J\033[H"); // Clear screen and move cursor to home
            UARTprintf("Last 15 characters:\r\n");
            UARTprintf("-------------------\r\n");
            
            // Display in the order they were received
            startIdx = bufferIndex;
            for (i = 0; i < DISPLAY_WINDOW_SIZE; i++)
            {
                idx = (startIdx + i) % DISPLAY_WINDOW_SIZE;
                if (displayBuffer[idx] != '\0')
                {
                    UARTprintf("%c", displayBuffer[idx]);
                }
            }
            UARTprintf("\r\n-------------------\r\n");
            UARTprintf("Total Characters: %d\r\n", totalCharCount);
        }
        else
        {
            // Count display mode: Show total character count (highlighted)
            UARTprintf("\033[2J\033[H"); // Clear screen
            UARTprintf("===========================\r\n");
            UARTprintf("  TOTAL CHARACTERS: %d\r\n", totalCharCount);
            UARTprintf("===========================\r\n");
            UARTprintf("(Displayed for 10 seconds)\r\n");
        }
        
        xSemaphoreGive(displayMutex);
        
        // Update display every 200ms
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Button Monitor Task

void ButtonMonitorTask(void *pvParameters)
{
    uint8_t buttonState;
    bool wasPressed = false;
    TickType_t pressTime = 0;
    
    while(1)
    {
        // Read button states (buttons are active low)
        buttonState = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        
        // Check if either button is pressed (0 = pressed)
        if ((buttonState & (GPIO_PIN_0 | GPIO_PIN_1)) != (GPIO_PIN_0 | GPIO_PIN_1))
        {
            if (!wasPressed)
            {
                // Button just pressed
                wasPressed = true;
                showCount = true;
                pressTime = xTaskGetTickCount();
            }
        }
        else
        {
            wasPressed = false;
        }
        
        // Check if 10 seconds have passed since button press
        if (showCount)
        {
            TickType_t currentTime = xTaskGetTickCount();
            if ((currentTime - pressTime) >= pdMS_TO_TICKS(COUNT_DISPLAY_TIME))
            {
                showCount = false;
            }
        }
        
        // Check buttons every 50ms
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}



void Task2_HardwareInit(void)
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
    
    // Initialize display buffer
    memset(displayBuffer, '\0', DISPLAY_WINDOW_SIZE);
}


void Task2_Init(void)
{
    // Initialize hardware
    Task2_HardwareInit();
    
    // Create mutex 
    displayMutex = xSemaphoreCreateBinary();
    xSemaphoreGive(displayMutex);
    
    // Create UART receive task (high priority)
    xTaskCreate(UARTReceiveTask,
                "UART_RX",
                256,
                NULL,
                3,  // Higher priority
                NULL);
    
    // Create display update task
    xTaskCreate(DisplayUpdateTask,
                "Display",
                256,
                NULL,
                2,  // Medium priority
                NULL);
    
    // Create button monitor task
    xTaskCreate(ButtonMonitorTask,
                "Buttons",
                128,
                NULL,
                2,  // Medium priority
                NULL);
    

    UARTprintf("\r\n");
    UARTprintf("========================================\r\n");
    UARTprintf("UART Echo Program - Task 2\r\n");
    UARTprintf("========================================\r\n");

}
