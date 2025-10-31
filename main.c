
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
#include "task1a.h"  
#include "task1b.h"  
#include "task2.h"
#include "task3.h"
#include "task3c.h"




//#define RUN_TASK1A   
//#define RUN_TASK1B
//#define RUN_TASK2
#define RUN_TASK3
//#define RUN_TASK3C

void HardwareInit(void)
{
    // Set the system clock to 120 MHz
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                            SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Enable GPIO ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);  
    
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


int main(void)
{
    // Initialize hardware
    HardwareInit();
    
    // Initialize the selected task
    #if defined(RUN_TASK1A)

        Task1a_Init();
        
    #elif defined(RUN_TASK1B)
 
        Task1b_Init();
        
    #elif defined(RUN_TASK2)

        Task2_Init();

    #elif defined(RUN_TASK3)

        Task3_Init();

    #elif defined(RUN_TASK3C)

        Task3C_Init();

    #else
        #error "No task selected! Please define RUN_TASK1A, RUN_TASK1B, RUN_TASK2, RUN_TASK3, or RUN_TASK3C"
    #endif
    

    vTaskStartScheduler();
    

    while(1)
    {

    }
}
