#include <LED.h>

#define SYSCTL_RCGC2            0x400FE108
#define GPIO_STRENGTH_2MA       0x00000500
#define GPIODEN                 0x0000051C
#define GPIODIR                 0x00000400
#define GPIOAFSEL               0x00000420

#define PIN4    0x00000010
#define PIN5    0x00000020
#define PINS    (PIN4|PIN5)

unsigned long PORTF_BASE =  0x40025000;         // GPIO_PORT_F 

void
LEDsInit (void)
{
    // Enable the GPIO port used to control the LEDs. //
  
    // Enable this peripheral.
    // Enable GPIO_F port       // LM3S9B92 Datasheet : Pg 291
    *((volatile unsigned long *)SYSCTL_RCGC2) |= 0x0020;
  
     // ----------------------Set the LED GPIOs as output.----------------------- //  
    
    // ***        Set the pad(s) for standard push-pull operation.         ***

    // 2mA drive is enabled : LM3S9B92 Datasheet : Pg 428
    *((volatile unsigned long *)(PORTF_BASE + GPIO_STRENGTH_2MA)) = 
                     (*((volatile unsigned long *)(PORTF_BASE + GPIO_STRENGTH_2MA)) | PINS);
    
    
    // ***                  Set the pin type.                               ***  //
   
    // Set PORTF as  digital enable : LM3S9B92 Datasheet : Pg 437
    *((volatile unsigned long *)(PORTF_BASE + GPIODEN)) = 
                               (*((volatile unsigned long *)(PORTF_BASE + GPIODEN)) | PINS);
                                  

    //***                Make the pin(s) be outputs.                        ***  //
    
    // Set the pin directions as output :  LM3S9B92 Datasheet : Pg 417             
    *((volatile unsigned long *)(PORTF_BASE + GPIODIR)) =
                               (*((volatile unsigned long *)(PORTF_BASE + GPIODIR)) | PINS);
                                  
    // Set other pins to be used for alternate functions :  LM3S9B92 Datasheet : Pg 426
    *((volatile unsigned long *)(PORTF_BASE + GPIOAFSEL)) =
                          (*((volatile unsigned long *)(PORTF_BASE + GPIOAFSEL)) & ~(PINS));
   
    // Turn off both LEDs
    LED_Off(0);
}

void
LED_On(unsigned char led)
{
    // Which LED are we to turn on? //
  
    switch (led)
    {
        // Turn both LEDs on.
        case 0:
        {
            *((volatile unsigned long *)(PORTF_BASE + (PINS << 2))) = PINS;

            break;
        }

        // Turn LED 1 on.
        //
        case 1:
        {
            *((volatile unsigned long *)(PORTF_BASE + (PIN4 << 2))) = PIN4;
            break;
        }

        // Turn LED 2 on.
        case 2:
        {
            *((volatile unsigned long *)(PORTF_BASE  + (PIN5 << 2))) = PIN5;
            break;
        }

        // An invalid LED value was passed.
        default:
            break;
    }
}

void
LED_Off (unsigned char led)
{
    // Which LED are we to turn off? //
  
    switch (led)
    {
        // Turn both LEDs off.
        case 0:
        {
            *((volatile unsigned long *)(PORTF_BASE + (PINS << 2))) = 0;
            break;
        }

        // Turn LED 1 off.
        case 1:
        {
            *((volatile unsigned long *)(PORTF_BASE + (PIN4 << 2))) = 0;
            break;
        }

        // Turn LED 2 off.
        case 2:
        {
           *((volatile unsigned long *)(PORTF_BASE + (PIN5 << 2))) = 0;
            break;
        }

        // An invalid value was passed.
        default:
            break;
    }
}

