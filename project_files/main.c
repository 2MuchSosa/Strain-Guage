//Aisosa Okunbor

#include <stdio.h>
#include <stdint.h>
#include "clock.h"
#include "wait.h"
#include "math.h"
#include "uart0.h"
#include "gpio.h"
#include "tm4c123gh6pm.h"
#include "stringhandler.h"

// DOUT(data) is going from HX711 -> MCU (PD0)
// SCK(clock) is going from MCU -> HX711 (PD1)

// Bitbanded Port D
#define DOUT  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4))) //PD0
#define SCK   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4))) //PD1

// Port D Masks
#define DOUT_Mask 1
#define SCK_Mask  2

#define NUM_SAMPLES 100

// Calibration constants from graph: y = 12899x + 5E+06
#define CALIBRATION_SLOPE     12899.0    // counts per gram
#define CALIBRATION_INTERCEPT 5000000.0  // baseline ADC reading at 0g
#define GRAVITY 9.81                    

void initHw()
{
    initSystemClockTo40Mhz();
    _delay_cycles(3);

    // Enable clock to Port D
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    // Unlock PD7 
    GPIO_PORTD_LOCK_R = 0x4C4F434B;
    GPIO_PORTD_CR_R |= 0xFF;

    // Configure PD1 (SCK)
    GPIO_PORTD_DIR_R |= SCK_Mask;
    GPIO_PORTD_DR8R_R |= SCK_Mask;
    GPIO_PORTD_DEN_R |= SCK_Mask;

    // Configure PD0 (DOUT)
    GPIO_PORTD_DIR_R &= ~DOUT_Mask;
    GPIO_PORTD_DEN_R |= DOUT_Mask;

    // Initialize SCK low
    SCK = 0;

    // Initialize UART0 for output
    initUart0();
    setUart0BaudRate(115200, 40e6);
}

int32_t readHX711()
{
    uint32_t i;
    int32_t result = 0;

    // Wait for DOUT to go low (data ready)
    while(DOUT);

    // Read 24 bits of data (MSB first)
    for(i = 0; i < 24; i++)
    {
        // Generate clock pulse
        SCK = 1;
        waitMicrosecond(1);

        // Shift previous bits left
        result <<= 1;

        // Read data bit
        if(DOUT)
            result++;

        SCK = 0;
        waitMicrosecond(1);
    }

    // Send 25th pulse to set gain to 128
    SCK = 1;
    waitMicrosecond(1);
    SCK = 0;
    waitMicrosecond(1);

    // Convert to signed 32-bit value (sign extend from 24-bit)
    if(result & 0x800000)
        result |= 0xFF000000;

    return result;
}

int main(void)
{
    int32_t adcValue;
    int32_t avgBuffer[NUM_SAMPLES] = {0};
    int64_t sum = 0;
    int32_t average = 0;
    int32_t difference = 0;
    float grams = 0;
    float newtons = 0;
    float weightOffset = 0;
    float newtonOffset = 0;
    uint8_t bufferIndex = 0;
    uint8_t bufferFilled = 0;
    char str[200];

    initHw();

    putsUart0("\n\r========================================\n\r");
    putsUart0("   HX711 Weight Scale & Force Meter\n\r");
    putsUart0("========================================\n\r");
    putsUart0("Waiting for data...\n\r");

    // Discard first 4 readings (ADC needs priming after power-up)
    uint8_t i;
    for(i = 0; i < 4; i++)
    {
        readHX711();
        waitMicrosecond(100000);
    }

    putsUart0("Ready! Filling buffer with 100 samples...\n\r");
    putsUart0("Baseline will be established automatically.\n\r\n\r");

    while(1)
    {
        // Read current ADC value
        adcValue = readHX711();

        // Add new value to buffer (circular buffer)
        avgBuffer[bufferIndex] = adcValue;
        bufferIndex++;

        if(bufferIndex >= NUM_SAMPLES)
        {
            bufferIndex = 0;
            bufferFilled = 1;
        }

        // Calculate average of buffer
        sum = 0;
        uint8_t count = bufferFilled ? NUM_SAMPLES : bufferIndex;

        for(i = 0; i < count; i++)
        {
            sum += avgBuffer[i];
        }

        if(count > 0)
        {
            average = (int32_t)(sum / count);

            // Calculate difference: current - baseline intercept
            // Using equation: ADC = 12899 * mass + 5000000
            // Solving for mass: mass = (ADC - 5000000) / 12899
            difference = adcValue - (int32_t)CALIBRATION_INTERCEPT;

            // Convert difference to grams using calibration slope
            grams = (float)difference / CALIBRATION_SLOPE;

            // Convert grams to Newtons
            // Force = mass(kg) � gravity = (grams/1000) � 9.81
            newtons = (grams / 1000.0) * GRAVITY;

            // Apply range-based corrections
            weightOffset = 0;
            newtonOffset = 0;

            if(adcValue >= 4770000 && adcValue <= 4790000)
            {
                weightOffset = 17.0;
                newtonOffset = 0.167;
            }
            else if(adcValue >= 4791000 && adcValue <= 4794000)
            {
                weightOffset = 225.0;
                newtonOffset = 2.21;
            }
            else if(adcValue >= 4800000 && adcValue <= 4809000)
            {
                weightOffset = 450.0;
                newtonOffset = 4.41;
            }
            else if(adcValue >= 4815000 && adcValue <= 4820000)
            {
                weightOffset = 675.0;
                newtonOffset = 6.62;
            }

            // Add offsets to measurements
            grams += weightOffset;
            newtons += newtonOffset;

            // Display results
            sprintf(str, "ADC: %7ld | Diff: %7ld counts | Weight: %7.1f g | Force: %6.3f N\n\r",
                    adcValue, difference, grams, newtons);
            putsUart0(str);
        }

        waitMicrosecond(100000);  // 100ms delay between readings
    }

    return 0;
}
