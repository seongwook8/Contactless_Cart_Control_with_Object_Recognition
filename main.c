#include "msp.h"
#include "driverlib.h"
#include <stdlib.h>
#include <stdio.h>

volatile char rx = 0; // buffer with 8 bits
volatile char a = (char) 0x41;
volatile char b = (char) 0x42;
volatile char c = (char) 0x43;
volatile char d = (char) 0x44;
volatile int state = 1;
volatile int beep = 0;
volatile float DC = 12000;
volatile float keepd = 123;
volatile float dist;
volatile float Kp = 0.03;
volatile float Kd = .2;
volatile float speed;
volatile float pos;
volatile float lastPos;


Timer_A_PWMConfig pwmConfig1 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        30000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
};

Timer_A_PWMConfig pwmConfig2 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        30000,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
};

/**
 * main.c
 */
void main(void) {
    // calculated for baud rate of 9600
    const eUSCI_UART_Config UART_init =
    {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
            19,                                     // BRDIV = 19
            8,                                       // UCxBRF = 8
            85,                                       // UCxBRS = 85
            EUSCI_A_UART_NO_PARITY,                  // No Parity
            EUSCI_A_UART_LSB_FIRST,                  // LSB First
            EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
            EUSCI_A_UART_MODE,                       // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
    };

    WDT_A_holdTimer(); // stop watchdog timer

    // enable FPU
    FPU_enableModule();

    // Set clock frequency (refer to Lab 4 manual)
    unsigned int dcoFrequency = 3E+6;
    CS_setDCOFrequency(dcoFrequency);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Define GPIOs and UART TX/RX (P1.2 & P1.3)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2|GPIO_PIN3, GPIO_TERTIARY_MODULE_FUNCTION);

    // Define GPIOs for LEDs and Buzzer
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);

    /* Configuring PWM outputs */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring P2.4 as peripheral input for capture */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);

    /* Configuring Button1 & Button2 */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    /* Enabling interrupts */
    Interrupt_enableInterrupt(INT_PORT1);

    // Initialize UART x
    UART_initModule(EUSCI_A0_BASE, &UART_init);
    UART_enableModule(EUSCI_A0_BASE);

    //Enable UART receive interrupts
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    Interrupt_disableMaster();
    // Also enable interrupt master...
    Interrupt_enableInterrupt(INT_EUSCIA0);
    UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableMaster();

    /* Starting the Timer32 */
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_disableInterrupt(TIMER32_0_BASE);
    Timer32_setCount(TIMER32_0_BASE, 1);
    Timer32_startTimer(TIMER32_0_BASE, true);

    /* Starting the Timer_A0 in continuous mode */
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);

    // This is where you TX through your eUSCI UART module
    while(1) {
    // If ready, transmit data in buffer until buffer is empty.
        Interrupt_disableInterrupt(INT_EUSCIA0);
        while((UCA0IFG & 0x02 == 0)) {
        }
        Interrupt_enableInterrupt(INT_EUSCIA0);

    } // end while
}

void EUSCIA0_IRQHandler(void) {
    // Read the RX buffer also resets the receive interrupt flag
    // If character represented in buffer is certain type (end condition)... // Stop reading, set some flag to signal start of echo transmission
    // Else, keeping reading buffer until done.

    // save input as rx
    rx = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
    // if the input is 'return'
    if (rx == a || rx == b || rx == c) {
        int cnt = 0;
        if (rx == a) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        } else if (rx == b) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        } else {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        }
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
        Timer32_setCount(TIMER32_0_BASE, 24 * 10);
        while (Timer32_getValue(TIMER32_0_BASE) > 0); // Wait 10us
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
        while (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == 0) {
        }
        while (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6)) {
            cnt = cnt + 1;
        }

        dist = ((float)cnt);
        pos = dist - keepd;

        speed = (pos - lastPos)/10;
        lastPos = pos;

        float output = Kp*pos - Kd*speed;

        if (output > .8) {
            DC = 10000 + 700*output;
            pwmConfig1.dutyCycle = 0;
            pwmConfig2.dutyCycle = DC;
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
            beep = 0;
        } else if (output < -.8) {
            DC = 10000 + 700*output*-1+1000;
            pwmConfig1.dutyCycle = DC;
            pwmConfig2.dutyCycle = 0;
            if ((beep/5)%2 == 0) {
                GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
            } else {
                GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
            }
            beep = beep + 1;
        } else {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
            pwmConfig2.dutyCycle = 0;
            pwmConfig1.dutyCycle = 0;
            beep = 0;
        }

    } else { // if the input is not 'return' save the data in the buffer
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
        pwmConfig2.dutyCycle = 0;
        pwmConfig1.dutyCycle = 0;
    }
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);

    UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

void PORT1_IRQHandler(void)
{
    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1) // Button 1 Pressed
    {
        if(state == 0)
        {
            keepd = 123;
            state = 1;
        } else if (state == 1) {
            keepd = 183;
            state = 2;
        } else {
            keepd = 63;
            state = 0;
        }

    }

    if (status & GPIO_PIN4) // Button 2 Pressed
    {
        if(state == 0)
        {
            keepd = 183;
            state = 2;
        } else if (state == 1) {
            keepd = 63;
            state = 0;
        } else {
            keepd = 123;
            state = 1;
        }
    }

}
