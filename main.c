/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Alan Barr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * See the README for more information on use.
 *
 * TODO:
 *  - Try to make it work with all compilers... (ISRs should be OK, but the SR?)
 *  - Cleanup
 *  - Consider removing looping with sleeps? 
 */

#include "msp430g2553.h"
#include "time430.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#define UART_BUF_SIZE       255

#define SPI_RX_BUF_SIZE     32
#define SPI_TX_BUF_SIZE     32

#define LED_RED             BIT0 /* P1.0 */

#define UART_BAUD           9600
#define UART_RX             BIT1 /* Don't connect, not used. */
#define UART_TX             BIT2 /* P1.2 */

#define SPI_CS              BIT4 /* P1.4 */
#define SPI_CLK             BIT5 /* P1.5 */
#define SPI_SOMI            BIT6 /* P1.6 */
#define SPI_SIMO            BIT7 /* P1.7 */

#define CC3000_EN           BIT0 /* P2.0 */
#define CC3000_IRQ          BIT1 /* P2.1 */
#define DEBUG_PIN           BIT2 /* P2.2 */

#define STEP_ERROR          255 /* Step value when error occurs */

/* List of potential errors that can be reported */
#define ERRORS              \
    ERROR(NONE)             \
    ERROR(EN_PIN_HIGH)      \
    ERROR(EN_PIN_LOW)       \
    ERROR(CS_PIN_LOW)       \
    ERROR(CS_PIN_HIGH)      \
    ERROR(WAIT_TIME)        \
    ERROR(SPI_DATA)         \
    ERROR(SPI_DATA_COUNT)   \
    ERROR(SPI_OVERRUN)      \
    ERROR(UNEXPECTED)

/* Changes the value of step unless step is set to STEP_ERROR.
 * Interrupt safe */
#define SET_STEP(VAL)               \
    do {                            \
        __disable_interrupt();      \
        if (step != STEP_ERROR)     \
        {                           \
            step = VAL;             \
        }                           \
        __enable_interrupt();       \
    } while(0)

/* Disables all of the interrupts that are being used for checking
 * the behaviour of the host. */
#define DISABLE_MONITORING_INTERRUPTS() \
    do {                                \
        P1IE &= ~SPI_CS;                \
        P2IE &= ~CC3000_EN;             \
        IE2 &= ~UCB0RXIE;               \
        IE2 &= ~UCB0TXIE;               \
    } while (0)

/* Wrapper for _REPORT_ERROR, providing line number */
#define REPORT_ERROR(TYPE) _REPORT_ERROR(TYPE,__LINE__)

/* Sets error variables, but doesn't lose previous error.
 * Interrupt safe */
#define _REPORT_ERROR(TYPE,LINE)            \
    do {                                    \
        __disable_interrupt();              \
        DISABLE_MONITORING_INTERRUPTS();    \
        if (step != STEP_ERROR)             \
        {                                   \
            P2OUT ^= DEBUG_PIN;             \
            errorStep = step;               \
            errorType = TYPE;               \
            errorLine = LINE;               \
            step = STEP_ERROR;              \
        }                                   \
        __enable_interrupt();               \
    } while(0)

/* Checks for and reports an error */
#define ERROR_CHECK()               \
    do {                            \
        if (step == STEP_ERROR)     \
        {                           \
            error();                \
        }                           \
    } while (0)

/* Checks for and reports an error that is likely due
 * to SPI RX */
#define ERROR_CHECK_SPI_RX()                \
do {                                        \
    if (step == STEP_ERROR)                 \
    {                                       \
        DISABLE_MONITORING_INTERRUPTS();    \
        printSpiRxInfo();                   \
        error();                            \
    }                                       \
} while (0);

#define SEND_NEWLINE() addStrToUartBuffer("\n\r")

/* Configure TA to error. 5000 = 5 seconds. If debugging, you can set this 
 * to zero. */
#define SETUP_TA_DONT_GET_STUCK()   setupTimerA(false, 5000, TIME430_MS_CYCLES)

/* enum of all potentential error values. */
typedef enum
{
#undef ERROR
#define ERROR(E) E,
    ERRORS
    ERROR_MAX
} eError;

/* array of strings for all potentential error values. */
static char * errorStrings[] =
{
#undef ERROR
#define ERROR(E) #E,
    ERRORS
    "ERROR_MAX"
};

volatile uint8_t step = 0;

volatile eError errorType = NONE; /* Type of error */
volatile uint16_t errorLine = 0;  /* Line error reported on */
volatile uint8_t errorStep = 0;   /* Step error occured on */

char uartTxBuffer[UART_BUF_SIZE];
volatile unsigned char uartTxBufferStart = 0;                 /* Always indexes first untransmitted element */
volatile unsigned char uartTxBufferEnd = UART_BUF_SIZE - 1;   /* Always indexes last received element. */
volatile unsigned char uartTxBufferElements = 0;              /* Total number of elements in the buffer */

volatile uint8_t spiRxBuffer[SPI_RX_BUF_SIZE];
volatile uint8_t spiRxBufferIndex;

volatile uint8_t spiTxBuffer[SPI_TX_BUF_SIZE];
volatile uint8_t spiTxBufferIndex;
volatile uint8_t spiTxBufferElements;

volatile bool timerADesired;    /* If false and timer A expires an error is raised */
volatile uint16_t timerATicks;  /* Number of times timer A will interrupt */


/* Sets Timer A to count up to <ccr0> <ticks> times. If <desired> is false, 
 * when we reach <ticks> times an error will be raised. Setting ticks to zero
 * is not treated as an error, and the function returns immediately.*/
void setupTimerA(bool desired, uint16_t ticks,
                 uint16_t ccr0)
{ 
    /* Do nothing. Might be useful for debugging, e.g. with GDB
     * on the host. */
    if (ticks == 0)
    {
        return;
    }

    timerADesired = desired;
    timerATicks = ticks;
    TACTL |= TACLR;
    TACCR0 = ccr0;
    TACTL = TASSEL_2;
    TACCTL0 = CCIE;
    TACTL |= MC_1;
}


/* Basic print function. Adds parameter <str> to the UART TX buffer and enables
 * transmission (enables interrupts in the process. It blocks and waits if it 
 * cannot add <str> to the TX buffer. */
static void addStrToUartBuffer(char * str)
{
    int remaining = strlen(str);

    while (remaining)
    {
        while ((UART_BUF_SIZE - uartTxBufferElements) == 0); /* No Free Space */

        /* Buffer end should always remain on last element */
        __disable_interrupt();
        uartTxBufferEnd++;
        __enable_interrupt();

        if (uartTxBufferEnd == (UART_BUF_SIZE))
        {
            uartTxBufferEnd = 0;
        }

        uartTxBuffer[uartTxBufferEnd] = *(str++);

        __disable_interrupt();
        uartTxBufferElements++;

        IE2 |= UCA0TXIE;
        __enable_interrupt();

        remaining--;
    }
}


/* Converts parameter <number> to a  string (hexadecimal representation) and
 * transmits it over UART */
static void addHexToUartBuffer(uint16_t number)
{
    char hexStr[7];  /* "0x" +  nibbles in uint16_t + NULL bytes */
    uint8_t i;

    hexStr[0] = '0';
    hexStr[1] = 'x';

    for (i=5; i>=2; i--)
    {
        hexStr[i] = number & 0x000F;
        number = number >> 4;

        if (hexStr[i] > 9)
        {
            hexStr[i] += 55; /* 10 == A */
        }
        else
        {
            hexStr[i] += 48;
        }
    }

    hexStr[6] = 0;

    addStrToUartBuffer(hexStr);
}


/* Converts parameter <number> to a string (decimal representation) and 
 * transmits it over UART */
static void addDecToUartBuffer(uint16_t number)
{
    char decStr[6];  /*  65535 + NULL bytes */
    uint8_t i;
    uint8_t start = 0;

    memset(decStr,'0' ,sizeof(decStr));

    if (number == 0)
    {
        start = 4;
    }

    else
    {
        for (i=4; i>=0; i--)
        {
            decStr[i] = number % 10;
            number = number / 10;
            decStr[i] += 48;

            if (number == 0)
            {
                start = i;
                break;
            }
        }
    }

    decStr[5] = 0;

    addStrToUartBuffer(&decStr[start]);
}


/* Sends useful SPI RX information over UART */
static void printSpiRxInfo(void)
{
    int i;

    SEND_NEWLINE();
    addStrToUartBuffer("--ERROR MSG--");
    SEND_NEWLINE();
    addStrToUartBuffer("SPI RX bytes received: ");
    addDecToUartBuffer(spiRxBufferIndex);
    SEND_NEWLINE();
    addStrToUartBuffer("SPI RX Bytes were: ");

    for (i=0; i<=spiRxBufferIndex - 1; i++)
    {
        addHexToUartBuffer(spiRxBuffer[i]);
        SEND_NEWLINE();
    }
}


/* Reports error over UART. Doesn't return. */
static void error(void)
{
    /* Disable all interrupts - except UART */
    P1IE = 0x00;
    P2IE = 0x00;
    UCB0CTL1 |= UCSWRST;

    SEND_NEWLINE();
    addStrToUartBuffer("--STEP INFO--");
    SEND_NEWLINE();
    addStrToUartBuffer("STEP: ");
    addDecToUartBuffer(errorStep);
    SEND_NEWLINE();
    addStrToUartBuffer("ERROR TYPE: ");
    addStrToUartBuffer(errorStrings[errorType]);
    SEND_NEWLINE();
    addStrToUartBuffer("ERROR LINE: ");
    addDecToUartBuffer(errorLine);
    SEND_NEWLINE();

    while(1)
    {
        /* Sit Here Forever */
        __bis_status_register(LPM0_bits);
    }
}


int main(void)
{
    uint8_t startStateCorrectCtr = 0;
    uint16_t startStateDebugCtr = 0;
    WDTCTL = WDTPW | WDTHOLD;

    TIME430_CALIBRATE_CLOCK();

    /* Setup LEDS */
    P1DIR = LED_RED;
    P1OUT = ~LED_RED;

    /* Setup UART */
    UCA0CTL1 |= UCSWRST;
    P1SEL |= UART_TX;
    P1SEL2 |= UART_TX;
    UCA0CTL1 |= UCSSEL_2;
    UCA0BR0 = 0xFF & (unsigned short)(TIME430_CLOCK_HZ / UART_BAUD);
    UCA0BR1 = 0xFF & ((unsigned short)(TIME430_CLOCK_HZ / UART_BAUD) >> 8);
    UCA0MCTL = UCBRS0;
    UCA0CTL1 &= ~UCSWRST;

    /* Initial Setup input pins */
    P1DIR &= ~SPI_CS;
    P1DIR &= ~SPI_CLK;
    P2DIR &= ~CC3000_EN;

    /* Initial Setup output pins */
    P2DIR |= CC3000_IRQ;
    P2DIR |= DEBUG_PIN;
    P2OUT &= ~ DEBUG_PIN;

    /* Default the IRQ line to inactive/high */
    P2OUT |= CC3000_IRQ;

    SEND_NEWLINE();
    SEND_NEWLINE();
    SEND_NEWLINE();
    SEND_NEWLINE();

    addStrToUartBuffer("Waiting for idle setup...");

    /* Step one check - We wait until pins have remained at their
       expected values for 50 ms
     * (EN low, CS High, CLK Low) */
    while (1)
    {
        if (((P2IN & CC3000_EN) == 0) &&
            ((P1IN & SPI_CLK) == 0) &&
            ((P1IN & SPI_CS) == SPI_CS))
        {
            startStateCorrectCtr++;
        }

        if (startStateCorrectCtr == 50)
        {
            break;
        }
        TIME430_DELAY_MS(1);

        startStateDebugCtr++;

        if (startStateDebugCtr == 2000)
        {
            startStateDebugCtr = 0;
            addStrToUartBuffer("Pins are:");
            SEND_NEWLINE();
            addStrToUartBuffer("CC3000_EN: ");
            addDecToUartBuffer((P2IN & CC3000_EN) ? 1:0);
            SEND_NEWLINE();
            addStrToUartBuffer("SPI_CLK: ");
            addDecToUartBuffer((P1IN & SPI_CLK) ? 1:0);
            SEND_NEWLINE();
            addStrToUartBuffer("SPI_CS: ");
            addDecToUartBuffer((P1IN & SPI_CS) ? 1:0);
            SEND_NEWLINE();
        }
    }

    SET_STEP(1);

    addStrToUartBuffer("idle Setup OK.");
    SEND_NEWLINE();

    /* Finish Configuring SPI_CS - we monitor this */
    P1IFG &= ~ SPI_CS;
    P1IES |= SPI_CS; /* High to low */
    P1IE |= SPI_CS;

    /* Finish Configuring CC3000_EN */
    P2IFG &= ~ CC3000_EN;
    P2IE |= CC3000_EN;
    P2IES &= ~CC3000_EN; /* Low to high */

    /* Setup SPI */
    UCB0CTL1 |= UCSWRST;
    P1SEL |= SPI_SOMI |SPI_SIMO | SPI_CLK;
    P1SEL2 |= SPI_SOMI | SPI_SIMO | SPI_CLK;
    UCB0CTL0 = UCSYNC;
    UCB0CTL0 |= UCMSB;
    UCB0CTL1 &= ~UCSWRST; /* Take it out of sw reset */

    /* Setup Interrupts */
    __enable_interrupt();

    while (1)
    {
        if (step == 2)
        {
            SET_STEP(3);
            P2OUT &= ~CC3000_IRQ;
        }

        else if (step == 4 || step == 6)
        {
            /* Setup Timer A for 50 us sleep*/
            setupTimerA(true, 1, 50 * TIME430_US_CYCLES);
            __bis_status_register(LPM0_bits);

            if (step == 4)
            {
                SET_STEP(5);
            }
            else if (step == 6)
            {
                SET_STEP(7);
            }
        }

        else if (step == 5)
        {
            /* Setup Timer A to prevent getting stuck */
            SETUP_TA_DONT_GET_STUCK();

            while (spiRxBufferIndex != 4 && step != STEP_ERROR);

            /* Ensure Timer A has stopped */
            TACTL |= TACLR;

            if (spiRxBuffer[0] == 0x01 &&
                spiRxBuffer[1] == 0x00 &&
                spiRxBuffer[2] == 0x05 &&
                spiRxBuffer[3] == 0x00)
            {
                SET_STEP(6);
            }

            else
            {
                REPORT_ERROR(SPI_DATA);
                ERROR_CHECK_SPI_RX();
            }
        }

        else if (step == 7)
        {
            /* Setup Timer A to prevent getting stuck */
            SETUP_TA_DONT_GET_STUCK();

            while (spiRxBufferIndex != 10 && step != STEP_ERROR);

            /* Ensure Timer A has stopped */
            TACTL |= TACLR;

            ERROR_CHECK_SPI_RX();

            /* CS is very likely to go high at this point */

            if (spiRxBuffer[4] == 0x00 &&
                spiRxBuffer[5] == 0x01 &&
                spiRxBuffer[6] == 0x00 &&
                spiRxBuffer[7] == 0x40 &&
                spiRxBuffer[8] == 0x01 &&
                spiRxBuffer[9] == 0x00)
            {
                /* Setup Timer A to prevent getting stuck */
                SETUP_TA_DONT_GET_STUCK();
                SET_STEP(8);

                /* manually checking CS here, as the interrupt could
                   have happened at either */
                while ((P1IN & SPI_CS) == 0 && step != STEP_ERROR);

                /* Ensure that timer A has stopped */
                TACTL |= TACLR;

                SET_STEP(9);
            }
            else
            {
                REPORT_ERROR(SPI_DATA);
                ERROR_CHECK_SPI_RX();
            }
        }

        else if (step == 9)
        {
            P2OUT |= CC3000_IRQ;
            addStrToUartBuffer("Received wlan_start(0): "
                               "HCI_CMND_SIMPLE_LINK_START.");
            SEND_NEWLINE();
            break;
        }

        else if (step == STEP_ERROR)
        {
            error();
        }
    }

    TIME430_DELAY_MS(2);
    SET_STEP(10);

    /* prepare the tx buffer */
    spiTxBuffer[0] = 0x02;
    spiTxBuffer[1] = 0x00;
    spiTxBuffer[2] = 0x00;
    spiTxBuffer[3] = 0x00;
    spiTxBuffer[4] = 0x05;
    spiTxBuffer[5] = 0x04;
    spiTxBuffer[6] = 0x00;
    spiTxBuffer[7] = 0x40;
    spiTxBuffer[8] = 0x01;
    spiTxBuffer[9] = 0x00;

    spiTxBufferIndex = 0;
    spiTxBufferElements = 10;

    IE2 &= ~UCB0RXIE;
    IE2 |= UCB0TXIE;
    P2OUT &= ~CC3000_IRQ;
    P2OUT ^= DEBUG_PIN;

    while (1)
    {
        if (step == 11)
        {
            SET_STEP(12);
            /* Hang about here, waiting for TX to happen. */

            /* Setup Timer A to prevent getting stuck */
            SETUP_TA_DONT_GET_STUCK();
            while(spiTxBufferElements && step != STEP_ERROR);

            /* Ensure that timer A has stopped */
            TACTL |= TACLR;
        }

        else if (step == 13)
        {
            SET_STEP(14);
            //P2OUT |= CC3000_IRQ; //XXX moved to irq for now...

            addStrToUartBuffer("Replied to wlan_start(0): "
                                "HCI_CMND_SIMPLE_LINK_START.");
            SEND_NEWLINE();
            break;
        }

        else if (step == STEP_ERROR)
        {
            error();
        }

    }

    while (1)
    {
        if (step == 15)
        {
            TIME430_DELAY_MS(1); /* ensure we dont get confused by irq */
            spiRxBufferIndex = 0;

            SET_STEP(16);
            IE2 |= UCB0RXIE;

            /* TODO well get an interrupt firing.... quick fix */
            while (spiRxBufferIndex != 1);
            spiRxBufferIndex = 0;

            P2OUT &= ~CC3000_IRQ;
        }

        else if (step == 16)
        {
            /* Setup Timer A to prevent getting stuck */
            SETUP_TA_DONT_GET_STUCK();

            while (spiRxBufferIndex != 10 &&
                   step != STEP_ERROR);

            /* Ensure that timer A has stopped */
            TACTL |= TACLR;

            /* Did we get more than 10 bytes */
            TIME430_DELAY_MS(1);
            if (spiRxBufferIndex != 10)
            {
                REPORT_ERROR(SPI_DATA_COUNT);
            }

            /* Again, CS is likely to go high at this point */

            ERROR_CHECK_SPI_RX();

            if (spiRxBuffer[0] == 0x01 &&
                spiRxBuffer[1] == 0x00 &&
                spiRxBuffer[2] == 0x05 &&
                spiRxBuffer[3] == 0x00 &&
                spiRxBuffer[4] == 0x00 &&
                spiRxBuffer[5] == 0x01 &&
                spiRxBuffer[6] == 0x0b &&
                spiRxBuffer[7] == 0x40 &&
                spiRxBuffer[8] == 0x00 &&
                spiRxBuffer[9] == 0x00 )
            {
                SET_STEP(17);
                /* TODO need a timer here ? */
                while ((P1IN & SPI_CS) == 0);
                SET_STEP(18);
            }

            else
            {
                REPORT_ERROR(SPI_DATA);
                ERROR_CHECK_SPI_RX();;
            }
        }
        else if (step == 18)
        {
            SET_STEP(19);
            P2OUT |= CC3000_IRQ;

            addStrToUartBuffer("Received wlan_start(0): "
                               "HCI_CMND_READ_BUFFER_SIZE.");
            SEND_NEWLINE();
            break;
        }

        else if (step == STEP_ERROR)
        {
            error();
        }
    }

    while (1)
    {
        if (step == 19)
        {
            SET_STEP(20);

            /* prepare the tx buffer */
            spiTxBuffer[0]  = 0x02;
            spiTxBuffer[1]  = 0x00;
            spiTxBuffer[2]  = 0x00;
            spiTxBuffer[3]  = 0x00;
            spiTxBuffer[4]  = 0x09;
            spiTxBuffer[5]  = 0x04;
            spiTxBuffer[6]  = 0x0b;
            spiTxBuffer[7]  = 0x40;
            spiTxBuffer[8]  = 0x04;
            spiTxBuffer[9]  = 0x00;
            spiTxBuffer[10] = 0x06;
            spiTxBuffer[11] = 0xdc;
            spiTxBuffer[12] = 0x05;
            spiTxBuffer[13] = 0x00;

            spiTxBufferIndex = 0;
            spiTxBufferElements = 10;

            IE2 &= ~UCB0RXIE;
            IE2 |= UCB0TXIE;

            P2OUT &= ~CC3000_IRQ;
        }

        else if (step == 21)
        {
            SET_STEP(22);
            while(spiTxBufferElements);
        }

        else if (step == 23)
        {
            DISABLE_MONITORING_INTERRUPTS();

            P2OUT |= CC3000_IRQ;

            SET_STEP(24);

            addStrToUartBuffer("Replied to wlan_start(0): "
                               "HCI_CMND_READ_BUFFER_SIZE.");
            SEND_NEWLINE();
            addStrToUartBuffer("Thats all folks...");
            SEND_NEWLINE();
            while (1);
        }

        else if (step == STEP_ERROR)
        {
            error();
        }

    }
    return 0;
}


/* Interrupt handler for pin interrupts on port 1. */
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    if (P1IFG & SPI_CS)
    {
        P1IFG &= ~SPI_CS;

        if (P1IES & SPI_CS)   /* High to Low Caused this */
        {
            if (step == 3)
            {
                step = 4;
                IE2 |= UCB0RXIE; /* Enable interrupts on SPI RX */
            }

            else if (step == 10)
            {
                step = 11;
            }

            else if (step == 20)
            {
                step = 21;
            }

            else if (step == 14)
            {
                step = 15;
            }

            else
            {
                REPORT_ERROR(CS_PIN_LOW);
            }
        }

        else            /* Low to high */
        {
            if (step == 7 || step == 8)
            {
                /* We don't do anything here - since it gets messy.
                 * Typically CS goes high when we are on step 7 - i.e.
                 * SPI finishes but we can't check the bytes in time. */
            }

            else if (step == 12)
            {
                step = 13;

                /* TODO why can't this be in the step ... messed up timing
                 * somewhere? */
                P2OUT |= CC3000_IRQ;
            }

            else if (step == 16 || step == 17)
            {
                /* Handled by polling */
            }

            else if (step == 22)
            {
                step = 23;
            }

            else
            {
                REPORT_ERROR(CS_PIN_HIGH);
            }
        }

        P1IES ^= SPI_CS; /* Toggle Edge Interrupt */
    }

    /* Alan TODO - ensure any low power mode left */
    __bic_status_register_on_exit(LPM4_bits);
}


/* Interrupt handler for pin interrupts on port 2. */
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    if (P2IFG & CC3000_EN)
    {
        P2IFG &= ~CC3000_EN;

        if ((P2IES & CC3000_EN) == 0) /* If low to high - enabled */
        {
            if (step == 1)
            {
                step = 2;
            }
            else
            {
                REPORT_ERROR(EN_PIN_HIGH);
            }
        }
        else /* high to low - off */
        {
/* TODO - maybe if EN falls do a SW reset? This will require more startup
 * code though to put registers not affected by the PUC into their desired
 * state*/
            REPORT_ERROR(EN_PIN_LOW);
        }

        P2IES ^= CC3000_EN; /* Toggle Edge Interrupt */
    }

    /*Ensure any low power mode left */
    __bic_status_register_on_exit(LPM4_bits);
}


/* Interrupt handler for transmit activity on USCI A and B. */
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI_TX_ISR(void)
{
    if (IFG2 & UCA0TXIFG)
    {
        if (uartTxBufferElements)
        {
            UCA0TXBUF = uartTxBuffer[uartTxBufferStart];

            uartTxBufferStart++;

            if (uartTxBufferStart == UART_BUF_SIZE)
            {
                uartTxBufferStart = 0;
            }
            uartTxBufferElements--;
        }
        else
        {
            IE2 &= ~UCA0TXIE;
        }
    }

    else if (IFG2 & UCB0TXIFG)
    {
        if (spiTxBufferElements)
        {
            UCB0TXBUF = spiTxBuffer[spiTxBufferIndex];
            spiTxBufferIndex++;
            spiTxBufferElements--;
        }
        else
        {
            IE2 &= ~UCB0TXIE;
        }
    }
}


/* Interrupt handler for receive activity on USCI A and B. */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI_RX_ISR(void)
{
    if (IFG2 & UCB0RXIFG)
    {
        spiRxBuffer[spiRxBufferIndex++] = UCB0RXBUF;
        if (spiRxBufferIndex >= SPI_RX_BUF_SIZE)
        {
            IE2 &= ~UCB0RXIE;
            REPORT_ERROR(SPI_OVERRUN);
        }
    }
    else
    {
        REPORT_ERROR(UNEXPECTED);
    }

    /* Ensure any low power mode left */
    __bic_status_register_on_exit(LPM4_bits);
}


/* Interrupt handler for Timer A.*/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER_A_ISR(void)
{
    timerATicks--;

    if (timerATicks == 0)
    {
        TACTL = TACLR;

        if (timerADesired == false)
        {
            REPORT_ERROR(WAIT_TIME);
        }

        /* Ensure any low power mode left */
        __bic_status_register_on_exit(LPM4_bits);
    }
}

