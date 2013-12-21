/*
 * File:   main.c
 * Author: tylerjw
 *
 * Created on December 19, 2013, 11:59 AM
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright � 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/


#include <p32xxxx.h>
#include <plib.h>
#include <stdio.h>

#define SYS_CLK 100000000L // 100MHz

//	Function Prototypes
int main(void);
void delay(volatile unsigned int count);
int U1_write(const char *buffer);
unsigned int getBRG(unsigned int pb_clock, int baud);
float U1_init(unsigned int pb_clock, int baud);
unsigned int U1_read(char *buffer, unsigned int max_size);

int main(void) {
    char buffer[80];
    unsigned int pb_clock;
    float actual_baud;
    const int baud = 9600;

    pb_clock = SYSTEMConfigPerformance(SYS_CLK); // if sys_clock > 100MHz, pb_clock = sys_clock/2 else pb_clock = sys_clock

    PORTSetPinsDigitalOut(IOPORT_E, BIT_4); // led

    REFOCON = BIT_17 | BIT_15 | BIT_12 | BIT_0; // REFCLKO output EN, PBCLK
    REFOTRIM = 0; // reference trim

    // setup UART
    PPSUnLock;
    PPSInput(1,U1RX,RPF4); // Rx - F4 (pin 49) 5V tolerent
    PPSOutput(2,RPF5,U1TX); // Tx - F5 (pin 50) 5V tolerent
    PPSOutput(3,RPE3,REFCLKO); // REFCLO - E3 (pin 99)
    PPSLock;

    
    actual_baud = U1_init(pb_clock, baud);  // Baud rate of 9600

    sprintf(buffer, "SYSCLK: %d\r\n", SYS_CLK);
    U1_write(buffer);
    sprintf(buffer, "PBCLK: %d\r\n", pb_clock);
    U1_write(buffer);
    sprintf(buffer, "U1BRG: %d\r\n", U1BRG);
    U1_write(buffer);
    sprintf(buffer, "target baud: %d\r\n", baud);
    U1_write(buffer);
    sprintf(buffer, "actual baud: %f\r\n", actual_baud);
    U1_write(buffer);

    while (1) {
        mPORTEWrite(0);
        U1_write("Hello World!\r\n");
        delay(SYS_CLK/4);
        mPORTEWrite(BIT_4);
        delay(SYS_CLK/4);
    }
}

float U1_init(unsigned int pb_clock, int baud)
{
    float actual_baud;

    U1MODE = BIT_15 | BIT_3; // UART1 EN | 4x multiplier
    U1STA |= BIT_12 | BIT_10; // EN RX and TX
    U1BRG = (pb_clock / (4 * baud)) - 1;
    actual_baud = (float)pb_clock / (4 * (U1BRG + 1));

    return actual_baud;
}

unsigned int getBRG(unsigned int pb_clock, int baud)
{
    return (pb_clock / (4 * baud)) - 1;
}

void delay(volatile unsigned int count)
{
    while(--count);
}

/* U1_write() transmits a string to the UART2 TX pin MSB first
 *
 * Inputs: *buffer = string to transmit */
int U1_write(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while( size)
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer

        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }

    while( !U1STAbits.TRMT);        // wait for last transmission to finish

    return 0;
}

/* U1_read() is a blocking function that waits for data on
 *  the UART1 RX buffer and then stores all incoming data into *buffer
 *
 * Note that when a carriage return '\r' is received, a nul character
 *  is appended signifying the strings end
 *
 * Inputs:  *buffer = Character array/pointer to store received data into
 *          max_size = number of bytes allocated to this pointer
 * Outputs: Number of characters received */
unsigned int U1_read(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;

    /* Wait for and store incoming data until either a carriage return is received
     *   or the number of received characters (num_chars) exceeds max_size */
    while(num_char < max_size)
    {
        while( !U1STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer

        // insert nul character to indicate end of string
        if( *buffer == '\r'){
            *buffer = '\0';
            break;
        }

        buffer++;
        num_char++;
    }

    return num_char;
} // END SerialReceive()