/* 
 * File:   yesgoodfinalprojectpleasework.c
 * Author: 1407kpeavot
 *
 * Created on 22 March 2017, 11:36
 */
#define _XTAL_FREQ 4000000 //4 Million Hz, T = 0.25 us

//Headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <xc.h>
#include <pic18f2550.h>


// PIC18F2550 Configuration Bit Settings

// 'C' source line config statements

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = XT_XT     // Oscillator Selection bits (XT oscillator (XT))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#define DATA PORTAbits.RA0
#define LOAD PORTAbits.RA1
#define CLK  PORTAbits.RA2


unsigned long tickCount;


float frequencyList[66] = {
    {16.35}, //C
    {17.32}, //C#
    {18.35}, //D
    {19.45}, //D#
    {20.60}, //E
    {21.83}, //F
    {23.12}, //F#
    {24.50}, //G
    {25.96}, //G#
    {27.50}, //A
    {29.14}, //A#
    {30.87}, //B
    {32.70}, //C
    {34.64}, //C#
    {36.70}, //D
    {38.90}, //D#
    {41.20}, //E
    {43.66}, //F
    {46.24}, //F#
    {49.00}, //G
    {51.92}, //G#
    {55.00}, //A
    {58.28}, //A#
    {61.74}, //B
    {65.40}, //C
    {69.28}, //C#
    {73.40}, //D
    {77.80}, //D#
    {82.40}, //E
    {87.32}, //F
    {92.48}, //F#
    {98.00}, //G
    {103.84}, //G#
    {110.00}, //A
    {116.56}, //A#
    {123.48}, //B
    {130.80}, //C
    {138.56}, //C#
    {146.80}, //D
    {155.60}, //D#
    {164.80}, //E
    {174.64}, //F
    {184.96}, //F#
    {196.00}, //G
    {207.68}, //G#
    {220.00}, //A
    {233.12}, //A#
    {246.96}, //B
    {261.60}, //C
    {277.12}, //C#
    {293.60}, //D
    {311.12}, //D#
    {329.60}, //E
    {349.28}, //F
    {369.92}, //F#
    {392.00}, //G
    {415.36}, //G#
    {440.00}, //A
    {466.24}, //A#
    {493.92}, //B
    {523.20}, //C
    {554.24}, //C#
    {587.20}, //D
    {622.40}, //D#
};

const unsigned char blank[8] = {//For if the second display doesn't need lighting up
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
};
const unsigned char noteA[8] = {//A
    0b00111100,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01111110,
    0b01000010,
    0b01000010,
    0b01000010
};
const unsigned char noteB[8] = {//B
    0b01111100,
    0b01000010,
    0b01000010,
    0b01111100,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01111100
};
const unsigned char noteC[8] = {//C
    0b00111110,
    0b01000000,
    0b01000000,
    0b01000000,
    0b01000000,
    0b01000000,
    0b01000000,
    0b00111110
};
const unsigned char noteD[8] = {//D
    0b01111100,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01111100
};
const unsigned char noteE[8] = {//E
    0b01111110,
    0b01000000,
    0b01000000,
    0b01111000,
    0b01000000,
    0b01000000,
    0b01000000,
    0b01111110
};
const unsigned char noteF[8] = {//F
    0b01111110,
    0b01000000,
    0b01000000,
    0b01111000,
    0b01000000,
    0b01000000,
    0b01000000,
    0b01000000
};
const unsigned char noteG[8] = {//G
    0b00111100,
    0b01000010,
    0b01000000,
    0b01000000,
    0b01001110,
    0b01000010,
    0b01000010,
    0b00111100
};
const unsigned char noteSharp[8] = {//#
    0b00100100,
    0b00100100,
    0b11111111,
    0b00100100,
    0b00100100,
    0b11111111,
    0b00100100,
    0b00100100
};
const unsigned char noteFlat[8] = {//b
    0b00100000,
    0b00100000,
    0b00100000,
    0b00100000,
    0b00111000,
    0b00100100,
    0b00100100,
    0b00111000,
};

void storeImageD1(const unsigned char image[8], unsigned char firstDisplay[8]) //Stores an [8] array in the value 'firstDisplay'
{
    for (unsigned char i = 0; i < 8; i++)
        firstDisplay[i] = image[i];
}

void storeImageD2(const unsigned char image[8], unsigned char secondDisplay[8]) //Stores an [8] array in the value 'firstDisplay'
{
    for (unsigned char i = 0; i < 8; i++)
        secondDisplay[i] = image[i];
}

void sendData(unsigned char data) {
    for (int i = 0; i < 8; i++, data <<= 1) {
        RA2 = 0; //RA2 = CLK
        if (data & 0b10000000)
            RA0 = 1; //RA0 = DIN
        else
            RA0 = 0; //RA0 = DIN
        //__delay_us(5);
        RA2 = 1; //RA2 = CLK
        __delay_us(5);
    }
    RA2 = 0; //RA2 = CLK
}

void max7219_write(unsigned char mostSignificantByte, unsigned char leastSignificantByte) {
    sendData(mostSignificantByte);
    sendData(leastSignificantByte);
}

void update_Displays(unsigned char firstDisplay[8], unsigned char secondDisplay[8]) {
    for (char i = 0; i < 8; i++) {
        LOAD = 0;
        max7219_write(i + 1, secondDisplay[i]);
        max7219_write(i + 1, firstDisplay[i]);
        LOAD = 1;
        __delay_us(5);
        // __delay_ms(25);
        LOAD = 0; //LOAD = LOAD
    }
}

void init_PIC18F2550(void) {
    UCONbits.USBEN = 0; //disable USB module
    UCFGbits.UTRDIS = 1; //disable USB transceiver
    ADCON1 = 0x0F; //all digital i/o
    TRISA = 0b00010000; //
    TRISB = 0x00; //
    TRISC = 0x00; //

    T0CON = 0x88; // enable Timer0 in 16 bit mode
    TMR0IE = 1; // enable interrupts on timer0 overflow
}

void max7219_init(int noOfDisplays) {
    // Decode mode: none
    LOAD = 0;
    for (int i = 0; i < noOfDisplays; i++)
        max7219_write(0x09, 0);
    //__delay_ms(25);
    LOAD = 1;
    __delay_us(5);
    // Intensity: 3 (0-15)
    LOAD = 0;
    for (int i = 0; i < noOfDisplays; i++)
        max7219_write(0x0A, 0x0F);
    //__delay_ms(25);
    LOAD = 1;
    __delay_us(5);
    // Scan limit: All "digits" (rows) on
    LOAD = 0;
    for (int i = 0; i < noOfDisplays; i++)
        max7219_write(0x0B, 7);
    //__delay_ms(25);
    LOAD = 1;
    __delay_us(5);
    // Shutdown register: firstDisplay on
    LOAD = 0;
    for (int i = 0; i < noOfDisplays; i++)
        max7219_write(0x0C, 1);
    //__delay_ms(25);
    LOAD = 1;
    __delay_us(5);
    // firstDisplay test: on
    LOAD = 0;
    for (int i = 0; i < noOfDisplays; i++)
        max7219_write(0x0F, 1);
    //__delay_ms(100);
    LOAD = 1;
    __delay_us(5);
    // firstDisplay test off
    LOAD = 0;
    for (int i = 0; i < noOfDisplays; i++)
        max7219_write(0x0F, 0);
    //    max7219_clear();
    //__delay_ms(25);
    LOAD = 1;
    __delay_us(5);
}

void interrupt tc_int(void) {
    if (TMR0IE && TMR0IF) {
        TMR0IF = 0;
        ++tickCount; //increment global variable each time timer0 overflows
        return;
    }
    // process other interrupt sources here, if required
}

unsigned long getTimeMicroSec(void) {

    //   unsigned char LSB;
    //   unsigned char MSB;
    unsigned long temp;
    temp = (tickCount << 16 | TMR0);
    //   temp = tickCount*65536 + ((TMR0H << 8)|TMR0L);
    return temp;
}

void findTargetFreqs(float frequency, unsigned char firstDisplay[8], unsigned char secondDisplay[8], int targetFreqSharp, int targetFreqFlat) {
    int found = 0;
    int count = 0;
    while (found != 1) {
        if (frequency > frequencyList[65]) {
            storeImageD1(noteSharp, firstDisplay);
            storeImageD2(noteSharp, secondDisplay);
            update_Displays(firstDisplay, secondDisplay);
            break;
        } else if (frequency < frequencyList[0]) {
            storeImageD1(noteFlat, firstDisplay);
            storeImageD2(noteFlat, secondDisplay);
            update_Displays(firstDisplay, secondDisplay);
            break;
        } else {

            if ((count + 1) <= 65) {

                if (frequencyList[count] == frequency) {
                    targetFreqSharp = count;
                    targetFreqFlat = count;
                    found = 1;
                } else if ((frequency > frequencyList[count]) && (frequency < frequencyList[count + 1])) {
                    targetFreqSharp = count + 1;
                    targetFreqFlat = count;
                    found = 1;
                }
                count++;

            } else if ((count + 1) > 65) {

                if (frequencyList[count] == frequency) {
                    targetFreqSharp = count;
                    targetFreqFlat = count;
                    found = 1;
                } else if ((frequency > frequencyList[count - 1]) && (frequency < frequencyList[count])) {
                    targetFreqSharp = count;
                    targetFreqFlat = count - 1;
                    found = 1;
                }
                count++;

            }
        }
    }
}

void displayOutOfTune(float frequency, float closestNote, int targetFreqSharp, int targetFreqFlat) {
    float a, b, c; //temp values
    float midpoint; //midpoint between target freq sharp and target freq flat

    if (targetFreqFlat == targetFreqSharp) {
        //Light up green LED e.g. RA7 = 1;
    } else {

        midpoint = (frequencyList[targetFreqSharp] + frequencyList[targetFreqFlat]) / 2;
        a = frequency - frequencyList[targetFreqFlat];
        b = frequencyList[targetFreqSharp] - frequency;

        if (a < b) {
            a = midpoint - frequency; //How far the frequency of the string being played is away from the correct note
            b = midpoint - frequencyList[targetFreqFlat]; //The amount of frequencies between the note to be tuned to and the midpoint between the note above it
            c = (b / a) * 100; //Get the percentage the note being played is to the note to be tuned to.
            //e.g. If the not being played was 50Hz, the note to be tuned to was 0Hz, and the midpoint was 100Hz,
            //c should be 50%
            closestNote = frequencyList[targetFreqFlat];
            if (c < 3) {
                //PORTB = 10000000;
            } else if (c < 15) {
                //PORTB = 01000000;
            } else if (c < 50) {
                //PORTB = 00100000;
            } else if (c < 99) {
                //PORTB = 00010000;
            } else if (c == 100) {
                //RA7 = 1;
            } else if (c > 100) {
                return; //Or break if return doesn't work
            }

            // frequency note is closer to targetFreqFlat

        } else {

            a = frequency - midpoint; //How far the frequency of the string being played is away from the correct note
            b = frequencyList[targetFreqSharp] - midpoint; //The amount of frequencies between the note to be tuned to and the midpoint between the note below it
            c = (b / a) * 100; //Get the percentage the note being played is to the note to be tuned to.
            //e.g. If the not being played was 50Hz, the note to be tuned to was 0Hz, and the midpoint was 100Hz,
            //c should be 50%
            closestNote = frequencyList[targetFreqSharp];
            if (c < 3) {
                //PORTB = 0b00000001;
            } else if (c < 15) {
                //PORTB = 0b00000010;
            } else if (c < 50) {
                //PORTB = 0b00000100;
            } else if (c < 99) {
                //PORTB = 0b00001000;
            } else if (c == 100) {
                //RA3 = 1;
            } else if (c > 100) {
                return; //Or break if return doesn't work
            }
            // frequency note is closer to targetFreqSharp

        }

    }
}

float detectFrequency(char n, float frequency) {
    char x = 0;
    unsigned long startTimeMicroSec;
    unsigned long finishTimeMicroSec;

    //get total time for n cycles in microseconds
    while (RA4 == 1) {
    } //wait for +ve edge
    while (RA4 == 0) {
    }
    startTimeMicroSec = getTimeMicroSec();
    while (x < n) {
        while (RA4 == 1) {
        } //wait for +ve edge
        while (RA4 == 0) {
        }
        x++;
    }
    finishTimeMicroSec = getTimeMicroSec();

    //calculate the frequency in Hz
    frequency = (n * 1000000) / (finishTimeMicroSec - startTimeMicroSec);
}

void displayClosestNote(float closestNote, unsigned char firstDisplay[8], unsigned char secondDisplay[8]) {
    if ((closestNote / 27.5) == 1) {
        storeImageD1(noteA, firstDisplay);
        storeImageD2(blank, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 29.14) == 1) {
        storeImageD1(noteA, firstDisplay);
        storeImageD2(noteSharp, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 30.87) == 1) {
        storeImageD1(noteB, firstDisplay);
        storeImageD2(blank, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 16.35) == 1) {
        storeImageD1(noteC, firstDisplay);
        storeImageD2(blank, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 17.32) == 1) {
        storeImageD1(noteC, firstDisplay);
        storeImageD2(noteSharp, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 18.35) == 1) {
        storeImageD1(noteD, firstDisplay);
        storeImageD2(blank, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 19.45) == 1) {
        storeImageD1(noteD, firstDisplay);
        storeImageD2(noteSharp, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 20.6) == 1) {
        storeImageD1(noteE, firstDisplay);
        storeImageD2(blank, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 21.83) == 1) {
        storeImageD1(noteF, firstDisplay);
        storeImageD2(blank, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 23.12) == 1) {
        storeImageD1(noteF, firstDisplay);
        storeImageD2(noteSharp, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 24.5) == 1) {
        storeImageD1(noteG, firstDisplay);
        storeImageD2(blank, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
    if ((closestNote / 25.96) == 1) {
        storeImageD1(noteG, firstDisplay);
        storeImageD2(noteSharp, secondDisplay);
        update_Displays(firstDisplay, secondDisplay);
    }
}

void main(void) {
    __delay_ms(100);
    //init_PIC16F628A(); //Initialise the PIC16F628A's inputs and outputs, along with other settings
    init_PIC18F2550();
    int noOfDisplays = 2;
    unsigned char firstDisplay[8];
    unsigned char secondDisplay[8];
    int targetFreqSharp = 0; //targetFreqSharp is the index of frequency above the input frequency
    int targetFreqFlat = 0; //targetFreqFlat is the frequency below the 0
    max7219_init(noOfDisplays); //Initialise the max7219's inputs and outputs, along with other settings (e.g. brightness)

    float closestNote; //Holds the frequency of the note to be tuned to
    float frequency = 0;
    
    while (1) {
        ei(); //enable interrupts
        detectFrequency(5, frequency);
        di(); //disable interrupts
        findTargetFreqs(frequency, firstDisplay, secondDisplay, targetFreqSharp, targetFreqFlat);
        displayOutOfTune(frequency, closestNote, targetFreqSharp, targetFreqFlat);
        displayClosestNote(closestNote, firstDisplay, secondDisplay);
    }
}