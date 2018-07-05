#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>        // Includes uint16_t definition
#include <stdbool.h>       // Includes true/false definition

// Declares For Delay Function
#define SYS_FREQ        121951219L // verified by using the noop time difference of pulses
#define FCY             SYS_FREQ/2
#include <libpic30.h>   

//**************************************************//

// setup oscillator as primary oscillator
// see table 3-1 of oscillator datasheet
#pragma config POSCMD   = 01     // XT external oscilator
#pragma config FNOSC    = 0b011  // PLL on

#pragma config WDTEN    = OFF    // Watchdog Timer off
#pragma config JTAGEN   = OFF    // JTAG port is disabled



//****************************************************************************
// Global Variables
//****************************************************************************

// Sets the value the systems believes to indicate the beginning of the ping
uint16_t DetectionThreshold = 2150;   
uint16_t result0;

uint16_t holdcounter;

volatile unsigned short result1;
volatile unsigned short result2;
volatile unsigned short result3;


void EnableAndCalibrate();
void SetADCs();
void UART_Config (void);
//void Packetize(uint8_t *H1High, uint8_t *H1Low, uint8_t *H2High, uint8_t *H2Low, uint8_t *H3High,uint8_t *H3Low, uint8_t *H4High,uint8_t *H4Low);

/*
 *      The Variables Required to capture and hold the values in the interrupts
 */
uint8_t Ping = 0;

uint8_t LowHolderH1[300];
uint8_t HighHolderH1[300];
uint8_t LowHolderH2[300];
uint8_t HighHolderH2[300];
uint8_t LowHolderH3[300];
uint8_t HighHolderH3[300];
uint8_t LowHolderH4[300];
uint8_t HighHolderH4[300];

uint8_t PacketizeH1 = 0;
uint8_t PacketizeH2 = 0;
uint8_t PacketizeH3 = 0;
uint8_t PacketizeH4 = 0;

uint16_t SampleCounter;



//****************************************************************************
// Main
//****************************************************************************
int main(int argc, char** argv)
{

    SetADCs();
    UART_Config();
    

// TIMER 2 INITIALIZATION (TIMER2 IS USED AS A TRIGGER SOURCE FOR ALL CHANNELS).
T2CONbits.TCS = 0; // use Fosc/2 for Timer2
T2CONbits.TCKPS = 0; // 1:1 prescale = Fcy = Fosc/2
PR2 = 300; // Match: rollover every 300 counts
T2CONbits.TON = 1; // start Timer2 --> generate triggers


 // Setup Oscillator as Primary External With PLL
    // see register 6-1 of oscillator datasheet
    OSCCONbits.COSC = 0b011;
    
    // set cpu peripheral clock
    // see register 6-2 of oscillator datasheet
    CLKDIVbits.DOZEN  = 0b0;    // 1:1 ratio
    CLKDIVbits.DOZE   = 0b000;  // 1:1 ratio     // may be unecessary
    CLKDIVbits.FRCDIV = 0b000; // divide by 1   // may be unecessary
    
    // example 7-1 and Table 7-1:
    // setup for 60mhz
    CLKDIVbits.PLLPRE  = 0;     // N1=2:
    PLLFBDbits.PLLDIV  = 60-2;  // M: // set to 60, -2 is due to all zeroes means 2
    CLKDIVbits.PLLPOST = 0b00;  // N2=2:
    
    //REFOCON - route clock to an external io pin
    //REFOCONbits.RODIV = 0b0000;
    //REFOCONbits.ROSEL = 0; // use the clock source for the rest of the device
    //REFOCONbits.ROON  = 1; // enable the clock output to rp pin REFCLKO
    
    //CLOCKOUT_MAP; // macro to map the clockout to rp pin
    //CLOCKOUT_MODE = OUTPUT; // setup pin as a output
    
      
        
    while(1)
    {    
        
        if((PacketizeH1 == 1)&&(PacketizeH2 == 1)&&(PacketizeH3 == 1)&&(PacketizeH4 == 1))
        {
//            Packetize(HighHolderH1, LowHolderH1, HighHolderH2, LowHolderH2, HighHolderH3, LowHolderH3, HighHolderH4, LowHolderH4);

            
            
            for(holdcounter = 0; holdcounter < 300; holdcounter++)
            {
                U1TXREG = HighHolderH1[holdcounter];
                while(U1STAbits.TRMT == 0);       
                HighHolderH1[holdcounter] = 0;
                U1TXREG = LowHolderH1[holdcounter];
                while(U1STAbits.TRMT == 0);       
                LowHolderH1[holdcounter] = 0;                
            } 
            
            for(holdcounter = 0; holdcounter < 300; holdcounter++)
            {            
                U1TXREG = HighHolderH2[holdcounter];
                while(U1STAbits.TRMT == 0);       
                HighHolderH2[holdcounter] = 0;
                U1TXREG = LowHolderH2[holdcounter];
                while(U1STAbits.TRMT == 0);                
                LowHolderH2[holdcounter] = 0;
            }
            
            for(holdcounter = 0; holdcounter < 300; holdcounter++)
            {            
                U1TXREG = HighHolderH3[holdcounter];
                while(U1STAbits.TRMT == 0);       
                HighHolderH3[holdcounter] = 0;
                U1TXREG = LowHolderH3[holdcounter];
                while(U1STAbits.TRMT == 0);                
                LowHolderH3[holdcounter] = 0;
            }
            
            for(holdcounter = 0; holdcounter < 300; holdcounter++)
            {            
                U1TXREG = HighHolderH4[holdcounter];
                while(U1STAbits.TRMT == 0);       
                HighHolderH4[holdcounter] = 0;
                U1TXREG = LowHolderH4[holdcounter];
                while(U1STAbits.TRMT == 0);                
                LowHolderH4[holdcounter] = 0;
            }
            
                                                __delay_ms(500);

            
            ADIELbits.IE0 = 1; // disable interrupt for AN0
            ADIELbits.IE1 = 1; // disable interrupt for AN1     
            ADIELbits.IE2 = 1; // disable interrupt for AN0
            ADIELbits.IE3 = 1; // disable interrupt for AN1  
            PacketizeH1 = 0;
            PacketizeH2 = 0;
            PacketizeH3 = 0;
            PacketizeH4 = 0;            
        }

    }
 
    return (EXIT_SUCCESS);
}

uint8_t high_byte_H1;
uint8_t low_byte_H1;
uint8_t high_byte_H2;
uint8_t low_byte_H2;
uint8_t high_byte_H3;
uint8_t low_byte_H3;
uint8_t high_byte_H4;
uint8_t low_byte_H4;


void __attribute__((interrupt, no_auto_psv)) _ADCAN0Interrupt(void)
{
    result0 = ADCBUF0;       // read conversion result
    low_byte_H1 = (result0 & 0xFF);
    high_byte_H1 = result0 >> 8;
    
    if((result0 > DetectionThreshold)&&(Ping == 0))
    {
        Ping = 1;
        LowHolderH1[SampleCounter] = low_byte_H1;
        HighHolderH1[SampleCounter] = high_byte_H1;        
    }
    else if((Ping == 1)&&(SampleCounter < 300))
    {
        LowHolderH1[SampleCounter] = low_byte_H1;
        HighHolderH1[SampleCounter] = high_byte_H1;         
    }
    else if((Ping == 1)&&(SampleCounter == 300))
    {
        PacketizeH1 = 1;
    }
    else
    {
        LowHolderH1[0] = low_byte_H1;
        HighHolderH1[0] = high_byte_H1;             
    }
    
    _ADCAN0IF = 0;          // clear interrupt flag
}

// ADC AN1 ISR
void __attribute__((interrupt, no_auto_psv)) _ADCAN1Interrupt(void)
{
    
    result1 = ADCBUF1; // read conversion result
    low_byte_H2 = (result1 & 0xFF);
    high_byte_H2 = result1 >> 8;

    
    if((result1 > DetectionThreshold)&&(Ping == 0))
    {
        Ping = 1;
        LowHolderH2[SampleCounter] = low_byte_H2;
        HighHolderH2[SampleCounter] = high_byte_H2;       
    }
    else if((Ping == 1)&&(SampleCounter < 300))
    {
        LowHolderH2[SampleCounter] = low_byte_H2;
        HighHolderH2[SampleCounter] = high_byte_H2;       
    }
    else if((Ping == 1)&&(SampleCounter == 300))
    {
        PacketizeH2 = 1;
    }
    else
    {
        LowHolderH2[0] = low_byte_H2;
        HighHolderH2[0] = high_byte_H2;            
    }    
    
    _ADCAN1IF = 0; // clear interrupt flag
}


// ADC AN2 ISR
void __attribute__((interrupt, no_auto_psv)) _ADCAN2Interrupt(void)
{
    
    result2 = ADCBUF2;       // read conversion result
    
    low_byte_H3 = (result2 & 0xFF);
    high_byte_H3 = result2 >> 8;

    
    if((result2 > DetectionThreshold)&&(Ping == 0))
    {
        Ping = 1;
        LowHolderH3[SampleCounter] = low_byte_H3;
        HighHolderH3[SampleCounter] = high_byte_H3;       
    }
    else if((Ping == 1)&&(SampleCounter < 300))
    {
        LowHolderH1[SampleCounter] = low_byte_H3;
        HighHolderH1[SampleCounter] = high_byte_H3;       
    }
    else if((Ping == 1)&&(SampleCounter == 300))
    {
        PacketizeH3 = 1;     
    }
    else
    {
        LowHolderH3[0] = low_byte_H3;
        HighHolderH3[0] = high_byte_H3;            
    }    
    
    _ADCAN2IF = 0;          // clear interrupt flag
}

// ADC AN3 ISR
void __attribute__((interrupt, no_auto_psv)) _ADCAN3Interrupt(void)
{
    
    result3 = ADCBUF3; // read conversion result
    low_byte_H4 = (result3 & 0xFF);
    high_byte_H4 = result3 >> 8;

    
    if((result3 > DetectionThreshold)&&(Ping == 0))
    {
        Ping = 1;
        LowHolderH4[SampleCounter] = low_byte_H4;
        HighHolderH4[SampleCounter] = high_byte_H4;       
        SampleCounter++;
    }
    else if((Ping == 1)&&(SampleCounter < 300))
    {
        LowHolderH4[SampleCounter] = low_byte_H4;
        HighHolderH4[SampleCounter] = high_byte_H4;       
        SampleCounter++;
    }
    else if((Ping == 1)&&(SampleCounter == 300))
    {
        PacketizeH4 = 1;
        SampleCounter = 0;
        Ping = 0;

        ADIELbits.IE0 = 0; // disable interrupt for AN2
        ADIELbits.IE1 = 0; // disable interrupt for AN3           
        ADIELbits.IE2 = 0; // disable interrupt for AN2
        ADIELbits.IE3 = 0; // disable interrupt for AN3        
        
    }
    else
    {
          
    }       

    _ADCAN3IF = 0; // clear interrupt flag
}



void SetADCs() 
{
// AN0/RA0 uses Dedicated ADC Core 0 
ANSELAbits.ANSA0 = 1; // AN0/RA0 pin is analog
TRISAbits.TRISA0 = 1; // AN0/RA0 pin is input 
// AN1/RA1 uses Dedicated ADC Core 1 
ANSELAbits.ANSA1 = 1; // AN1/RA1 pin is analog
TRISAbits.TRISA1 = 1; // AN1/RA1 pin is input 
// AN2/RA2 uses Dedicated ADC Core 2 
ANSELAbits.ANSA2 = 1; // AN2/RA2 pin is analog
TRISAbits.TRISA2 = 1; // AN2/RA2 pin is input 
// AN3/RA3 uses Dedicated ADC Core 3 
ANSELBbits.ANSB0 = 1; // AN3/RB0 pin is analog
TRISBbits.TRISB0 = 1; // AN3/RB0 pin is input 


// Configure common ADC clock
ADCON3Hbits.CLKSEL = 2; // clock from FRC oscillator                        
ADCON3Hbits.CLKDIV = 0; // no clock divider (1:1)                           

// Configure cores ADC clocks
ADCORE0Hbits.ADCS = 0; // clock divider (1:2)                              
ADCORE1Hbits.ADCS = 0; // clock divider (1:2)
ADCORE2Hbits.ADCS = 0; // clock divider (1:2)
ADCORE3Hbits.ADCS = 0; // clock divider (1:2)

// Configure the ADC reference voltages (high and low)
ADCON3Lbits.REFSEL = 0; // Use AVdd and AVss as high and low voltage ref.'s

// integer or fractional output format?
ADCON1Hbits.FORM = 0; // ADC output is integer format
ADCON1Hbits.SHRRES = 1; // 8bit resolution

// Select single-ended input configuration and unsigned output format.
ADMOD0Lbits.SIGN0 = 0; // Channel 0 output data is unsigned                 
ADMOD0Lbits.DIFF0 = 0; // Channel 0 is single-ended 
ADMOD0Lbits.SIGN1 = 0; // Channel 1 output data is unsigned                 
ADMOD0Lbits.DIFF1 = 0; // Channel 1 is single-ended   
ADMOD0Lbits.SIGN2 = 0; // Channel 2 output data is unsigned                 
ADMOD0Lbits.DIFF2 = 0; // Channel 2 is single-ended  
ADMOD0Lbits.SIGN3 = 0; // Channel 3 output data is unsigned                 
ADMOD0Lbits.DIFF3 = 0; // Channel 3 is single-ended 

// Enable and calibrate the module.
EnableAndCalibrate(); // See Example 5-1

ADIELbits.IE0 = 1; // enable interrupt for AN0
ADIELbits.IE1 = 1; // enable interrupt for AN1
ADIELbits.IE2 = 1; // enable interrupt for AN2
ADIELbits.IE3 = 1; // enable interrupt for AN3  

_ADCAN0IF = 0; // clear interrupt flag for AN0
_ADCAN0IE = 1; // enable interrupt for AN0
_ADCAN1IF = 0; // clear interrupt flag for AN1
_ADCAN1IE = 1; // enable interrupt for AN1
_ADCAN2IF = 0; // clear interrupt flag for AN2
_ADCAN2IE = 1; // enable interrupt for AN2
_ADCAN3IF = 0; // clear interrupt flag for AN3
_ADCAN3IE = 1; // enable interrupt for AN3

ADTRIG0Lbits.TRGSRC0 = 13; // TriggerSource forAN0 =Timer2 period match
ADTRIG0Lbits.TRGSRC1 = 13; // TriggerSource forAN1 =Timer2 period match 
ADTRIG0Hbits.TRGSRC2 = 13; // TriggerSource forAN2 =Timer2 period match 
ADTRIG0Hbits.TRGSRC3 = 13; // TriggerSource forAN3 =Timer2 period match 
}


void EnableAndCalibrate()
{
// Set initialization time to maximum
ADCON5Hbits.WARMTIME = 15;
// Turn on ADC module
ADCON1Lbits.ADON = 1;

// Turn on analog power for dedicated core 0
ADCON5Lbits.C0PWR = 1;
// Wait when the core 0 is ready for operation
while(ADCON5Lbits.C0RDY == 0);
// Turn on digital power to enable triggers to the core 0
ADCON3Hbits.C0EN = 1;

// Turn on analog power for dedicated core 1
ADCON5Lbits.C1PWR = 1;
// Wait when the core 1 is ready for operation
while(ADCON5Lbits.C1RDY == 0);
// Turn on digital power to enable triggers to the core 1
ADCON3Hbits.C1EN = 1;

// Turn on analog power for dedicated core 2
ADCON5Lbits.C2PWR = 1;
// Wait when the core 0 is ready for operation
while(ADCON5Lbits.C2RDY == 0);
// Turn on digital power to enable triggers to the core 0
ADCON3Hbits.C2EN = 1;

// Turn on analog power for dedicated core 3
ADCON5Lbits.C3PWR = 1;
// Wait when the core 1 is ready for operation
while(ADCON5Lbits.C3RDY == 0);
// Turn on digital power to enable triggers to the core 1
ADCON3Hbits.C3EN = 1;

// Enable calibration for the dedicated core 0
ADCAL0Lbits.CAL0EN = 1;
// Single-ended input calibration
ADCAL0Lbits.CAL0DIFF = 0;
// Start calibration
ADCAL0Lbits.CAL0RUN = 1;
// Poll for the calibration end
while(ADCAL0Lbits.CAL0RDY == 0);
// Differential input calibration
ADCAL0Lbits.CAL0DIFF = 1;
// Start calibration
ADCAL0Lbits.CAL0RUN = 1;
// Poll for the calibration end
while(ADCAL0Lbits.CAL0RDY == 0);
// End the core 0 calibration
ADCAL0Lbits.CAL0EN = 0;

// Enable calibration for the dedicated core 1
ADCAL0Lbits.CAL1EN = 1;
// Single-ended input calibration
ADCAL0Lbits.CAL1DIFF = 0;
// Start calibration
ADCAL0Lbits.CAL1RUN = 1;
// Poll for the calibration end
while(ADCAL0Lbits.CAL1RDY == 0);
// Differential input calibration
ADCAL0Lbits.CAL1DIFF = 1;
// Start calibration
ADCAL0Lbits.CAL1RUN = 1;
// Poll for the calibration end
while(ADCAL0Lbits.CAL1RDY == 0);
// End the core 1 calibration
ADCAL0Lbits.CAL1EN = 0;

// Enable calibration for the dedicated core 2
ADCAL0Hbits.CAL2EN = 1;
// Single-ended input calibration
ADCAL0Hbits.CAL2DIFF = 0;
// Start calibration
ADCAL0Hbits.CAL2RUN = 1;
// Poll for the calibration end
while(ADCAL0Hbits.CAL2RDY == 0);
// Differential input calibration
ADCAL0Hbits.CAL2DIFF = 1;
// Start calibration
ADCAL0Hbits.CAL2RUN = 1;
// Poll for the calibration end
while(ADCAL0Hbits.CAL2RDY == 0);
// End the core 0 calibration
ADCAL0Hbits.CAL2EN = 0;

// Enable calibration for the dedicated core 3
ADCAL0Hbits.CAL3EN = 1;
// Single-ended input calibration
ADCAL0Hbits.CAL3DIFF = 0;
// Start calibration
ADCAL0Hbits.CAL3RUN = 1;
// Poll for the calibration end
while(ADCAL0Hbits.CAL3RDY == 0);
// Differential input calibration
ADCAL0Hbits.CAL3DIFF = 1;
// Start calibration
ADCAL0Hbits.CAL3RUN = 1;
// Poll for the calibration end
while(ADCAL0Hbits.CAL3RDY == 0);
// End the core 1 calibration
ADCAL0Hbits.CAL3EN = 0;

}


void UART_Config (void)
{
U1MODEbits.STSEL = 0; // 1-Stop bit
U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits
U1MODEbits.ABAUD = 0; // Auto-Baud disabled
U1MODEbits.BRGH = 1; // High-Speed mode (FP/(4xBaudRate))-1
U1BRG = 127; // Baud Rate setting (FP/(4xBaudRate))-1 (115200 BAUD)
U1MODEbits.UARTEN = 1; // Enable UART
U1STAbits.UTXEN = 1; // Enable UART TX

//Pin 16 dsPICEP64GS502
RPOR2bits.RP37R = 1;
TRISBbits.TRISB5 = 0;

}
