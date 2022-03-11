//C file: ServoCrossingSwitch_v22.c
//---------------------------------------------------------------------------

//Purpose: Switch relay when servo signal passes threshold point.
//Method: Monitor servo pulse width & switch relay when threshold width passed.


//Copyright (C) 2016  Dave Harris
//This program is free software: you can redistribute it and/or modify it under
//the terms of the GNU General Public License as published by the Free Software
//Foundation, either version 3 of the License, or (at your option) any later
//version. This program is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//FITNESS FOR A PARTICULAR PURPOSE. You should see http://www.gnu.org/licenses/

//------------------------History---------------------------------------
// Dec-2014 Dave Harris v0: started.
// 13-Dec-2015 DH v0.4: move to XC8 and re-factor.
// 20-Apr-2016 DH v2.0: changes to suit v2 (Rev B) circuit. 
//                      a/ 100 mS delay after relay switch
//                      b/ Power-up, hold relay pin o/c till latch set
// 16-Sep-2020 IH v2.1: ensure that the short servo pulse is of a minimum width
//                      to prevent switching on noise spikes and added a
//                      counter to ensure the pulse width is stable before
//                      switching. Ian Hogg
// 11-Mar-2022 DH v2.2: Simplify LED flash codes for Andy Batters.
//                      In setLED() removed NoSigState from case
//

//-------------------------Target---------------------------------------
//MCU: Microchip PIC12F519, 5V, 4MHz internal RC clock
//PCB: ServoCrossingSwitch Rev A(v1 10/2015) or B(v2 04/2016)
//dev: MPLAB X IDE v5.50 + XC8 v2.32 (Free mode) C90

// Memory Summary V2.2
//   Program space used   280h (   640) of   400h words   ( 62.5%)
//   Data space    used    21h (    33) of    29h bytes   ( 80.5%)

#include <xc.h>

//------------------------------------------PIC specific code---------------
#pragma config CP = OFF     //no code protection
#pragma config CPDF = OFF   //ditto
#pragma config WDTE = OFF   //no watchdog timer
#pragma config MCLRE = OFF  //pin functions as RB3
#pragma config FOSC = INTRC //internal RC osc
#pragma config IOSCFS = 4MHz
#define _XTAL_FREQ 4000000  //for __delay_ms()

//------------------------IO pin definition--------------------------------
#define iPINSETUP  RB1 //Setup strap input. Has WPU. (is also ICSPCLK)
#define iPINSERVO  RB3 //Servo signal input. Has WPU. (is also MCLR/VPP)
#define oPINLED    RB2 //LED driver.
#define oPINSTATUS RB0 //Relay state logic output. (is also ICSPDAT)

// B4 and RB5 are the Relay driver and the 2 pins are strapped together
//  and have to be set at same time in setRelay() using RELAYPINMASK.
#define RELAYPINMASK 0b00110000 //map port pins RB4 and RB5

//----------------------global defines----------------------------------
#define OFF 0
#define ON  1
#define LEDNORMAL 0
#define LEDNOSIG  1

//----------------------global variables--------------------------------
unsigned char PwNow;    // +20 to +168 at 16 uS count, see measurePw()
unsigned char PwSaved;  // save PW on entry to set up proc
signed char   PwDiff2;  // half difference, (PwNow-PwSaved)/2 = +74 to -74
unsigned char PwMidpos; // exact mid position
unsigned char PwThreshold[2]; //[1]=lower threshold, [0]=higher threshold
unsigned char ErrorStateCount= 0;
unsigned char PortValue;

bit ErrorState = OFF;
bit SetupState = OFF;
bit RelayState = OFF;
bit NoSigState = OFF;

//-------------NVM(Non Volatile Memory)----------------------------------
//NVM is on-chip High Endurance Flash Data Memory of 64 bytes.
 //Flash needs 8 byte erase before write. Set address in steps of 8 as there
 //is NO lib flash_Write() for this PIC & my function don't save other 7 byte.
#define NVM_ADDR_MID   0 //mid pulse width
#define NVM_ADDR_STATE 8 //relay state

//-----------------------------------------readNVM-----------------------
unsigned char readNVM( unsigned char Addr )
{                       //lib does not have flash routines for 12F519.
	EEADR = Addr;
	EECONbits.RD = 1;   //no poll required
	return(EEDATA);
}

//-----------------------------------------writeNVM----------------------
void writeNVM( unsigned char Addr, unsigned char Data )
{                       //lib does not have flash routines for 12F519.
    EEADR = Addr;       //This code is very 12F519 specific.
    EECONbits.FREE = 1; //start an 8 byte block flash erase
	EECONbits.WREN = 1; //WREN & WR must use BSF instruction pair -see ASM
	EECONbits.WR = 1;   //no need to poll as 12F519 CPU suspends for ~3 mS
    
	EEADR = Addr;       //set the flash address again
	EEDATA = Data;      //write one byte
	EECONbits.WREN = 1; //see WREN & WR note above.
	EECONbits.WR = 1;   //no poll needed as 12F519 suspends for ~3 mS
}

//--------------------------------------------------measurePw-------------
unsigned char measurePw(void)
{                                //This is 12F519 specific...
//TMR0 measures servo signal pulse width range 0.32 to 2.68 mS & 1.5 mS mid
//for FOSC= 4 MHz, Pre Scaler clock= FOSC/4= 1 MHz, pre-scaler=div 16
//, TMR0 clock=62.5 KHz=16.0 uS per count. Count max=255=4.1 mS.
#define PwHILIMIT   168  // 2.68 mS max servo pulse width
#define PwCENTER     94  // 1.5 mS
#define PwLOLIMIT    20  // 0.32 mS min servo pulse width
#define PwHYSTERESIS  4  // 64 uS
#define PwHYSTDIV2    2  // half of above.
	//variable case1 case2 ... count maths.
	// PwNow    168    20  // 20 to 168
	// PwSaved   20   168
	// PwDiff2   74   -74  //(PwNow-PwSaved)/2 is +74 to -74
	TMR0 = 0;
	while( (iPINSERVO == 1) && (TMR0 <= PwHILIMIT) );
	return TMR0;
}

//-----------------------------------------------------setRelay----------
void setRelay( unsigned char RelayBit )
{                                     //This is 12F519 specific...
	oPINSTATUS = RelayBit;
    PortValue = PORTB;
	PORTB = ( RelayBit )
            ? ( PortValue | RELAYPINMASK )
            : ( PortValue & ~RELAYPINMASK );
	if( RelayBit != RelayState )
    {
    	RelayState = RelayBit;
		writeNVM( NVM_ADDR_STATE, (unsigned char)RelayBit );
		__delay_ms(97); 
	}
}

//------------------------------------------setLED-----------------------
void setLED( unsigned char LedMode )
{
//an LED pulse cycle about every second... each short pulse ~70 mS.
// 10000000  relay on and servo signal OK (1 short flash and a pause)
// 10100000  relay off and servo signal OK (2 short flashes & a pause)
// 10001100  relay on and NO servo signal (1 short flash & 1 long flash)
// 10101100  relay off and NO servo signal (2 short flashes & 1 long flash)
// 11110000  in setup mode (slow regular flash)
// 11111110  error state (short blink off) ... remove link

	static unsigned char LedCycleCount = 0;
	static unsigned char LedBeat = 0;
    
	NoSigState = (LedMode == LEDNOSIG) ? ON : OFF;
    
	if( ++LedCycleCount > 4 ) //adjust this constant to get 1 second cycle
    {
		LedCycleCount = 0;
		switch( ++LedBeat )
        {
		 case 1:
            oPINLED = 1; //heartbeat pulse
            break;
		 case 2: case 3:
            oPINLED = SetupState | ErrorState;
            break;
		 case 4:
            oPINLED = RelayState | SetupState | ErrorState;
            break;
		 case 5: case 6: case 7: case 8: case 9:
            oPINLED = SetupState | ErrorState;
            break;
         case 10: case 11: case 12: case 13:
            oPINLED = ErrorState; //was = NoSigState | ErrorState;
            break;
    	 case 14: case 15:
            oPINLED = ErrorState;
            break;
         case 16:
            oPINLED = 0;
            LedBeat = 0;
		}
	}
}

//------------------------------------------procSetupMode------------------
void procSetupMode(void)
{
	static unsigned char Debounce = 0;
    
    if( ErrorStateCount > 0 )
    {
        ErrorStateCount--;
    }
    else 
    {
        ErrorState = OFF;
    }

	oPINSTATUS = 0; // set pin low, if link is on then set up pin goes low...
    
	if( iPINSETUP == 0 )  // so set up link is on...
    {
		oPINSTATUS = RelayState; // reset pin to whatever it was. ~10 uS
		if( Debounce < 5 )
        {
            Debounce++; 
        }
		else
        {
			if( SetupState == OFF )
            {
				PwSaved = PwNow;
				SetupState = ON;
			}
		}
	}
	else //so... setup pin is high (no link on and input has pull up)
    {
		oPINSTATUS = RelayState; //reset pin to whatever it was. ~10 uS
		if( SetupState == ON )
        {
			PwDiff2 = (signed char)(PwNow>>1) - (signed char)(PwSaved>>1);
			// = a/2 - b/2 = (a>>1) - (b>>1) nb: >>1 is shiftR 1 = div 2
			if( ((PwDiff2 < 0) ? (-PwDiff2) : (PwDiff2)) <= PwHYSTERESIS )
            {
				ErrorStateCount = 20;
                ErrorState = ON;
			}
			else // calculate PW mid position
            {
				PwMidpos = (unsigned char)((signed int)PwSaved
							+ (signed int)PwDiff2 );
				writeNVM( NVM_ADDR_MID, PwMidpos );
			}
		}
		SetupState = OFF;
		Debounce = 0;
	}
}

//------------------------------------------initDevice---------------
void initDevice(void)
{                                     //This is 12F519 specific...
	RelayState = readNVM( NVM_ADDR_STATE );
	PORTB = ( RelayState )
            ? RELAYPINMASK
            : 0;
	oPINSTATUS = RelayState;
    TRIS = 0b00001010;
        // RB5=out, 4=out, 3=in, 2=out, 1=in, 0=out (out=0,in=1)
    OPTION = 0b10000011;
        // WakeUpOnPinChange=1=disable, WeakPullUp=0=enable,
        // T0CS=0=internal, T0SE=0=Lo2Hi, PSA=0=TMR0, PS=011=1:16
}

//--------------------------------------------main----------------------
#define STABILITY_COUNT 3
void main(void)
{
	initDevice();
	PwMidpos = readNVM( NVM_ADDR_MID );
	if( ( PwMidpos >= PwHILIMIT ) || ( PwMidpos <= PwLOLIMIT ) )
    {
		PwMidpos = PwCENTER;
		writeNVM( NVM_ADDR_MID, PwMidpos );
	}
	unsigned int LowCount = 0;
    unsigned char validPulseCount = 0;
    
	while(1) //loop forever
    {
		PwThreshold[ 1 ] = PwMidpos - PwHYSTDIV2; //RelayState selects which
		PwThreshold[ 0 ] = PwMidpos + PwHYSTDIV2;
        
		if( iPINSERVO == 1 ) //servo signal is high
        {
			PwNow = measurePw();
			if( PwNow > PwHILIMIT ) // servo pulse is too long
            {
                validPulseCount = 0;
				setLED( LEDNOSIG );
                __delay_ms( 15 );
			}
            else
            {
                if( PwNow < PwLOLIMIT )
                { // a short spike on the servo signal input
                
                    validPulseCount = 0;
                    // do nothing
                }
                else // its a normal pulse
                { 
                    validPulseCount++;

                    if( validPulseCount >= STABILITY_COUNT ) 
                    {   //set relay conditional; PW > selected threshold
                        setRelay( PwNow > PwThreshold[RelayState] );
                        setLED( LEDNORMAL );
                        procSetupMode();
                    }
                }
                LowCount = 0;
            }
		}
		else //servo signal is low
        {
			if( ++LowCount > 2500 ) //low for too long?
            {
				setLED( LEDNOSIG );
				LowCount = 0;
			}
		}
	}
}
//--------------------EoF ServoCrossingSwitch_v22.c----------------
