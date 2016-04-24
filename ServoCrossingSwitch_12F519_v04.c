//C file: ServoCrossingSwitch_12F519_v04.c
//---------------------------------------------------------------------------
//Purpose: Switch crossing polarity when servo signal changes.
//Author: Dave Harris

// Monitor servo pulse width & change relay state when threshold width passed.

// Circuit used has 6 i/o pins...
//  1 output pin drives on board LED (active high)
//  1 input pin to measure servo pulse width (active high)
//  2 pins strapped together to drive relay (active low)
//  1 output pin which reflects the relay state (active high)
//  1 input to signal setting up of mid position (active low)

//-------------------------Target---------------------------------------
//MCU: Microchip PIC12F519; 5 V, 4 MHz internal, 53% words & 76% Ram used.
//PCB: 'ServoCrossingSwitch Rev A (10/2015)'
//dev: MPLAB X IDE v3.15 / XC8 v1.35(Free mode)

//------------------------History---------------------------------------
// Dec-2014 Dave Harris v0: started.
// 13-Dec-2015 DH v0.4: move to XC8 and re-factor.
//

//----------------------- Libraries ------------------------------------

#include <xc.h>

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
bit ErrorState = OFF;
bit SetupState = OFF;
bit RelayState = OFF;
bit NoSigState = OFF;

//------------------------------------------MCU specific code---------------

#pragma config CP = OFF     //no code protection
#pragma config CPDF = OFF   //ditto
#pragma config WDTE = OFF   //no watchdog timer
#pragma config MCLRE = OFF  //pin functions as RB3
#pragma config FOSC = INTRC //internal RC osc
#pragma config IOSCFS = 4MHz
#define _XTAL_FREQ 4000000  //for __delay_ms()

//------------------------------------------initDevice---------------
void initDevice(void)
{
 #define oPINSTATUS RB0 //Relay state logic output. (is also ICSPDAT)
 #define iPINSETUP  RB1 //Setup strap input. Has WPU. (also ICSPCLK)
 #define oPINLED    RB2 //LED driver.
 #define iPINSERVO  RB3 //Servo signal input. Has WPU. (also MCLR/VPP)
 //RB4 and RB5 are the Relay driver and the 2 pins are strapped together
 //  and have to be set at same time in setRelay() using RELAYPINMASK.
 #define RELAYPINMASK 0b00110000 //map port pins, RB4 and RB5

 TRIS = 0b00001010;  //RB5=out, 4=out, 3=in, 2=out, 1=in, 0=out (out=0,in=1)

 OPTION = 0b10000011;//WakeUpOnPinChange=1=disable, WeakPullUp=0=enable,
}                    //T0CS=0=internal, T0SE=0=Lo2Hi, PSA=0=TMR0, PS=011=1:16

//-------------Non Volatile Memory-------------------------------------------
// is on-chip High Endurance Flash Data Memory = 64 bytes

//Flash need 8 byte erase before write. Set address in 8s as there
// is NO lib flash_Write() for this & my function don't save other 7 byte.

#define NVMADRRESS_MID   0 //mid pulse width
#define NVMADDRESS_RELAY 8 //relay state

//-----------------------------------------readNVM-----------------------
unsigned char readNonVolatileMemory(unsigned char Addr)
{                       //lib does not have flash routines for 12F519.
	EEADR = Addr;
	RD = 1;         //no poll needed
	return(EEDATA);
}
//-----------------------------------------writeNVM----------------------
void writeNonVolatileMemory(unsigned char Addr, unsigned char Data)
{                       //lib does not have flash routines for 12F519.
    EEADR = Addr;       //This is very 12F519 specific...
    EECONbits.FREE = 1; //start an 8 byte block flash erase
	EECONbits.WREN = 1; //WREN & WR must use BSF instruction pair -see listing
	EECONbits.WR = 1;   //no need to poll as 12F519 CPU suspends for ~3 mS
	EEADR = Addr;
	EEDATA = Data;      //write one byte
	EECONbits.WREN = 1; //see WREN & WR note above.
	EECONbits.WR = 1;   //no poll needed as 12F519 suspends for ~3 mS
}

//--------------------------------------------------measurePw-------------
unsigned char measurePw(void)
{
//TMR0 measures servo signal pulse width range 0.32 to 2.68 mS & 1.5 mS mid
//for FOSC = 4 MHz, Pre Scaler clock = FOSC/4 = 1 MHz, pre-scaler = div 16
//  , TMR0 clock = 62.5 KHz = 16.0 uS per count. Count max = 255 = 4.1 mS.

#define PwHILIMIT   168 // 2.68 mS max servo pulse width
#define PwCENTER     94 // 1.5 mS
#define PwLOLIMIT    20 // 0.32 mS min servo pulse width
#define PwHYSTERESIS  4 // 64 uS
#define PwHYSTDIV2    2 // half of above.

	//variable case1 case2 ... count maths.
	// PwNow    168    20  // 20 to 168
	// PwSaved   20   168
	// PwDiff2   74   -74  //(Now-Saved)/2 is +74 to -74

	TMR0 = 0;
	while((iPINSERVO == 1) && (TMR0 <= PwHILIMIT));
	return TMR0;
}

//-----------------------------------------------------setRelay----------
void setRelay(unsigned char RelayBit)
{
	oPINSTATUS = RelayBit;
	unsigned char PortValue = PORTB;

	PORTB = (RelayBit)?(PortValue|RELAYPINMASK):(PortValue&~RELAYPINMASK);
	if(RelayBit != RelayState){
    	RelayState = RelayBit;
		writeNonVolatileMemory(NVMADDRESS_RELAY, (unsigned char)RelayBit);
	}
}

//----------------------------------- Non chip Specific code -----------
// loop timings might change but not the code...

//------------------------------------------setLED------------
void setLED(unsigned char LedMode)
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
	if(++LedCycleCount > 4){ //adjust this constant to get 1 second cycle
		LedCycleCount = 0;
		switch(++LedBeat){
		 case 1: oPINLED = 1; break;//heartbeat pulse
		 case 2: case 3: oPINLED = SetupState | ErrorState; break;
		 case 4: oPINLED = RelayState | SetupState | ErrorState; break;
		 case 5: case 6: case 7: case 8: case 9:
                            oPINLED = SetupState | ErrorState; break;
         case 10: case 11: case 12: case 13:
                            oPINLED = NoSigState | ErrorState; break;
    	 case 14: case 15: oPINLED = ErrorState; break;
         case 16: oPINLED = 0; LedBeat = 0;
		}
	}
}

//------------------------------------------procSetupMode------------------
void procSetupMode(void)
{
	static unsigned char Debounce = 0;

    if( ErrorStateCount > 0 ){ ErrorStateCount--;}else{ErrorState = OFF;}

	oPINSTATUS = 0; //set pin low, if link is on then set up pin goes low...
	if(iPINSETUP == 0){ //its low, so set up link is on...
		oPINSTATUS = RelayState; //reset pin to whatever it was. ~10 uS
		if(Debounce < 5 ){ Debounce++; }
		else{
			if(SetupState == OFF){
				PwSaved = PwNow;
				SetupState = ON;
			}
		}
	}
	else{//so... setup pin is high (no link on and input has pull up)
		oPINSTATUS = RelayState; //reset pin to whatever it was. ~10 uS
		if(SetupState == ON){
			PwDiff2 = (signed char)(PwNow>>1) -(signed char)(PwSaved>>1);
			// = a/2 - b/2 = (a>>1) - (b>>1) ... >>1 is shift1= div 2
			if( ((PwDiff2 < 0)?(-PwDiff2):(PwDiff2)) <= PwHYSTERESIS ){
				ErrorStateCount = 20;
                ErrorState = ON;
			}
			else{		// calculate PW mid position
				PwMidpos = (unsigned char)((signed int)PwSaved
							+ (signed int)PwDiff2 );
				writeNonVolatileMemory(NVMADRRESS_MID, PwMidpos);
			}
		}
		SetupState = OFF;
		Debounce = 0;
	}
}

//--------------------------------------------main----------------------
void main(void)
{
	initDevice();
	RelayState = readNonVolatileMemory(NVMADDRESS_RELAY);
	setRelay(RelayState);
	PwMidpos = readNonVolatileMemory(NVMADRRESS_MID);
	if(( PwMidpos >= PwHILIMIT ) || ( PwMidpos <= PwLOLIMIT )){
		PwMidpos = PwCENTER;
		writeNonVolatileMemory(NVMADRRESS_MID, PwMidpos);
	}
	unsigned int LowCount = 0;
	while(1){ //loop forever
		PwThreshold[1] = PwMidpos-PwHYSTDIV2; //RelayState selects which one
		PwThreshold[0] = PwMidpos+PwHYSTDIV2;
		if( iPINSERVO == 1 ){ //servo signal is high
			PwNow = measurePw();
			if(PwNow > PwHILIMIT){
				setLED(LEDNOSIG);
                __delay_ms(15);
			}
			else{ //set relay conditional; PW > selected threshold
 				setRelay(PwNow > PwThreshold[RelayState] );
				setLED(LEDNORMAL);
				procSetupMode();
			}
			LowCount = 0;
		}
		else{ //servo signal is low
			if( ++LowCount > 2500 ){ //low for too long?
				setLED(LEDNOSIG);
				LowCount = 0;
			}
		}
	}
}//----------------------------------------------EndOfFile----------------
