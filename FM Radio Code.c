//
// FILE     main.c
// DATE     140128
// WRITTEN  RAC
// PURPOSE  Utilities to communicate with FM receiver IC.   $
// LANGUAGE MPLAB C18
// KEYWORDS USB/I2C/SPARKFUN/FM_MODULE/RECEIVER
// PROJECT  FM TxRx Lab experiment
// CATEGORY UTILITY
// TARGET   Darwin
//
//











// Check numbers of segments in LCD

// Volume up/down - get to know what array is

// Add verifications where needed

// Check Ar1010 dotasheet for codes for volume

// Check if it is possible to explicitely write value directly to the register after masking rest out, it is like regImg[3]=arr[n]
//If so, then it simplifies the code as it will be just one number










// This is skeletal code which won't run until 'fleshed out'.
// It does give an idea of the setup and initialization required
// of the PIC and the FM tuner module.  The basics of communication
// over the I2C bus are also covered.
//
// This version contains eevisions needed to compile under MPLABX.
//
// PIC is highly configurable to meet various needs. For this project
// we need to enable the LCD segments and I2C bus. The following
// code and also in Init() configure PIC for our need in this project.
// You should not change the configuration unless you modify the
// hardware design.


#pragma config OSC = INTIO7  // Internal osc, RA6=CLKO, RA7=I/O
#pragma config FCMEN = OFF   // Fail-Safe Clock Monitor disabled
#pragma config IESO = OFF    // Oscillator Switchover mode disabled
#pragma config WDT = OFF     // WDT disabled (control through SWDTEN bit)
#pragma config PWRT = OFF    // racmod  -> PWRT disabled
#pragma config MCLRE = ON    // MCLR pin enabled; RG5 input pin disabled
#pragma config XINST = OFF   // Instruction set extension disabled
#pragma config BOREN = OFF   // Brown-out controlled by software
#pragma config BORV = 3      // Brown-out voltage set for 2.0V, nominal
#pragma config STVREN = OFF  // Stack full/underflow will not cause Reset
#pragma config CP = OFF	     // Program memory block not code-protected



#include <plib/i2c.h> // If compiler could not find this header file
                      // check if the legacy peripheral library
                      // is installed properly

#include "fm.h"


// FM register bank defaults -
const unsigned int regDflt[18] = {
	0xFFFF,     // R0 -- the first writable register .  (disable xo_en)
	0x5B15,     // R1.
	0xD0B9,     // R2.
	0xA010,     // R3   seekTHD = 16
	0x0780,     // R4
	0x28AB,     // R5
	0x6400,     // R6
	0x1EE7,     // R7
	0x7141,     // R8
	0x007D,     // R9
	0x82C6,     // R10  disable wrap
	0x4F55,     // R11. <--- (disable xo_output)
	0x970C,     // R12.
	0xB845,     // R13
	0xFC2D,     // R14
	0x8097,     // R15
	0x04A1,     // R16
	0xDF6A      // R17
};


// We keep a copy of AR1010 register data in regImg[].
// When writing to AR1010, we take the value from regImg
// and write to AR1010 via I2C.
unsigned int regImg[18];	// FM register bank images
int volume;	// Declaration - used in volUp func


/*
 * Obtain latest change in state for the pushbutton set.
 *
 * @param butn Which button changed.  See fm.h.
 *
 * @return 	0 if no button has changed state,
 *			1 if button is pushed,
 *
 */
unsigned char butnEvent(unsigned char *butn) {

    unsigned int but, i;
    unsigned int ports[8];

    ports[0] = PORTBbits.RB0;
    ports[1] = PORTBbits.RB5;
    ports[2] = PORTAbits.RA0;
    ports[3] = PORTAbits.RA1;
    ports[4] = PORTGbits.RG0;
    ports[5] = PORTGbits.RG1;
    ports[6] = PORTGbits.RG2;
    ports[7] = PORTGbits.RG3;

    for(i=0; i<8; i++) {
        but = ports[i];
        if (but==1) {
            *butn = i;
            while(1){
                if (ports[i]==0) break;
            }
            return 1;
        }
    }

    return 0;		// No changes
}
//
// end butnEvent ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

void dly(int d) {

	int i = 0;

	for ( ; d; --d)
		for (i = 100;  i;  --i) ;
}
//
// end dly ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

/*
 * Set all LCD segments to 0 (off, clear).
 *
 */
void clrscn() {
	int i = 0;
	unsigned char *CLEARptr;        // Pointer used to clear all LCDDATA

	for (	i = 0,
			CLEARptr = (unsigned char *) &LCDDATA0;  // Point to first segment
			i < 28;
			i++)		// Turn off all segments
		*CLEARptr++ = 0x00;
}
//
// end clrscn ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

void Init() {

	int i;

	OSCCON = 0b01110010;        	// Select 8 MHz internal oscillator
	LCDSE0 = 0b11111111;        	// Enable  LCD segments 07-00
	LCDSE1 = 0b11111111;        	// Enable  LCD segments 15-08
	LCDSE2 = 0b11111111;        	// Enable  LCD segments 23-16
	LCDSE3 = 0b00000000;        	// Disable LCD segments 31-24
	LCDCON = 0b10001000;         	// Enab LC controller. Static mode. INTRC clock
	LCDPS  = 0b00110110;         	// 37 Hz frame frequency
	ADCON1 = 0b00111111;        	// Make all ADC/IO pins digital
	TRISA = 0b00000011;             // RA0 and RA1 pbutton
	TRISB = 0b00100001;             // RB0 and RB5 pbutton
	TRISC = 0b00011000;             // RC3 and RC4 do the I2C bus
	TRISG = 0b11111111;             // RG0, RG1 & RG3 pbutton
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
	INTCONbits.TMR0IF = 0;          // Clear timer flag
	//T0CON = 0b00000011;           // Prescale by 16
	T0CON = 0b00001000;             // No prescale
	TMR0H = 0;                      // Clear timer count
	TMR0L = 0;
	T0CONbits.TMR0ON = 1;           // Start timer
	OpenI2C( MASTER, SLEW_OFF);
	SSPADD = 0x3F;
}
//
// end Init ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


/*
 * Write an individual LCD segment.
 *
 * @param segOrd The segment ordinal.  Between 0 and 22.
 *
 * @param state Whether to turn the segment dark (true) or clear (false).
 *
 */
void segWrt(unsigned char segOrd,  unsigned char state) {

	unsigned char bitSelect;
	unsigned char *LCReg;

	if (segOrd > 23) return;
	LCReg = (unsigned char *)&LCDDATA0 + (segOrd >> 3);
	bitSelect = 1 << (segOrd & 0x07);
	if (state) *LCReg  |=  bitSelect;  // Segment on
	else *LCReg &= ~bitSelect;         // Segment off

	// The above is an optimized implementation of concisely writing
	// a segment to the appropriate bit onto the right LCDDATA register
	// which can be LCDDATA0, LCDDATA1 (accessed by LCDDATA0+1), or
	// LCDDATA2 (accessed by LCDDATA0+2).
	// More explicit (and readable, but slower) logic is:
	/*
	   if (segOrd==0)      { LCReg=&LCDDATA0; bitSelect=0b00000001; }
	   else if (segOrd==1) { LCReg=&LCDDATA0; bitSelect=0b00000010; }
	   else if (segOrd==2) { LCReg=&LCDDATA0; bitSelect=0b00000100; }
	   ...
	   else if (segOrd==7) { LCReg=&LCDDATA0; bitSelect=0b10000000; }
	   else if (segOrd==8)  { LCReg=&LCDDATA0+1; bitSelect=0b00000001; }
	   else if (segOrd==9)  { LCReg=&LCDDATA0+1; bitSelect=0b00000010; }
	   else if (segOrd==10) { LCReg=&LCDDATA0+1; bitSelect=0b00000100; }
	   ...
	   else if (segOrd==15) { LCReg=&LCDDATA0+1; bitSelect=0b10000000; }
	   else if (segOrd==16)  { LCReg=&LCDDATA0+2; bitSelect=0b00000001; }
	   else if (segOrd==17)  { LCReg=&LCDDATA0+2; bitSelect=0b00000010; }
	   ...
	   else if (segOrd==22)  { LCReg=&LCDDATA0+2; bitSelect=0b01000000; }

	   if (state) *LCReg |= bitSelect; // Paste 1 to the selected bit in LCReg
	   else *LCReg &= ~bitSelect;      // Mask out the selected bit from LCReg
	*/
}
//
// end segWrt ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//




/*
 * Write an individual LCD segment.
 *
 * @param segOrd The segment ordinal.  Between 0 and 22.
 *
 * @param state Whether to turn the segment dark (true) or clear (false).
 *
 */
unsigned int display(unsigned int freq){

    int dA[4], limit = 1;
    unsigned int pos = -1;
    //store each digit in an array
    dA[3] = freq/1000;
    dA[2] = (freq - (dA[3]*1000))/100;
    dA[1] = (freq-(dA[3]*1000)- (dA[2]*100))/10;
    dA[0] = (freq-(dA[3]*1000)- (dA[2]*100)-(dA[1]*10));


    clrscn(); //Clears the screen;

		if (freq > 22)
		{
			segWrt(22, 1);//light up the decimal if it is not a volume level
			limit = 2;
		}


    do {
        pos++;
        switch (dA[pos]) {
            case 0:
                segWrt(0 + pos*7, 1);
                segWrt(1 + pos*7, 1);
                segWrt(2 + pos*7, 1);
                segWrt(3 + pos*7, 1);
                segWrt(4 + pos*7, 1);
                segWrt(5 + pos*7, 1);
            case 1:
                segWrt(1 + pos*7, 1);
                segWrt(2 + pos*7, 1);
            case 2:
                segWrt(0 + pos*7, 1);
                segWrt(2 + pos*7, 1);
                segWrt(3 + pos*7, 1);
                segWrt(4 + pos*7, 1);
                segWrt(6 + pos*7, 1);
            case 3:
                segWrt(0 + pos*7, 1);
                segWrt(1 + pos*7, 1);
                segWrt(2 + pos*7, 1);
                segWrt(3 + pos*7, 1);
                segWrt(6 + pos*7, 1);
            case 4:
                segWrt(1 + pos*7, 1);
                segWrt(2 + pos*7, 1);
                segWrt(5 + pos*7, 1);
                segWrt(6 + pos*7, 1);
            case 5:
                segWrt(0 + pos*7, 1);
                segWrt(2 + pos*7, 1);
                segWrt(3 + pos*7, 1);
                segWrt(5 + pos*7, 1);
                segWrt(6 + pos*7, 1);
            case 6:
                segWrt(0 + pos*7, 1);
                segWrt(2 + pos*7, 1);
                segWrt(3 + pos*7, 1);
                segWrt(4 + pos*7, 1);
                segWrt(5 + pos*7, 1);
                segWrt(6 + pos*7, 1);
            case 7:
                segWrt(0 + pos*7, 1);
                segWrt(1 + pos*7, 1);
                segWrt(2 + pos*7, 1);
            case 8:
                segWrt(0 + pos*7, 1);
                segWrt(1 + pos*7, 1);
                segWrt(2 + pos*7, 1);
                segWrt(3 + pos*7, 1);
                segWrt(4 + pos*7, 1);
                segWrt(5 + pos*7, 1);
                segWrt(6 + pos*7, 1);
            case 9:
                segWrt(0 + pos*7, 1);
                segWrt(1 + pos*7, 1);
                segWrt(2 + pos*7, 1);
                segWrt(3 + pos*7, 1);
                segWrt(5 + pos*7, 1);
                segWrt(6 + pos*7, 1);
        }
    } while (pos < limit);

		if (freq>22)
		{
	    switch (dA[3]) {
	        case 0: // do nothing
	        case 1:
	            segWrt(21, 1);
	    }
		}

    return XS;
}


/*
 * FMwrite() -  Write a two byte word to the FM module.  The new
 * register contents are obtained from the image bank.
 * The content must be ready in regImg[adr] before calling FMwrite(adr).
 *
 * @param adr The address of the register in the FM module that needs
 * to be written.
 *
 *
 * @return XS on success or XF on error.
 *
 */


unsigned char FMwrite(unsigned char adr) {

	unsigned int  regstr;
	unsigned char firstByt;
	unsigned char secndByt;
	unsigned char rpy;

	firstByt = regImg[adr] >> 8;
	secndByt = regImg[adr];

	StartI2C();				// Begin I2C communication
	IdleI2C();

	// Send slave address of the chip onto the bus
	if (WriteI2C(FMI2CADR)) return XF;
	IdleI2C();
	WriteI2C(adr);				// Adress the internal register
	IdleI2C();
	WriteI2C(firstByt);			// Ask for write to FM chip
	IdleI2C();
	WriteI2C(secndByt);
	IdleI2C();
	StopI2C();
	IdleI2C();
	return XS;
}
//
// end FMwrite ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//



/*
 * FMread - Read a two byte register from the FM module.
 *
 * @param regAddr The address of the register in the module that needs
 *        to be read.
 *
 * @param data Where to store the reading.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMread(unsigned char regAddr, unsigned int *data) {

	unsigned char firstByt;
	unsigned char secndByt;

	StartI2C();					// Begin I2C communication
	IdleI2C();					// Allow the bus to settle

	// Send address of the chip onto the bus
	if (WriteI2C(FMI2CADR)) return XF;
	IdleI2C();
	WriteI2C(regAddr);			// Adress the internal register
	IdleI2C();
	RestartI2C();				// Initiate a RESTART command
	IdleI2C();
	WriteI2C(FMI2CADR + DEVRD);	// Ask for read from FM chip
	IdleI2C();
	firstByt = ReadI2C(); 		// Returns the MSB byte
	IdleI2C();
	AckI2C();					// Send back Acknowledge
	IdleI2C();
	secndByt = ReadI2C();		// Returns the LSB of the temperature
	IdleI2C();
	NotAckI2C();
	IdleI2C();
	StopI2C();
	IdleI2C();
	*data = firstByt;
	*data <<= 8;
	*data = *data | secndByt;

	return XS;
}
//
// end FMread ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//



/*
 * FMready - See if the FM module is ready.
 *
 * @param rdy Where to store the busy/ready status.  Will become
 * non-zero if the chip is ready, zero if busy.
 *
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMready(unsigned int *rdy) {

	unsigned int sts;

	if (FMread(FMCHIPSTSADR, &sts)  != XS) return XF;
	sts &= FMASKSTATUS;
	*rdy = sts ? TRUE : FALSE;
	return XS;
}
//
// end FMready ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


/*
 * FMinit() -  Initialise the FM module.
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMinit() {

	unsigned char ad;
	unsigned int dat;

	// Copy default FM register values to the image set -
	for(ad = 0; ad < 18; ad++) regImg[ad] = regDflt[ad];

	dat = regImg[0];
	regImg[0] &= ~1;
	if (FMwrite(0) != XS) return  XF;
	for(ad = 1; ad < 18; ad++) {
		if (FMwrite(ad) != XS)return XF;
	}

	regImg[0] = dat | 1;
	if (FMwrite(0) != XS) return XF;
	dly(20);
	while (FMready(&dat), !dat) dly(2);
	showFreq();
	return XS;
}
//
// end FMinit ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//




/*
 * FMfrequenc(f) -  Tune the FM module to new frequency.
 *
 *
 * @param f The new frequency as a multiple of 100 kHz.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMfrequenc(unsigned int f) {

	unsigned int dat;
	unsigned int cn;		// AR1010 channel number

	cn = f - 690;


	// NB AR1010 retunes on 0 to 1 transition of TUNE bit -
	regImg[2] &= ~FMASKTUNE;           // Mask out bit D9 in R2 (see FMASKTUNE)
	if (FMwrite(2) != XS) return XF;   // Write R2 to AR1010
	regImg[2] &= 0xfe00;               // Clear bits D8-D0 in R2
	regImg[2] |= (cn | FMASKTUNE);     // Paste 1 to bit D9 (see FMASKTUNE)
	                                   //   and also paste 'cn' onto D8-D0 in R2
	if (FMwrite(2) != XS) return XF;   // Write R2 to AR1010
	do {
		dly(2);
		if (FMready(&dat) != XS) return XF;
	} while (!dat);

    return XS;
}
//
// end FMfrequenc ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//


/*
 * FMvers - Obtain the FM chip version.
 *
 * @param vsn Where to store the version number.  Will become
 * 0x65B1 for vintage 2009 devices.
 *
 * @return XS on success or XF on error. *
 */
unsigned char FMvers(unsigned int *vsn) {
	if (FMread(FMCHIPVERSADR, vsn)  != XS) return XF;
	return XS;
}
//
// end FMvers ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


/*
 * FMid - Obtain the FM chip ID.
 * * @param id Where to store the ID number.  Will become
 * 0x1010 for AR1010 devices.
 *
 * @return XS on success or XF on error. *
 */
unsigned char FMid(unsigned int *id) {

	if (FMread(FMCHIPIDADR, id)  != XS) return XF;
	return XS;
}
//
// end FMid ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//




/*	// Checks
 * errfm() -  Firmware error.   Call this on a showstopper.
 *
 *
 * @return Never!
 *
 */
void errfm() {

	;		// Do something helpful
	for(;;) ;
}
//
// end errfm ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//



/*
 * Display the frequency that the receiver chip is set to.
 *
 * @return XS if successful, else XF on failure.
 *
 */
unsigned char showFreq() {

    unsigned int ch;

    if (ch != FMASKRDCHAN)	// Checks current channel;
        display(ch + 690);
    return XS;

}
//
// end showFreq ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//



// NEW FUNCTIONS ##########################################################




/*
 * nextChan() -  Tune to the next channel.
 *
 * @param up Set to non-zero for next channel up,
 *  zero for preset down.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char nextChan(unsigned char up) {

	unsigned int chan = FMASKRDCHAN;		// Checks current channel

	if (up) {				// Next channel case
		chan++;
		if (chan > FMHIGHCHAN) chan = FMLOWCHAN;	// "Overflow"
		FMfrequenc(chan + 690);				// Tune FM module to new frequency
        showFreq();
		return XS;
	}

	else {					// Previous channel case
		chan--;
		if (chan < FMLOWCHAN) chan = FMHIGHCHAN;	// "Overflow"
		FMfrequenc(chan + 690);				// Tune FM module to new frequency
        showFreq();
		return XS;
	}

};
//
// end nextChan ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//


/*
 *	Increase the volume.
 *
 *	@param vup set to non zero for volume up, zero for volume down.
 *
 * @return XS if successful, else XF on failure.
 *
 */
unsigned char volUp(unsigned char vup) {

		// Array of codes for registers at each volume level
		// Position in the array correspondes to Volume level
    // It is: regImg[3]=arr[4]; regImg[14]=arr[2*4]; will maps to volume level of 4
		unsigned char arr[44] = {
														 0xf, 0xf, 0xf, 0xf, 0xf, 0xe, 0xe, 0xd, 0xd, 0xb, 0xa,
	                           0x9, 0x7, 0x6, 0x6, 0x5, 0x5, 0x4, 0x4, 0x2, 0x1, 0x0,
	                           0x0, 0xc, 0xd, 0xe, 0xf, 0xe, 0xf, 0xe, 0xf, 0xf, 0xf,
                      			 0xf, 0xf, 0xe, 0xf, 0xe, 0xf, 0xe, 0xf, 0xf, 0xf, 0xf
													 	};
		short temp;

		// vup case - changes global variable "volume", which is used later
		if(vup)
   	{
    	if (volume = 22)
		    return XS;
			volume++;
   	}
	  else
   	{
     	if (volume = 0)
	      return XS;
			volume--;
   	}

		// Writing part

		// VOLUME
		regImg[3] &= FMASKNOVOL;  		// Clear bits D7-D10 in R3
		temp = arr[volume];						// Assign code for this volume lvl
		temp << 7;										// Shift to match position of VOLUME2 registers
		regImg[3] + temp;							// Add temp to register - fills the gap of VOLUME registers
		if (FMwrite(3) != XS) return XF;   // Write R14 to AR1010

		// VOLUME2
		regImg[14] &= FMASKNOVOL2;  	// Clear bits D12-D15 in R14
		temp = arr[2*volume];					// Assign code for this volume lvl
		temp << 12;										// Shift to match position of VOLUME2 registers
		regImg[14] + temp;						// Add temp to register - fills the gap of VOLUME2 registers
		if (FMwrite(14) != XS) return XF;   // Write R14 to AR1010

		return XS;
}

//
// end volUp ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//



/*
 *	Run the test for segments in the display.
 *
 * @return XS if successful, else XF on failure.
 *
 */
unsigned char segTest(/*unsigned char test_arg*/) {
    int i = 0;
	unsigned char *FILLptr;        // Pointer used to fill all LCDDATA
	for (	i = 0,
			FILLptr = (unsigned char *) &LCDDATA0;  // Point to first segment
			i < 28;
			i++)		// Turn on all segments
		*FILLptr++ = 1;

    dly(1000);       // Waits a bit
    clrscn();       // Clears the screen
    return XS;
};
//
// end segTest ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//



/*
 *	Change to the next preseted channel.
 *
 *	@param next set to non zero for next preseted channel up,
 *	zero for preseted channel down.
 *
 * @return XS if successful, else XF on failure.
 *
 */
unsigned char preSet(unsigned char next){

	unsigned int chn;
    unsigned int frq[] = {880, 964, 977, 1046};      // Frequencies

    if (!(topChn>=0))	// If not initialised case
    	int topChn = sizeof(frq) / sizeof(unsigned int) - 1;

	if (!((chn >= 0)&&(chn <= topChn))) chn = 0;      // Initialise channel no.

	if (next) {					// Next pre-set channel case
		chn++;						// Increment channel numbers
		if (chn > topChn) chn = 0;	// "Overflow"
		FMfrequenc(frq[chn]);				// Tune FM module to new frequency
        showFreq();
		return XS;

    }else {							// Previous pre-set channel case
		chn--;						// Increment channel numbers
		if (chn < 0) chn = topChn;	// "Overflow"
		FMfrequenc(frq[chn]);				// Tune FM module to new frequency
        showFreq();
		return XS;
	}
};
//
// end preSet ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


void main(void) {

	unsigned char btn;
	unsigned char evt;
	unsigned int ui;
    unsigned int cf = 964; //INitialised to Eagle Radio

	dly(20);
	Init();
	FMvers(&ui);     // Check we have comms with FM chip
	if (ui != 0x1010) errfm();
	if (FMinit() != XS) errfm();
    FMfrequenc(cf);

	for (;;) {
        display(786); //check for decimals
		evt = butnEvent(&btn);
		if (evt == 1) switch (btn) {
            case 0 : nextChan(TRUE); break;				// Next channel (in UK band)
            case 1 : nextChan(FALSE); break; 			// Prev channel (in UK band)
            case 2 : preSet(TRUE); break;				// Next preset channel
            case 3 : preSet(FALSE); break;				// Previous preset channel
            case 4 : volUp(TRUE); break;				// Increase volume
            case 5 : volUp(FALSE); break;				// Decrease volume
            case 6 : segTest(TRUE); break;				// Run the test
            case 7 : display(volume); break;					// Display volume level
            default : break;
		}
	}
}
//
// end main ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
