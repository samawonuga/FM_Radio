//
// FILE     fm.h
// DATE     140128
// WRITTEN  RAC
// PURPOSE  Header for PIC firmware .   $
// LANGUAGE MPLAB C18
// KEYWORDS FM/RECEIVE/EXPERIMENT/PROJECT/TEACH/I2C/PIC
// PROJECT  FM TxRx Lab experiment
// CATEGORY FIRMWARE
// TARGET   PIC18F6490
//
//
//
// See also -
//
//  http://read.pudn.com/downloads142/sourcecode/embed/615356/AR1000FSamplev085.c__.htm
//  

// This is skeletal code which won't run until 'fleshed out'.
// It does give an idea of the setup and initialization required
// of the PIC and the FM tuner module.  The basics of communication
// over the I2C bus are also covered.
// 
//
//


#define XS                      0       // Exit success
#define XF                      1       // Exit fail

#define FMI2CADR                0x20    // Address (for writes) of FM module on I2C bus


#define DEVRD                   0x01    // Read not write an I2C device 
#define FMCHIPVERSADR           0x1C    // Address of FM chip version
#define FMCHIPIDADR             0x1B    // Address of FM chip ID  
#define FMCHIPSTSADR            0x13    // Address of FM chip status

#define FMASKMUTE		0x0001		// Register 1, bit 1
#define FMASKTUNE		0x0200		// Register 2, bit 9
#define FMASKSTATUS		0x0020		// Register 0x13, bit 5
#define FMASKSEEK		0x4000		// Register 3, bit 14
#define FMASKRDCHAN		0xFF80		// Register 2, channel number bits

#define FMASKVOLUME     0x0780      // Register 3, bits 7-10
#define FMASKVOLUME2    0xf000      // Register 14, bits 12-15
#define FMASKNOVOL      0xf87f      // Register 3, bits 11-15 and 0-6

#define BUTN1			0b00000001	// Button number one
#define BUTN2			0b00000010	// Button number two
#define BUTN3			0b00000100
#define BUTN4			0b00001000
#define BUTN5			0b00010000
#define BUTN6			0b00100000
#define BUTN7			0b01000000
#define BUTN8			0b10000000

#define BTN1            PORTBbits.RB0
#define BTN2            PORTBbits.RB5
#define BTN3            PORTAbits.RA0
#define BTN4            PORTAbits.RA1
#define BTN5            PORTGbits.RG0
#define BTN6            PORTGbits.RG1
#define BTN7            PORTGbits.RG2
#define BTN8            PORTGbits.RG3

#define LCDSEGDP3		22			// LCD segment for decimal point
#define LCDSEGZ 		23			// LCD segment for Z

#define FMHIGHCHAN		(1080-690)	// Highest FM channel number
#define FMLOWCHAN		(875-690)
#define FALSE			0
#define TRUE			1

enum {							// Global error numbers
	GERNONE, 					// No error
	GERWCOL,					// I2C write collision
	GERFINT,					// Could not initialize FM module
	GERFMID						// Could not read chip ID (0x1010)
};

void Init();								// Processor initialisation.
void dly(int d);
unsigned char FMread(unsigned char regAddr, unsigned int *data);
unsigned char FMwrite(unsigned char adr);				// Write a new value to a register
unsigned char FMinit();									// Initialise the chip
unsigned char FMready(unsigned int *rdy);				// Status is ready or busy
unsigned char FMid(unsigned int *id);					// Obtain ID number
unsigned char showFreq(void);						// Display the current f in MHz
unsigned char FMvers(unsigned int *vsn);				// Obtain version number



//
// end fm.h ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//
