#include <MSP430G2553.h>

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1
#define PB BIT3
#define ESCPWM BIT1
#define STEERPWM BIT4
#define CAMPWM BIT6

// Global variable definitions
unsigned int i, a, current; //Counter
unsigned int x;
int count = 0;
int camflag = 0;
char dir;
char temprx;
char* string = "I got ";
unsigned char top4;
unsigned char bottom4;

// Hardware parameters
const int period = 20000;
const int ESCmax = 1900;
const int ESCmin = 990;
const int ESCneutral = 1500;
const int SteeringFullLeft = 1950;
const int SteeringFullRight = 950;
const int SteeringMid = 1500;
const int CameraFullRight = 2000;
const int CameraFullLeft = 1000;
const int CameraMid = 1500;

// Function prototypes
void BlinkXXTimes(int XX, int whichLED);
void PinSetup(void);
void ESCArming_prot(void);
void Heading_prot(void);
void SwingSteering_prot(void);

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
	DCOCTL = 0; 					// Select lowest DCOx and MODx settings<
	BCSCTL1 = CALBC1_1MHZ; 			// Set DCO
	DCOCTL = CALDCO_1MHZ;

	PinSetup();
	__enable_interrupt(); 			// enable all interrupts

    //// Various protocols can be toggled on or off as desired
    //// e.g. ESC only needs to be armed once and can be 
    //// deactivated after arming once
	//ESCArming_prot();
	//Heading_prot();
	//SwingSteering_prot();
	TA0CCR1 = CameraMid;            // Default camera and wheel steering
	TA1CCR1 = SteeringMid;          // to middle values

	while (temprx != 0x99)          // temprx variable stores the contents of the 
                                    // RX buffer on interrupts (see USCI0RX_ISR)
	{
		P1OUT = TXLED + RXLED;
		__bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/ int until Byte RXed
		P1OUT &= ~(TXLED + RXLED);

        // Split temprx to 4-bits for control: upper 4 bits are for setting the
        // control function, lower 4 bits designate the degree to which the 
        // steering/camera is to be modulated
		top4 = 0xF0 & temprx;
		bottom4 = 0x0F & temprx;

		switch (top4)
		{
		case 0x20:	// Right turn at different magnitudes (Green track)
			TA1CCR1 = SteeringMid - bottom4*50;
			TA1CCR2 = ESCneutral + 70;
			//TA0CCR1 = CameraMid;
			break;
		case 0x30:	// Left turn at different magnitudes (Green track)
			TA1CCR1 = SteeringMid + bottom4*50;
			TA1CCR2 = ESCneutral + 70;
			//TA0CCR1 = CameraMid;
			break;
		case 0x40:	// Cam Servo: Left turn at different magnitudes (Heading)
			TA0CCR1 = CameraMid + bottom4*100;
			break;
		case 0x50:	// Nothing in camera
			TA1CCR2 = ESCneutral + 75;
			TA1CCR1 = SteeringMid + bottom4*50; // Slow turn left
			break;
		case 0x60:	// Stop when heading obtained
			TA1CCR1 = SteeringMid;
			TA1CCR2 = ESCneutral;
			break;
		case 0x70:	// Right turn at different magnitudes (Orange track)
			TA1CCR1 = SteeringMid - bottom4*50;
			TA1CCR2 = ESCneutral + 160;
			//TA0CCR1 = CameraMid;
			break;
		case 0x80:	// Left turn at different magnitudes (Orange track)
			TA1CCR1 = SteeringMid + bottom4*50;
			TA1CCR2 = ESCneutral + 160;
			//TA0CCR1 = CameraMid;
			break;
		/*case 0x00:
			TA1CCR2 = ESCneutral;
			TA1CCR1 = SteeringMid;
			break;*/
		default:
			TA1CCR1 = SteeringMid;
			TA1CCR2 = ESCneutral;
			BlinkXXTimes(2, RXLED + TXLED);
			break;
		}
	}

	while (dir != 83)
	{
		while (!(IFG2 & UCA0RXIFG)) P1OUT &= ~TXLED;
		P1OUT ^= TXLED;
		//TA1CCR2 = ESCneutral + 50;
		//___________________________________________________________//
		while (dir == 77) //'M'
		{
			TA1CCR1 = SteeringMid;
			if (dir != 82) break;
			TA1CCR2 = ESCneutral + 50;
			//P1OUT ^= TXLED;
			__delay_cycles(500000);
			if (dir != 82) break;
			TA1CCR2 = ESCneutral;
			__delay_cycles(500000);
			//if (dir != 77) break;

			if (TA1CCR1 < SteeringMid - 25)
				for (TA1CCR1; TA1CCR1 > SteeringMid - 25; TA1CCR1++)
				{
					if (dir != 77) break;
					__delay_cycles(1000);
				}
			else if (TA1CCR1 > SteeringMid + 25)
				for (TA1CCR1; TA1CCR1 < SteeringMid + 25; TA1CCR1--)
				{
					if (dir != 77) break;
					__delay_cycles(1000);
				}//
		}
		//___________________________________________________________//
		while (dir == 76) //'L'
		{
			//TA1CCR1 = SteeringFullLeft;
			P1OUT &= ~RXLED;
			P1OUT |= TXLED;
			for (TA1CCR1; TA1CCR1 < SteeringFullLeft; TA1CCR1++)
			{
				if (dir != 76) break;
				__delay_cycles(2500);
			}
			if (dir != 82) break;
			TA1CCR2 = ESCneutral + 50;
			__delay_cycles(500000);
			if (dir != 82) break;
			TA1CCR2 = ESCneutral;
			__delay_cycles(500000);
		}
		//___________________________________________________________//
		while (dir == 82) //'R'
		{
			//TA1CCR1 = SteeringFullRight;
			P1OUT &= ~TXLED;
			P1OUT |= RXLED;
			for (TA1CCR1; TA1CCR1 > SteeringFullRight; TA1CCR1-=0.1)
			{
				if (dir != 82) break;
				__delay_cycles(1000);
			}
			if (dir != 82) break;
			TA1CCR2 = ESCneutral + 50;
			__delay_cycles(500000);
			if (dir != 82) break;
			TA1CCR2 = ESCneutral;
			__delay_cycles(1000000);
		}
	}

	TA1CCR1 = SteeringMid;
	TA1CCR2 = ESCneutral - 50;
	__delay_cycles(450000);
	TA1CCR2 = ESCneutral;
	for(;;)
		BlinkXXTimes(1, RXLED+TXLED);
	/*UC0IE |= UCA0TXIE;		// Once heading is obtained, transmit a
	UCA0TXBUF = 1;			// signal to BBB*/
}

// TX interrupt service routine
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
	P1OUT ^= TXLED;
	UCA0TXBUF = top4;
	UC0IE &= ~UCA0TXIE;
	P1OUT ^= TXLED;
}

// RX interrupt service routine
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	P1OUT ^= RXLED;
	temprx = UCA0RXBUF;
	/*if (temprx==82)
		dir = 'R';
	else if (temprx==76)
		dir = 'L';
	else if (temprx==77)
		dir = 'M';
	else if (temprx==83)
		dir = 'S';
	else if (temprx==114)
		dir = 'r';
	else if (temprx==108)
		dir = 'l';
	else if (temprx==109)
		dir = 'm';
	else if (UCA0RXBUF==102)
		TA1CCR2 = ESCneutral + 50;
	else
		dir = 0;*/
	//UC0IE |= UCA0TXIE;		// Once heading is obtained, transmit a
	//UCA0TXBUF = 1;			// signal to BBB
	P1OUT ^= RXLED;
	__bic_SR_register_on_exit(CPUOFF);
}

// Port 1 interrupt service routine
// Configures the ESC
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
	__delay_cycles(30000);	// Switch debounce delay

	count++;
	switch (count) {
	case 1:					// ESC Full throttle p_on
		TA1CCR2 = ESCneutral;
		BlinkXXTimes(1, RXLED);
		break;
	case 2:					// ESC Full reverse p_on
		TA1CCR2 = ESCneutral + 49;
		BlinkXXTimes(2, RXLED);
		break;
	/*case 3:					// ESC Neutral p_on
		TA1CCR2 = ESCneutral + 44;
		BlinkXXTimes(count, RXLED);
		break;
	case 4:					// ESC Neutral p_on
		TA1CCR2 = ESCneutral + 46;
		BlinkXXTimes(count, RXLED);
		break;
	case 5:					// ESC Neutral p_on
		TA1CCR2 = ESCneutral + 48;
		BlinkXXTimes(count, RXLED);
		break;*/
	default:				// ESC Minimal forward throttle
		TA1CCR2 = ESCneutral + 50;
		count = 0;
		break;
	}

	P1IFG &= ~BIT3; // P1.3 IFG cleared
}

