void PinSetup(void)
{
	// Defaults unused pins to be outputs to reduce power consumption
	P1SEL &= 0x00;
	P1SEL2 &= 0x00;
	P1DIR |= 0xFF;
	P1OUT = 0x00;
	P2SEL &= 0x00;
	P2SEL2 &= 0x00;
	P2DIR |= 0xFF;
	P2OUT = 0x00;

	// Pushbutton interrupt setup
	P1DIR |= TXLED;					// P1.0 output to LED
	P1DIR &= ~PB;					// P1.3 input from pushbutton
	P1OUT |= BIT3;					// Initialize P1.3 as high
	P1SEL |= RXD + TXD; 			// P1.1 = RXD, P1.2=TXD, P1.6 = Camera PWM
	P1SEL2 |= RXD + TXD; 			// P1.1 = RXD, P1.2=TXD
	P1REN |= BIT3;
	P1IE |= BIT3; 					// P1.3 interrupt enabled
	P1IFG &= ~BIT3; 				// P1.3 IFG cleared
	P1IES &= ~BIT3;					// Interrupt Lo-Hi

	//UART pins setup
	UCA0CTL1 |= UCSSEL_2; 			// SMCLK
	UCA0BR0 = 104; 					// 1MHz 9600 baud rate
	UCA0BR1 = 0x00;
	UCA0MCTL = UCBRS0; 				// Modulation UCBRSx = 5
	UCA0CTL1 &= ~UCSWRST; 			// **Initialize USCI state machine**
	UC0IE |= UCA0RXIE; 				// Enable USCI_A0 RX interrupt

	// PWM Signals: 2.1 ESC, 2.4 Steering angle, 1.6 Camera pan
	P1DIR |= CAMPWM;				// P1.6 output as camera PWM
	P1SEL |= CAMPWM;				// P1.6 TA0.1 option
	TA0CCTL1 = OUTMOD_7;			// CCR1 reset/set
	TA0CCR0 = period;				// PWM Period
	TA0CCR1 = CameraMid;			// Camera PWM duty cycle
	TA0CTL = TASSEL_2 + MC_1;		// SMCLK, up mode

	P2DIR |= ESCPWM + STEERPWM;		// P2.1 and P2.4 output
	P2SEL |= ESCPWM + STEERPWM;		// P2.1 and P2.4 TA1.1/2 options
	TA1CCTL1 = OUTMOD_7;			// CCR1 reset/set
	TA1CCTL2 = OUTMOD_7;			// CCR2 reset/set
	TA1CCR0 = period;				// PWM Period
	TA1CCR1 = SteeringMid;			// 2.2 Steering PWM duty cycle
	TA1CCR2 = ESCneutral;			// 2.4 ESC PWM duty cycle
	TA1CTL = TASSEL_2 + MC_1;		// SMCLK, up mode
}
