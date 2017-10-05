void ESCArming_prot(void)
{
	TA1CCR2 = ESCmax;

	__delay_cycles(6000000);
	TA1CCR2 = ESCmin;					// ESC full reverse

	__delay_cycles(3000000);
	TA1CCR2 = ESCneutral;					// ESC neutral

	__delay_cycles(5000000);
}
