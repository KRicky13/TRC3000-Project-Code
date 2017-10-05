void Heading_prot(void)
{
	/*while (!temprx)
	{
		P1OUT ^= TXLED;
		__delay_cycles(250000);
	}*/

	//P1OUT &= ~TXLED;
	while (temprx == 'R' || temprx == 'L')
	{
		if (temprx == 'R')
		{
			TA1CCR1 = SteeringFullRight;
			TA1CCR2 = ESCneutral + 49;
		}
		else if (temprx == 'L')
		{
			TA1CCR1 = SteeringFullLeft;
			TA1CCR2 = ESCneutral + 49;
		}
	}
	TA1CCR1 = SteeringMid;
	TA1CCR2 = ESCneutral - 50;
	__delay_cycles(750000);
	TA1CCR2 = ESCneutral;
	BlinkXXTimes(5, TXLED);
}
