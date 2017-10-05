void SwingSteering_prot(void)
{
	for(;;)
	{
		while(TA1CCR1<SteeringFullRight)		// Swing right
		{
			__delay_cycles(2000);
			TA1CCR1++;
		}
		while(TA1CCR1>SteeringFullLeft)
		{
			__delay_cycles(2000);	// Swing left
			TA1CCR1--;
		}
	}
}

