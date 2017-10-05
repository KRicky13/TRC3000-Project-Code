void BlinkXXTimes(int XX, int whichLED)
{
	int j;
	for(j=0; j<XX; j++)
	{
		P1OUT |= whichLED;
		__delay_cycles(125000);
		P1OUT &= ~whichLED;
		__delay_cycles(125000);
	}
}
