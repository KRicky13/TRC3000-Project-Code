double magnetometer(void)
{
	//local variables//
	char read_buffer[7] = { 0 };
	int headdiff = 0;
	int heading = 0;

	////
	if (write(file, &start_addr, 1) != 1) {
		printf("Failed to change address pointer to 0x03.\n"); exit(1);
	}

	if (read(file, read_buffer, 7) != 7)
	{
		printf("Something's wrong, received data length is not 7\n");
	}

	x = ((read_buffer[0] << 8) | read_buffer[1]);
	z = ((read_buffer[2] << 8) | read_buffer[3]);
	y = ((read_buffer[4] << 8) | read_buffer[5]);


	a = (0x8000 & x ? (int)(0x7FFF & x) - 0x8000 : x);
	b = (0x8000 & y ? (int)(0x7FFF & y) - 0x8000 : y);
	c = (0x8000 & z ? (int)(0x7FFF & z) - 0x8000 : z);


	heading = atan2(b, a)*(180 / 3.14159);
	heading = (heading + 180);



	return heading;
}