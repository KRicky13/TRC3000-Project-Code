void UARTSetup(void)
{
    //_____________Open/Configure_________________________//
	// --------------Uart -----------------------------//
	struct termios btPortOptions, lpPortOptions; // struct to hold the port settings

	// open launchpad and bluetooth ports as read/write, not as controlling terminal, and
	// don't block the CPU if it takes too long to open the port.
	lpPort = open(LaunchPort, O_RDWR | O_NOCTTY | O_NDELAY);
	if (lpPort == -1)
		printf("Failed to open Launchpad port\n");

	btPort = open(BluePort, O_RDWR | O_NOCTTY | O_NDELAY);
	if (btPort == -1)
		printf("Failed to open Bluetooth port\n");

	// Fetch the current port settings
	tcgetattr(lpPort, &lpPortOptions);
	tcgetattr(btPort, &btPortOptions);

	// Flush the port's buffers (in and out) before we start using it
	tcflush(lpPort, TCIOFLUSH);
	tcflush(btPort, TCIOFLUSH);

	// Set the input and output baud rates
	//Launchpad
	cfsetispeed(&lpPortOptions, B9600);
	cfsetospeed(&lpPortOptions, B9600);
	//Bluetooth
	cfsetispeed(&btPortOptions, B9600);
	cfsetospeed(&btPortOptions, B9600);


	//	c_cflag contains a few important things- CLOCAL and CREAD, to prevent
	//	this program from "owning" the port and to enable receipt of data.
	//	Also, it holds the settings for number of data bits, parity, stop bits,
	//	and hardware flow control.

	//Launchpad
	lpPortOptions.c_cflag |= CLOCAL;
	lpPortOptions.c_cflag |= CREAD;
	// Set up the frame information.
	lpPortOptions.c_cflag &= ~CSIZE; // clear frame size info
	lpPortOptions.c_cflag |= CS8;    // 8 bit frames
	lpPortOptions.c_cflag &= ~PARENB;// no parity
	lpPortOptions.c_cflag &= ~CSTOPB;// one stop bit

	//Bluetooth
	btPortOptions.c_cflag |= CLOCAL;
	btPortOptions.c_cflag |= CREAD;
	// Set up the frame information.
	btPortOptions.c_cflag &= ~CSIZE; // clear frame size info
	btPortOptions.c_cflag |= CS8;    // 8 bit frames
	btPortOptions.c_cflag &= ~PARENB;// no parity
	btPortOptions.c_cflag &= ~CSTOPB;// one stop bit


	// push it back to the system.
	tcsetattr(lpPort, TCSANOW, &lpPortOptions);
	tcsetattr(btPort, TCSANOW, &btPortOptions);

	tcflush(lpPort, TCIOFLUSH);
	tcflush(btPort, TCIOFLUSH);

	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

	tcflush(lpPort, TCIOFLUSH);
	tcflush(btPort, TCIOFLUSH);

	printf("\nUART port to Launchpad and Bluetooth port setup complete.\n");
}