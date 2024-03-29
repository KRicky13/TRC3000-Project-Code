## Source code for TRC3000: Mechatronics Project II (Monash University, 2015 Semester 2)

### Project objective
To navigate a model race-car autonomously 3 laps around a 10m x 2m [LxW] track

![Diagram of course track](TRACK.png)

### System specifications
- BeagleBone Black (BBB) processor
- 16-bit TI Launchpad MSG2553 microcontroller
- Electronic speed control (ESC)
- Model race-car (approx. 40cm x 25cm x 20cm [LxWxH])
- Magnetometer
- Logitech USB webcam
- Bluetooth transceiver

### System functionalities
- PWM speed control
- PWM steering control
- Computer vision (OpenCV)
- Bluetooth-triggered start
- Automated ESC arming
- Heading detection (magnetometer)

The microcontroller interfaces to the hardware (ESC, servos, sensors, NOT incl. webcam) and transmits data to the processor. The processor utilizes the microcontroller data, in combination with visual data from the webcam, to output digitally-modulated control signals to the microcontroller to navigate around the track. System terminates itself after 3 laps.

### BeagleBone Black source code
- `bubblesort.c`
  - sorting algorithm used for sorting the contours obtained from the processed image in ascending order
- `choosecolour.c`
  - stores the desired RGB-colour threshold values into an integer array
- `create_trackbars.c:`
  - function used for debugging purposes; produces trackbars that alter the parameters of the thresholded image
- `FULL-FAST-22-OCT.c`
  - final complete code for the PROCESSOR used on project day!
- `magnetometer.c`
  - function call to the magnetometer that returns the heading of the model car in the range between [0-360) deg.
- `UARTSetup.c`
  - function macro to configure the necessary UART ports

### Launchpad source code
- `BlinkXXTimes.c`
  - function used for debugging purposes; blinks the LED on the microcontroller a number (XX) times to indicate modes of operation
- `ESCArming_prot.c`
  - protocol used to arm the ESC at the correct values for the purposes of the project
- `FULL-FAST-22-OCT.c`
  - final complete code for the MICROCONTROLLER used on project day!
- `Heading_prot.c`
  - protocol used to calibrate the steering heading
- `PinSetup.c`
  - configures the necessary I/O pins
- `SwingSteering_prot.c`
  - test function used to swing the steering left and right indefinitely
