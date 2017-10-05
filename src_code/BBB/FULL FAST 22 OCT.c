#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <linux/i2c-dev.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

#define FRAMEWIDTH 256
#define FRAMEHEIGHT 192
#define MINCONTOURSIZE 30
#define MINTHRESHOLDVAL 100
#define MAXTHRESHOLDVAL 1000
//#define MAGNONORTH 15 //+- 5
//#define MAGNOSOUTH 175 //+- 5

//// ________GLOBAL_DECLARATIONS_________________________////////
static const char* LaunchPort = "/dev/ttyO1";
static const char* BluePort = "/dev/ttyO2";
static const char* ResPort = "/home/ubuntu/FINAL/Results.txt";

int Lout, btPort, lpPort, resPort = 0;
int addr = 0x1e;
unsigned char start_addr = 0x03;
unsigned char mode_change[2] = { 0x02, 0x00 };
int  w = 0;
int w2 = 0;
int coneclose = 0;
int cone;
int sendvalue_old = 0;
int dir_old = 1;
int r = 0;
int btflag = 0;
int file;
char filename[40];

// MAGNETOMETER VARIABLES
//int a, b, c, x, y, z = 0

char buf[10];
int sendvalue;
int dir = 0;
int dir1 = 0;
int dirgate = 0;
int i = 0;
int j = 0;
char dataIn = 0;
unsigned char conesteer[8], camsteer[8], carsteer[8];
int lap = 0;

//****************************************************//
//LAUNCHPAD INSTRUCTIONS
// Cam servo left:0x4
//  -Magnitude: left 9 middle 0
// Steering: 
//  -stop:0x60
//  -Left: 0x8
//  -Right:0x7
//  -Magnitude: lsb 1,2,3,4,5,6,7,8,9
//____________FUNCTIONS PROTOTYPES______________________///
void UARTSetup(void)
void create_trackbars(int* colour);
void bubblesort(int arraysize, double * unsorted_array, int * index_array);
double magnetometer(void);
void choosecolour(int return_colour[6], int threshold_colour[6]);


// _______________Main Code _______________________//
int main(void)
{
    UARTSetup();    // Sets up the Bluetooth module and Launchpad microcontroller UART pins
    
	// --------------I2C -----------------------------//
/*
	sprintf(filename, "/dev/i2c-2"); // using i2c-1 bus
	if ((file = open(filename, O_RDWR)) < 0) {
		printf("Failed to open the bus."); exit(1);
	}
	else printf("%s is now open successfully\n", filename);

	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		printf("Failed to acquire bus access and/or talk to slave.\n"); exit(1);
	}
	else printf("I2C bus access to HMC5883 is acquired\n");

	if (write(file, mode_change, 2) != 2) {
		printf("Failed to mode change.\n"); exit(1);
	}
	else printf("Continuous measurement mode is set.\n");
	usleep(100000);

	if (write(file, &start_addr, 1) != 1) {
		printf("Failed to change address pointer to 0x03.\n"); exit(1);
	}
*/
	// ------------- Results File -----------------------------//

	/*
	if ((resPort = open(ResPort, O_RDWR)) < 0) {
	printf("Failed to open the Results port.\n"); exit(1);
	}
	else printf("%s is now open successfully\n", ResPort);
	*/



	//____________INITIALISE CAMERA_______________//
	VideoCapture cap(0);
	if (!cap.isOpened())
    {
		cout << "Webcam setup FAIL" << endl;
		return -1;
	}
	else
		cout << "Webcam setup OK" << endl;

	//_________________________VARIABLES FOR IMAGE PROCESSING_____________________________//
	Mat frame;
	Mat edges;
	Mat edgesThresholded;
	Mat edgesContour;
	Mat element = getStructuringElement(MORPH_RECT, Size(1, 1), Point(0, 0));	// element structure to be used for
                                                                                // opening and closing operations
	Size frameDim;
	bool flag = true;
    
	// Variables to store location of centre of object
	double x0, y0;
	double green_x01, green_y01;
	double green_x02, green_y02;
	double CenterX, CenterY;
	double diffx, diffy;
	double green_diffx1, green_diffy1;
	double green_diffx2, green_diffy2;
    
	// Colour threshold values: different values for colours for different times of day
	int orange[6] = { 116, 131, 140, 211, 154, 255 };
	//int orange[6] = { 110, 126, 152, 192, 162, 255 };
	//int green[6] = { 39, 60, 161, 225, 141, 255 };
	int green[6] = { 21, 82, 126, 255, 119, 255 };
	int colour[6] = { 0 };
    
	// Variables for finding and drawing contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Moments mymoments;
	Moments * orangemoments;
	Mat drawing;
	Scalar color = Scalar(0, 0, 255);
	int contour_number;
	double * area_array;
	int * index_array;
	double gate = 0;

    // Sets the cap on the window frame width and height
	cap.set(CV_CAP_PROP_FRAME_WIDTH, FRAMEWIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAMEHEIGHT);

	//namedWindow("edges", 1);
	//namedWindow("Original", CV_WINDOW_AUTOSIZE);
	//namedWindow("Thresholded with Moments", CV_WINDOW_AUTOSIZE);
	//namedWindow("Contoured Image", CV_WINDOW_AUTOSIZE);
	//moveWindow("Control", 700, 80);	// Window for control sliders////


	//****************************************************************//
	//________________START FINAL CHALLENGE ROUTINE___________________//
	//****************************************************************//

	//Hold program until start received from user input via Bluetooth
	printf("Waiting for Bluetooth.\n");
	while (btflag == 0)
    {
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        r = read(btPort, &dataIn, 4);
        if (dataIn != 0)
            btflag = 1;
	}
	printf("You entered: %c\n", dataIn);

	char readvalue = dataIn;
	//Flush the buffer one more time.
	tcflush(lpPort, TCIOFLUSH);
	tcflush(btPort, TCIOFLUSH);

	while (lap < 6)     //loops for 6 half-laps
    {
		int stage1 = 1; //moving to orange code
		int stage2 = 0; //moving 180 around orange cone
		int stage3 = 0; //moving to middle of green cones
		int currentAngle = 0;
		int steeringAngle = 0;
		int diffmag = 0;
		int offset = 0;

		//============================
		//==		STAGE 1			==
		//============================
		while (stage1)
		{
			int centredtarget = 0;
			choosecolour(colour, orange);
			printf("ENTERED STAGE 1\n");
            
			do      // Loops while the orange cone is still far away from the car
                    // i.e. the moments of the cone is less than the threashold value
			{
				//OPENCV CODE for ORANGE //Feature Tracking
				//-----------------------------------------
				cap.read(frame);	// get a new frame from camera
				//imshow("Original", frame);
                //waitKey(30)

				// Process image: Color conversion, Thresholding, Opening and Closing
				cvtColor(frame, edges, CV_RGB2HSV);
				inRange(edges, Scalar(orange[0], orange[2], orange[4]), Scalar(orange[1], orange[3], orange[5]), edgesThresholded); //Threshold the image
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_OPEN, element);	// Perform morphology operations Opening and Closing
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_CLOSE, element);	// to remove noise from thresholded image
                // Repeat n times for desired noise removal level, at the expense of processing speed
				/*morphologyEx(edgesThresholded, edgesThresholded, MORPH_OPEN, element);
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_CLOSE, element);
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_OPEN, element);
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_CLOSE, element);
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_OPEN, element);
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_CLOSE, element);
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_OPEN, element);
				morphologyEx(edgesThresholded, edgesThresholded, MORPH_CLOSE, element);*/


				// Calculate moments of the image
				mymoments = moments(edgesThresholded, 1);
				if (!mymoments.m00) {
					x0 = frameDim.width / 2;
					y0 = frameDim.height / 2;
				}
				else{
					x0 = mymoments.m10 / mymoments.m00;
					y0 = mymoments.m01 / mymoments.m00;
				}

				cout << "CURRENT MOMENTS: " << mymoments.m00 << endl;
				if (mymoments.m00 > MINTHRESHOLDVAL)
					centredtarget = 1;

				// Finds centre of blob and camera
				frameDim = frame.size();	// get frame dimensions
				CenterX = frameDim.width / 2;
				CenterY = frameDim.height / 2;

				//find difference in centre position
				if (!centredtarget)
					diffx = x0 - CenterX;
				else if (mymoments.m00 < 5)
					diffx = 0;
				else
					diffx = x0 - (CenterX - 15);
				diffy = y0 - CenterY;

				// Draw crosshairs of blob centre and camera frame centre
				cv::line(edgesThresholded, cvPoint(x0 - 10, y0), cvPoint(x0 + 10, y0), CV_RGB(255, 0, 0), 1, 8, 0);
				cv::line(edgesThresholded, cvPoint(x0, y0 - 10), cvPoint(x0, y0 + 10), CV_RGB(255, 0, 0), 1, 8, 0);
				cv::line(edgesThresholded, cvPoint(CenterX - 10, CenterY), cvPoint(CenterX + 10, CenterY), CV_RGB(5, 175, 175), 1, 8, 0);
				cv::line(edgesThresholded, cvPoint(CenterX, CenterY - 10), cvPoint(CenterX, CenterY + 10), CV_RGB(5, 175, 175), 1, 8, 0);

				//imshow("Thresholded with Moments", edgesThresholded);
                //waitKey(30)

				//----DIRECTIONS-----Keep to  orange cone
				if (diffx < -5){ //left
					//printf("LEFT\n\n");
					if (diffx > -25) { dir = 0x81; } //small change needed 
					else if (diffx > -45){ dir = 0x82; }
					else if (diffx > -65){ dir = 0x83; }
					else if (diffx > -85){ dir = 0x84; }
					else if (diffx > -105){ dir = 0x85; }
					else if (diffx > -125){ dir = 0x86; }
					else if (diffx > -145){ dir = 0x87; }
					else if (diffx > -165){ dir = 0x88; }
					else if (diffx > -180){ dir = 0x89; } //large change needed
					else { dir = 0x80; }
				}
				else if (diffx > 5){ //right
					//	printf("RIGHT\n\n");
					if (diffx < 25) { dir = 0x71; } //small change needed
					else if (diffx < 45){ dir = 0x72; }
					else if (diffx < 65){ dir = 0x73; }
					else if (diffx < 85){ dir = 0x74; }
					else if (diffx < 105){ dir = 0x75; }
					else if (diffx < 125){ dir = 0x76; }
					else if (diffx < 145){ dir = 0x77; }
					else if (diffx < 165){ dir = 0x78; }
					else if (diffx < 180){ dir = 0x79; } //large change needed
					else { dir = 0x70; }
				}
				else { dir = 0x70; }

				//----Send to launchPad----
                // Only writes on the first iteration, or when there is a change in state
                // of the dir variable from the previous loop
				*conesteer = { dir };
				if (j == 0 || dir_old != dir){
					write(lpPort, &conesteer, 1);
					printf("dir: %d\n", dir);
				}
				if (j == 255)
                    j = 1;
				else
                    j++;
				dir_old = dir;

				tcflush(lpPort, TCIOFLUSH);
				tcflush(btPort, TCIOFLUSH);
			} while (mymoments.m00 < MAXTHRESHOLDVAL);

			*conesteer = { 0x29 }; //heading obtained so stop
			write(lpPort, &conesteer, 1);
			//cout << "TURNED RIGHT" << endl;
			usleep(300000);
			mymoments.m00 = 0;

			stage1 = 0;
			stage2 = 1;

		}

		//============================
		//==		STAGE 2			==
		//============================
		while (stage2)
		{
			cout << "ENTERED STAGE 2" << endl;
			int angle200 = 0;
			int anglestage2 = 0;
			int upperrange = 0;
			int lowerrange = 0;
			int offset2 = 0;

            //// This part of the code was commented out as the magnetometer could not function
            //// optimally due to environmental circumstances that disrupted readings
            ////_______________________________________________________________________________\\\\
			//====PART 1====// LAUNCHPAD: Camera Servo Rotate Middle
			/*	sendvalue = 0x40; //servo is 4
				unsigned char camsteer[8] = { sendvalue };
				write(lpPort, &camsteer, 1);*/

			//tcflush(lpPort, TCIOFLUSH);
			//tcflush(btPort, TCIOFLUSH);

			//****PART2****// MAGNO: Current? Determine angle180.
			/*anglestage2 = magnetometer();
			offset2 = -60;
			diffmag = (anglestage2 + 180 + offset2) % 360;
			int upper = (diffmag + 5) % 360;
			int lower = (diffmag - 5) % 360;
			cout << "ANGLE STAGE2: " << anglestage2 << "; " << "diffmag: " << diffmag << endl;
			usleep(500000);
			//offset2 = (360 - (anglestage2 - 200)) % 360;

			//angle200 = (anglestage2 + 200) % 360;
			//upperrange=(angle200+10)%360;
			//lowerrange=(angle200-10)%360;

			//=====PART3=====////MAGNO: is direction current +180? LAUNCHPAD: full lock left, if Magno yes STOP
			sendvalue = 0x86; //full lock
			*carsteer = { sendvalue };
			write(lpPort, &carsteer, 1);
			cout << "TURNED LEFT" << endl;

			//tcflush(lpPort, TCIOFLUSH);
			//tcflush(btPort, TCIOFLUSH);

			do
			{
			anglestage2 = magnetometer();
			//diffmag = (anglestage2 + offset2) % 360;
			//printf("Diffmag: %d \nCurrent angle: %d\n", diffmag, anglestage2);
			printf("Desired angle: %d to %d \nCurrent angle: %d\n", lower, upper, anglestage2);
			printf("=========================================\n");
			} while (!((anglestage2 > lower) && (anglestage2 < upper)));*/
            ////_______________________________________________________________________________\\\\
			sendvalue = 0x56;
			*carsteer = { sendvalue };
			write(lpPort, &carsteer, 1);
			usleep(2800000);
			sendvalue = 0x60; //neutral, car stop
			*carsteer = { sendvalue };
			write(lpPort, &carsteer, 1);

			tcflush(lpPort, TCIOFLUSH);
			tcflush(btPort, TCIOFLUSH);

			stage2 = 0;
			stage3 = 1;
		}

		//============================
		//==		STAGE 3			==
		//============================
		while (stage3 == 1)
		{
			//create_trackbars(green);
			cout << "ENTERED STAGE 3" << endl;
			int anglestage3 = 0;
			int magdir3 = 0;
			int diffmag3 = 0;
			int offset3 = 0;
			int dirgate = 0;
			int diffgate = 0;
			int flag = 0;   // 0: Finished stage 2 but see no cones
                            // 1: Finished stage 2 see cones
                            // 2: Past gate, see no cones

			choosecolour(colour, green);
			//*****PART1****// //CAMERA: Can we see two green cones? LAUNCHPAD: Slow left until see two GREEN cones seen, STOP
			do
			{
				// CONTOURING IMAGE PROC
				cap.read(frame);

				// Process image: Color conversion, Thresholding, Opening and Closing
				cvtColor(frame, edges, CV_RGB2HSV);
				inRange(edges, Scalar(colour[0], colour[2], colour[4]), Scalar(colour[1], colour[3], colour[5]), edgesThresholded); //Threshold the image

				if (!flag)  // flag is toggled when the car detects one green cone from a distance of a minimum size
				{
					morphologyEx(edgesThresholded, edgesThresholded, MORPH_OPEN, element);	// Perform morphology operations Opening and Closing
					morphologyEx(edgesThresholded, edgesThresholded, MORPH_CLOSE, element);	// to remove noise from thresholded image
					//imshow("Original", edgesThresholded);
					//waitKey(30);
					mymoments = moments(edgesThresholded, 1);
					if (mymoments.m00 < MINCONTOURSIZE)
						flag = 0;
					else
                        flag = 1;
				}
				else
				{
					// Find contours
					frameDim = frame.size();	// get frame dimensions
					CenterX = frameDim.width / 2;
					CenterY = frameDim.height / 2;

					edgesContour = edgesThresholded;
					findContours(edgesContour, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

					/// Draw contours if there are any
					drawing = Mat::zeros(edgesContour.size(), CV_8UC3);
					area_array = (double *)malloc(contours.size() * 8);
					index_array = (int *)malloc(contours.size() * 4);

					if (contours.size() == 0)
					{
						green_x01 = 0;
						green_y01 = 0;
						if (flag == 1)   // If the green cone is lost, toggle flag back to 0 to scan for a green cone again
							flag = 0;
						else if (flag == 2) // If two green cones are lost, that means the car has past the gate
							flag = 4;       // and flag can be set to 4 to complete the half-lap
						else
							flag = 0;
						printf("NO CONTOURS FOUND\n");
					}

					else
					{
						for (int i = 0; i < contours.size(); i++)
						{
							area_array[i] = contourArea(contours[i], 0);
							//printf("Current value: %f\n", area_array[i]);
							index_array[i] = i;
						}
						// Sort contoues in ascending order
						bubblesort(contours.size(), area_array, index_array);

						// Calculate moments of first cone
						if (area_array[contours.size() - 1] > MINCONTOURSIZE)
						{
							Moments greenmoments1 = moments(contours[index_array[contours.size() - 1]], 1);
							// Finds centre of blob and camera
							green_x01 = greenmoments1.m10 / greenmoments1.m00;
							green_y01 = greenmoments1.m01 / greenmoments1.m00;
							// Find difference in centre position
							green_diffx1 = green_x01 - CenterX;
							green_diffy1 = green_y01 - CenterY;
							if (flag != 2)
								flag = 1;

							// Draw crosshair to indicate position of cone
							//cv::line(drawing, cvPoint(green_x01 - 10, green_y01), cvPoint(green_x01 + 10, green_y01), CV_RGB(255, 0, 0), 1, 8, 0);
							//cv::line(drawing, cvPoint(green_x01, green_y01 - 10), cvPoint(green_x01, green_y01 + 10), CV_RGB(255, 0, 0), 1, 8, 0);
							//drawContours(drawing, contours, index_array[contours.size() - 1], color, 1, 8, hierarchy, 0, Point());
						}
						//else cout << "CONE 1 TOO FAR AWAY" << endl;

						// Repeat for second cone if it exists
						if (area_array[contours.size() - 2] > MINCONTOURSIZE)
						{
							Moments greenmoments2 = moments(contours[index_array[contours.size() - 2]], 1);
							green_x02 = greenmoments2.m10 / greenmoments2.m00;
							green_y02 = greenmoments2.m01 / greenmoments2.m00;
							green_diffx2 = green_x02 - CenterX;
							green_diffy2 = green_y02 - CenterY;
							//cv::line(drawing, cvPoint(green_x02 - 10, green_y02), cvPoint(green_x02 + 10, green_y02), CV_RGB(255, 0, 0), 1, 8, 0);
							//cv::line(drawing, cvPoint(green_x02, green_y02 - 10), cvPoint(green_x02, green_y02 + 10), CV_RGB(255, 0, 0), 1, 8, 0);
							//drawContours(drawing, contours, index_array[contours.size() - 2], color, 1, 8, hierarchy, 0, Point());
							if (flag == 1)
								flag = 2;

							double gap[2] = { green_x01, green_x02 };
							if (gap[0] > gap[1])
								gate = (gap[0] - gap[1]) / 2 + gap[1];
							else
								gate = (gap[1] - gap[0]) / 2 + gap[0];
							cv::line(drawing, cvPoint(gate, 0), cvPoint(gate, FRAMEHEIGHT), CV_RGB(0, 255, 0), 1, 8, 0);
							drawContours(drawing, contours, index_array[contours.size() - 2], color, 1, 8, hierarchy, 0, Point());
							//printf("__________________________________________________________________\n");
							//printf("1st Cone X0: %.1f\t2nd Cone X0: %.1f\n", gap[0], gap[1]);
							//printf("__________________________________________________________________\n");

						}
						//else cout << "CONE 2 TOO FAR AWAY" << endl;
					}
					free(area_array);
					free(index_array);
					if ( (flag == 2) && ((area_array[contours.size() - 2] + area_array[contours.size() - 1]) > 3000) )
                    // When two cones are visible and the combined area of the cones exceed a threshold, break out of loop
                    // to complete half-lap
					{
						flag = 4;
						break;
					}
					//rectangle(drawing, bounding_rect, Scalar(0, 255, 0), 2, 8, 0);

					// Draw crosshair
					//cv::line(drawing, cvPoint(CenterX - 10, CenterY), cvPoint(CenterX + 10, CenterY), CV_RGB(0, 0, 255), 1, 8, 0);
					//cv::line(drawing, cvPoint(CenterX, CenterY - 10), cvPoint(CenterX, CenterY + 10), CV_RGB(0, 0, 255), 1, 8, 0);
					//namedWindow("Contoured Image", CV_WINDOW_AUTOSIZE);
					//imshow("Contoured Image", drawing);
					//waitKey(30);
				}

				switch (flag)
				{
				case 0:     // Slowly turn the car to the left to detect one green cone
					sendvalue = 0x51; //neutral, car stop
					*carsteer = { sendvalue };
					write(lpPort, &carsteer, 1);
					break;
				case 1:     // One cone detected, steer towards cone; generally, steering towards one cone
                            // brings the second cone into view
					cout << "I SEE ONE CONE NOW" << endl;
					diffgate = green_diffx1;
					if (diffgate < -5) {    //left
						if (diffgate > -27) { dirgate = 0x31; } //small change needed 
						else if (diffgate > -44){ dirgate = 0x32; }
						else if (diffgate > -61){ dirgate = 0x33; }
						else if (diffgate > -78){ dirgate = 0x34; }
						else if (diffgate > -95){ dirgate = 0x35; }
						else if (diffgate > -103){ dirgate = 0x36; }
						else if (diffgate > -120){ dirgate = 0x37; }
						else if (diffgate > -137){ dirgate = 0x38; }
						else if (diffgate > -154){ dirgate = 0x39; } //large change needed
						else { dirgate = 0x30; }
					}
					else if (diffgate > 5) { //right
						if (diffgate < 27) { dirgate = 0x21; } //small change needed
						else if (diffgate < 44){ dirgate = 0x22; }
						else if (diffgate < 61){ dirgate = 0x23; }
						else if (diffgate < 78){ dirgate = 0x24; }
						else if (diffgate < 95){ dirgate = 0x25; }
						else if (diffgate < 103){ dirgate = 0x26; }
						else if (diffgate < 120){ dirgate = 0x27; }
						else if (diffgate < 137){ dirgate = 0x28; }
						else if (diffx < 154){ dirgate = 0x29; } //large change needed
						else { dirgate = 0x20; }
					}
					else {      //straight            
						dirgate = 0x20;
						/*flag = 3;
						anglestage3 = magnetometer();
						offset3 = 360 - (anglestage3);*/
					}
					*carsteer = { dirgate };
					write(lpPort, &carsteer, 1);
					break;

				case 2:     // Two cones detected, steer towards centre of two green cones
					cout << "I NOW SEE TWO CONES" << endl;
					diffgate = gate - CenterX;
					if (diffgate < -5) {    //left
						if (diffgate > -27) { dirgate = 0x31; } //small change needed 
						else if (diffgate > -44){ dirgate = 0x32; }
						else if (diffgate > -61){ dirgate = 0x33; }
						else if (diffgate > -78){ dirgate = 0x34; }
						else if (diffgate > -95){ dirgate = 0x35; }
						else if (diffgate > -103){ dirgate = 0x36; }
						else if (diffgate > -120){ dirgate = 0x37; }
						else if (diffgate > -137){ dirgate = 0x38; }
						else if (diffgate > -154){ dirgate = 0x39; } //large change needed
						else { dirgate = 0x30; }
					}
					else if (diffgate > 5) {    //right
						if (diffgate < 27) { dirgate = 0x21; } //small change needed
						else if (diffgate < 44){ dirgate = 0x22; }
						else if (diffgate < 61){ dirgate = 0x23; }
						else if (diffgate < 78){ dirgate = 0x24; }
						else if (diffgate < 95){ dirgate = 0x25; }
						else if (diffgate < 103){ dirgate = 0x26; }
						else if (diffgate < 120){ dirgate = 0x27; }
						else if (diffgate < 137){ dirgate = 0x28; }
						else if (diffx < 154){ dirgate = 0x29; } //large change needed
						else { dirgate = 0x20; }
					}
					else {  //straight
						dirgate = 0x20;
						/*flag = 3;
						anglestage3 = magnetometer();
						offset3 = 360 - (anglestage3);*/
					}
					*carsteer = { dirgate };
					write(lpPort, &carsteer, 1);
					break;

				case 4:
					break;

				default:
					flag = 0;
					break;
				}

			} while (flag != 4);

			*carsteer = { 0x60 };
			write(lpPort, &carsteer, 1);
            
            // Past green cones, completed half-lap! Loop back to stage 1
            lap++;
			stage3 = 0;
			stage1 = 1;
		}
	}
}