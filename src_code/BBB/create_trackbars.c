void create_trackbars(int* colour)
{
	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &colour[0], 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &colour[1], 179);

	cvCreateTrackbar("LowS", "Control", &colour[2], 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &colour[3], 255);

	cvCreateTrackbar("LowV", "Control", &colour[4], 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &colour[5], 255);
	//thresh_callback(0, 0);
}
