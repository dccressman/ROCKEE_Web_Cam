// OpenCV3-Practice.cpp : Defines the entry point for the console application.


#include "stdafx.h"
#include<stdbool.h>
#include <stdio.h>
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv\cv.h"
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv2\opencv.hpp"
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv\highgui.h"
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv2\imgproc\imgproc.hpp"
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv2\features2d.hpp"


using namespace std;
using namespace cv;
char key;


Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
int theObject[2] = { 0,0 };
Mat edges;
int minArea = 1000;
int minDistance = 1000;
void SortKeypoints(vector<KeyPoint> &keypoints, Mat &maskedI, Mat &picture, vector<Point3f> &midpoints, vector<float> slope)
{
	int k = 0;
	bool keep = true;
	
	for (int i = 0; i < keypoints.size(); i++)
	{
		circle(maskedI, keypoints[i].pt, (keypoints[i].size / 2), (255, 255, 255), -1);
	}

	for (int i = 0; i < keypoints.size(); i++)
	{
		for (int j = i + 1;j < keypoints.size(); j++) {
			midpoints[k].x = (keypoints[i].pt.x + keypoints[j].pt.x) / 2;
			midpoints[k].y = (keypoints[i].pt.y + keypoints[j].pt.y) / 2;
			Vec3b color = maskedI.at<Vec3b>(Point(midpoints[k].x, midpoints[k].y));
			keep = true;
			if (color[0] == 0)
			{
				for (int m = midpoints[k].y; m < picture.rows; m++) {
					keep = true;
					Vec3b circlecheck = maskedI.at<Vec3b>(Point(midpoints[k].x, m));
					if ((circlecheck.val[0] + circlecheck.val[1] + circlecheck.val[2]) > 0)
					{
						keep = false;
						m = picture.rows;
					}
				}
				circle(maskedI, Point2f(midpoints[k].x, midpoints[k].y), 2, (0, 0, 255), 3);
				if (keep) {
					circle(picture, Point2f(midpoints[k].x, midpoints[k].y), 2, (0, 0, 255), 3); //Debug line
		 //assigning weight based on distance from that point to the two it is in between.
					float dist1, dist2;
					dist1 = sqrt((midpoints[k].x - keypoints[i].pt.x)*(midpoints[k].x - keypoints[i].pt.x) + (midpoints[k].y - keypoints[i].pt.y) *(midpoints[k].y - keypoints[i].pt.y)) - keypoints[i].size / 2;
					dist2 = sqrt((midpoints[k].x - keypoints[j].pt.x)*(midpoints[k].x - keypoints[j].pt.x) + (midpoints[k].y - keypoints[j].pt.y)*(midpoints[k].y - keypoints[j].pt.y)) - keypoints[j].size / 2;
					midpoints[k].z = dist1 + dist2;// can add a distance 3 for the distance from the robot.
											  //get the color at that point- if white then weight = zero. don't want it
											  // if the line from the point to the robot passses through white, then weight =0. no want, got rid of
											  //alternatively we could sort the keypoints. organize them from one direction to another via x values- then would have to do less sorting
					slope[k] = (keypoints[i].pt.x - keypoints[j].pt.x) / (keypoints[j].pt.y - keypoints[i].pt.y);
					k++;
				}
			}
		}
	}
	//DEBUG
	imshow("limited", maskedI);
	imshow("blacky", picture);

}
void pickFeatures(Mat bwdifferential, Mat &picture)
{
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point3f> midpoints;// change to point3f and get rid of the weight vector
	vector<float> slope;
	//parameters for the detectro to detect keypoints within the bwdifferential
	SimpleBlobDetector::Params params;
	//params.minThreshold = 10;
	//params.maxThreshold = 200;
	params.filterByArea = true;
	params.minArea = 604;
	params.maxArea = 1800;
	//params.minDistBetweenBlobs =  minDistance;
	params.filterByCircularity = false;
	params.filterByColor = false;
	params.filterByConvexity = false;
	//params.minConvexity = 0.8;
	params.filterByInertia = true;
	params.minInertiaRatio = float(0.05);
	//detect the keypoints within bwdifferential
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	vector<KeyPoint> keypoints;
	detector->detect(bwdifferential, keypoints);
	Mat maskedI;
	//create a black image for just the keypoints
	bwdifferential.copyTo(maskedI);
	threshold(maskedI, maskedI, 0, 0, THRESH_BINARY);//black image
	//DEBUG
	cvtColor(maskedI, maskedI, CV_GRAY2BGR);//lets colors be put onto it, will be able to cut out eventually
	midpoints.resize(keypoints.size()*keypoints.size());
	slope.resize(keypoints.size()*keypoints.size());
	SortKeypoints(keypoints, maskedI, picture,midpoints, slope);
}

void motionSearch(Mat bwdifferential, Mat &frame1)
{
	int threshold_value = 20;
	Mat temp;
	bool objectExists = false;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	bwdifferential.copyTo(temp);
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours
	if (contours.size() > 0) {
		objectExists = true;
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size() - 1));
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
		theObject[0] = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		theObject[1] = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
	}
	drawContours(frame1, contours, -1, (0, 255, 0), 3);

	SimpleBlobDetector::Params params;
	params.minThreshold = 10;
	params.maxThreshold = 200;
	params.filterByArea = true;
	params.minArea = 2000;
	params.minDistBetweenBlobs = 1700;
	params.filterByCircularity = false;
	params.filterByColor = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;

	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	vector<KeyPoint> keypoints;
	detector->detect(temp, keypoints);
	Mat im_keypoints;

	drawKeypoints(temp, keypoints, im_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//imshow("blobs", im_keypoints);


	////make some temp x and y variables so we dont have to type out so much
	//int x = theObject[0];
	//int y = theObject[1];
	////draw some crosshairs on the object
	//
	//circle(frame1, Point(x, y), 20, Scalar(0, 255, 0), 2);
	//line(frame1, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	//line(frame1, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	//line(frame1, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	//line(frame1, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	////putText(frame1, "Tracking object at (" + to_string(x) + "," + to_string(y) + ")", Point(x, y), 1, 1, Scalar(255, 0, 0), 2);

}

int main()
{
	bool debugMode = false;
	bool pause = false;
	Mat frame1, frame2;
	Mat gray1, gray2;
	Mat differential, bwdifferential;
	int threshold_value = 20;
	int BLUR_SIZE = 10;
	VideoCapture video;
	int max_thresh = 255;
	int threshold1 = 0, threshold2 = 0, threshold3 = 0, threshold4 = 0, threshold5 = 0, threshold6 = 0;
	Mat medianBlurred, OrangeCrop, OrangeCrop2;
	while (1)
	{
		//video.open("RMC.avi");
		//if (!video.isOpened()) {
		//	cout << "ERROR ACQUIRING VIDEO FEED\n";
		//	getchar();
		//	return -1;
		//}
		//while (video.get(CV_CAP_PROP_POS_FRAMES) < (video.get(CV_CAP_PROP_FRAME_COUNT) - 1))
		//{
		//	if (video.read(frame1) && video.read(frame2))
		//	{
		//		int offset_x = 600;
		//		int offset_y = 129;
		//		Rect roi = Rect(offset_x, offset_y, 400, (frame1.size().height - offset_y * 2));
		//		frame1 = frame1(roi);
		//		frame2 = frame2(roi);
		//		// convert color to grayscale
		//		cvtColor(frame1, gray1, CV_RGB2GRAY);
		//		cvtColor(frame2, gray2, CV_RGB2GRAY);
		//		// look at the difference between images, must be in grayscale
		//		absdiff(gray1, gray2, differential);
		//		// turns this difference to black and white
		//		threshold(differential, bwdifferential, threshold_value, 255, THRESH_BINARY);
		//		// makes the difference more obvious, scales it up
		//		blur(bwdifferential, bwdifferential, (Size(BLUR_SIZE, BLUR_SIZE)));
		//		// blur returns a grayscale, we want black and white
		//		threshold(bwdifferential, bwdifferential, threshold_value, 255, THRESH_BINARY);

		//	}
		//	motionSearch(bwdifferential, frame1);
		//	imshow("Frame1", frame1);
		//	imshow("threshold", bwdifferential);
		//	

		//image filtering the Orange
		/*OrangeCrop = imread("OPT.bmp");
		if (OrangeCrop.data == 0)
		{
			cout << "FAILED TO OPEN IMAGE\n";
			return -1;
		}
		//given picture
		imshow("Orange", OrangeCrop);
		OrangeCrop.copyTo(OrangeCrop2);

		cvtColor(OrangeCrop2, OrangeCrop2, CV_RGB2HSV);
		inRange(OrangeCrop2, Scalar(0, 80, 80), Scalar(40, 100, 100), OrangeCrop2);
		//HSV values thresholded
		imshow("Orange2", OrangeCrop2);
		char TrackbarName3[50], TrackbarName32[50], TrackbarName31[50], TrackbarName34[50], TrackbarName35[50], TrackbarName36[50];
		sprintf(TrackbarName3, "1L %i", 255);
		sprintf(TrackbarName32, "2L %i", 255);
		sprintf(TrackbarName31, "3L %i", 255);
		sprintf(TrackbarName34, "1H %i", 255);
		sprintf(TrackbarName35, "2H %i", 255);
		sprintf(TrackbarName36, "3H %i", 255);
		namedWindow("Trackbars", WINDOW_GUI_EXPANDED);
		createTrackbar(TrackbarName3, "Trackbars", &threshold1, 255);
		createTrackbar(TrackbarName32, "Trackbars", &threshold2, 255);
		createTrackbar(TrackbarName31, "Trackbars", &threshold3, 255);
		createTrackbar(TrackbarName34, "Trackbars", &threshold4, 255);
		createTrackbar(TrackbarName35, "Trackbars", &threshold5, 255);
		createTrackbar(TrackbarName36, "Trackbars", &threshold6, 255);
		inRange(OrangeCrop, Scalar(threshold1, threshold2, threshold3), Scalar(threshold4, threshold5, threshold6), OrangeCrop);
		//thresholding the RGB image
		imshow("Orange3", OrangeCrop);*/

		video.open("dirts.avi");
		if (!video.isOpened()) {
			cout << "ERROR ACQUIRING VIDEO FEED\n";
			getchar();
			return -1;
		}
		if (video.read(frame1))
		{
			// height =240
			// width = 320
			threshold1 = 22;
			threshold2 = 80;

			cvtColor(frame1, gray1, CV_RGB2GRAY);
			Rect roi = Rect(threshold1, threshold2, 298, (frame1.size().height - threshold2));
			gray1 = gray1(roi);

			//threshold(gray1, gray2, 106, 255, THRESH_BINARY);//111
			adaptiveThreshold(gray1, medianBlurred, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 131, 2);
			adaptiveThreshold(gray1, gray2, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 67, 2);
			imshow("Frame1", frame1);
			imshow("Gray", gray1);
			//motionSearch(gray2, gray1);
			imshow("BW", gray2);
			imshow("AdaptiveThreshold", medianBlurred);
			pickFeatures(medianBlurred, gray1);


		}

		switch (waitKey(10)) {
		case 27: //'esc' key has been pressed, exit program.
			return 0;
			//case 116: //'t' has been pressed. this will toggle tracking
			//	trackingEnabled = !trackingEnabled;
			//	if (trackingEnabled == false) cout << "Tracking disabled." << endl;
			//	else cout << "Tracking enabled." << endl;
			//	break;
		case 100: //'d' has been pressed. this will debug mode
			debugMode = !debugMode;
			if (debugMode == false) cout << "Debug mode disabled." << endl;
			else cout << "Debug mode enabled." << endl;
			break;
		case 112: //'p' has been pressed. this will pause/resume the code.
			pause = !pause;
			if (pause == true) {
				cout << "Code paused, press 'p' again to resume" << endl;
				while (pause == true) {
					//stay in this loop until 
					switch (waitKey()) {
						//a switch statement inside a switch statement? Mind blown.
					case 112:
						//change pause back to false
						pause = false;
						cout << "Code resumed." << endl;
						break;
					}
				}
			}

		}


		//	}
		video.release();

	}
	return 0;
}

