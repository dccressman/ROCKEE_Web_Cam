
// OpenCV3-Practice.cpp : Defines the entry point for the console application.


#include "stdafx.h"
#include<stdbool.h>
#include <stdio.h>
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv\cv.h"
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv2\opencv.hpp"
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv\highgui.h"
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv2\imgproc\imgproc.hpp"
#include "C:\Users\Dana\Desktop\OpenCV3\opencv\build\include\opencv2\features2d.hpp"

//#include "Adapt.h"

using namespace std;
using namespace cv;
char key;


Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
int theObject[2] = { 0,0 };
Mat edges;
int minArea = 1000;
int minDistance = 1000;

//calculates distance between two points given
float distance(Point p1, Point p2)
{
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y) *(p1.y - p2.y));
}
//crops the non-arena areas from the image passed to function
void CropNonArena(Mat &cutImage, Mat &gray2, Mat &gray3)
{

	vector< vector<Point> > contours;
	vector<Point> polygon;
	vector<Vec4i> hierarchy;
	int Lc = 0;
	int longest = 0;
	float longdist = 0;
	float dist = 0;
	// finding the largest contour in the image- should be the top part that it is desired to crop out( right now thisi s the roof of the tent, will need to check at competition to see how it reacts there.
	findContours(gray2, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	for (int i = 1;i < contours.size(); i++) {//scrolls through the contour list
		if (contourArea(contours.at(i))>contourArea(contours.at(Lc))) {//determines the contour that has the largest area
			Lc = i;
		}
	}
	drawContours(gray3, contours, Lc, Scalar(255, 255, 0), 1, 8, hierarchy, 0, Point(0, 0));//DEBUG
																							//rectangle makes it too big of a blocked area
																							//Rect topCut =boundingRect(contours[Lc]);
																							//rectangle(image, topCut, Scalar(255, 255, 255), -1);

	approxPolyDP(contours[Lc], polygon, 10, true); // approximate polygon of the largest contour
	for (int i = 0; i < (polygon.size() - 1); i++)// scroling through the sides of the polygon
	{
		dist = distance(polygon[i], polygon[i + 1]);
		if (dist > longdist)//picking the longest distance- should be a good cropping point in the back of the arena- so far has worked for all pictures, may need to update for an actual arena
		{
			longest = i;
			longdist = dist;
		}
		//take the distance between the points, find longest line. this gives the y coordinate for cutoff, then the x coord determined
		//from the rightmost extrema of the thing.  
		// alternatively, if always does the same order, could draw a line from the rightmost point at the bottom to the point 
		// of the longest line, then same from the left and crop everything above these 
		// not sure if able to save this as a new image/ mat if you do this
	}
	arrowedLine(gray3, polygon[1], polygon[2], Scalar(255, 255, 255), 4);//DEBUG
	arrowedLine(gray3, polygon[longest], polygon[longest + 1], Scalar(255, 255, 255), 4);//DEBUG

	Rect cropping = Rect(0, polygon[longest].y, gray3.cols, (gray3.rows - polygon[longest].y));// cropping rectangle- based on the location of the longest polygon length. 
																							   //May have to look at since the polygon goes around in a counterclockwise, the highest y to go with the first half of x values- if the longest polygon length of the largest contour area doesn't hold as a good place
	cutImage = cutImage(cropping);// crops the image to be within the cropping rectangle


	imshow("imshowing", gray3);
	imshow("blockedOut", gray2);
	imshow("cropped", cutImage);

}
//sorts the input of keypoints to pick out the robot path
void SortKeypoints(vector<KeyPoint> &keypoints, Mat &maskedI, Mat &picture, vector<Point3f> &midpoints, vector<float> slope)
{
	int k = 0;
	bool keep = true;
	// draws circles on maskedI for each of the features (keypoints)aka craters and rocks based on their size- the larger they are, the larger the size (keyfeatures)
	// used later to determine if the midpoints fall within the keyfeatures
	for (int i = 0; i < keypoints.size(); i++)
	{
		circle(maskedI, keypoints[i].pt, (keypoints[i].size / 2), (255, 255, 255), -1);
	}
	// picking out the midpoints between each of the keypoints- these will be potential places for the robot to drive through
	// see if there is a way to only look between adjacent keyfeatures... may not be possible depending
	for (int i = 0; i < keypoints.size(); i++)
	{
		for (int j = i + 1;j < keypoints.size(); j++) {
			//calc midpoints
			midpoints[k].x = (keypoints[i].pt.x + keypoints[j].pt.x) / 2;
			midpoints[k].y = (keypoints[i].pt.y + keypoints[j].pt.y) / 2;
			//checks the color of the midpoint
			Vec3b color = maskedI.at<Vec3b>(Point(midpoints[k].x, midpoints[k].y));
			keep = true;
			//if the area is black- not in the middle of another keyfeature, keep going
			if (color[0] == 0)
			{   
				//checking the midpoints to see if they are in the same line (vertical) as a keyfeature- this would make it undriveable by the robot without running into another keyfeature
				for (int m = midpoints[k].y; m < picture.rows; m++) {
					keep = true;
					Vec3b circlecheck = maskedI.at<Vec3b>(Point(midpoints[k].x, m));
					if ((circlecheck.val[0] + circlecheck.val[1] + circlecheck.val[2]) > 0)// this is the color vector, searching if it is black, basically
					{
						keep = false;
						m = picture.rows;
					}
				}
				// draw keypoints if they are not in keyfeature- DEBUGGING PURPOSES
				circle(maskedI, Point2f(midpoints[k].x, midpoints[k].y), 2, (0, 0, 255), 3);
				if (keep) {// if midpoint is "driveable" by robot- "driveable" means not in another keyfeature, not inline with a keyfeature
					circle(picture, Point2f(midpoints[k].x, midpoints[k].y), 2, (0, 0, 255), 3); //DEBUG- visualizing the driveable points
					
					//assigning weight based on distance from that point to the two it is in between. Helping to pick the best points
					float dist1, dist2;
					dist1 = sqrt((midpoints[k].x - keypoints[i].pt.x)*(midpoints[k].x - keypoints[i].pt.x) + (midpoints[k].y - keypoints[i].pt.y) *(midpoints[k].y - keypoints[i].pt.y)) - keypoints[i].size / 2;
					dist2 = sqrt((midpoints[k].x - keypoints[j].pt.x)*(midpoints[k].x - keypoints[j].pt.x) + (midpoints[k].y - keypoints[j].pt.y)*(midpoints[k].y - keypoints[j].pt.y)) - keypoints[j].size / 2;
					midpoints[k].z = dist1 + dist2;// can add a distance 3 for the distance from the robot.
											  //get the color at that point- if white then weight = zero. don't want it
											  // if the line from the point to the robot passses through white, then weight =0. no want, got rid of
											  //alternatively we could sort the keypoints. organize them from one direction to another via x values- then would have to do less sorting
					
        			  // looking at the verticality of the path of the robot- the more vertical the slope is, the better the robot's path of travel.
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
//called after the sortingKeypoints to pick the best path
void preferredOrdering(vector<Point3f> &midpoints, vector<float> &slope, Mat &Again)
{
	for (int k = 0; k < midpoints.size()-2; k++)
	{
// basic sort, maybe wants to be fancier, but speed is less of an issue than storage is
		if (slope[k] < slope[k + 1])
		{
			float temp;
			Point3f tempPoint;
			tempPoint = midpoints[k];
			midpoints[k] = midpoints[k + 1];
			midpoints[k + 1] = tempPoint;
			temp = slope[k];
			slope[k] = slope[k + 1];
			slope[k + 1] = temp;
		}
	}
	// plot the circle at which your main midpoint is
	circle(Again, Point2f(midpoints[0].x, midpoints[0].y), 2, (0, 0, 255), 3);
	imshow("Preferred Spot", Again);
	//need to take the slopes and compare to find the biggest slope and then put that point as the preferred point. maybe incorporate
	//this into the z point for the weight... maybe trade that out for the slope for the weight, though it is kind of nice to have the 
	// width of what space the robot has to traavel through. some kinds of percentages comaprison
}
// Handles the filtering to form the keypoints within the picture and make them into keyfeatures. Calls functions for 
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
	Mat maskedI,Again;
	//create a black image for just the keypoints
	bwdifferential.copyTo(maskedI);
	picture.copyTo(Again);
	threshold(maskedI, maskedI, 0, 0, THRESH_BINARY);//black image
	cvtColor(maskedI, maskedI, CV_GRAY2BGR);//lets colors be put onto it, will be able to cut out eventually, DEBUG

	midpoints.resize(keypoints.size()*keypoints.size()); // maximum amount of midpoints would the keypoints^2- one for each combination of two
	slope.resize(keypoints.size()*keypoints.size()); // same as midpoints
	SortKeypoints(keypoints, maskedI, picture,midpoints, slope);// SortKeypoints-sorts the input of keypoints to pick out the robot path in the middle of them
	preferredOrdering(midpoints,slope,Again);// takes a look at the sorted "acceptable" midpoints for robot path and selects the best one possible
	
	
	imshow("blacky2", picture);//DEBUG?
}
//looks at the mat of the frame differences to mark which one is the robot that is moving
void motionSearch(Mat bwdifferential, Mat &frame1)
{

	
	Mat temp;
	bool objectExists = false;
	// required for finding the contours which is doen by findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	bwdifferential.copyTo(temp);// copied so that you don't have to worry about messing up the original
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours
	if (contours.size() > 0) { // if contours are found (there are differences shown in bwdifferential)
		objectExists = true; // there is an object that has moved
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size() - 1));// sorting the contours so that you have the largest one
		objectBoundingRectangle = boundingRect(largestContourVec.at(0)); // bounding the largest contour with a rectangle (box around the robot)
		theObject[0] = objectBoundingRectangle.x + objectBoundingRectangle.width / 2; // setting the locations of the object middle X coord
		theObject[1] = objectBoundingRectangle.y + objectBoundingRectangle.height / 2; // setting the locations of the object middle Y coord
	}
	drawContours(frame1, contours, -1, (0, 255, 0), 3);// draw the object contours on the frame
	//Detect the main blob - largest contour based on area and the threshold difference- area is area of blobs, thresholding is the grouping of the pixels based on closeness of pixels of same white and black
	// parameters for the detector
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

	//actual blob detector creation and detection of blobs
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	vector<KeyPoint> keypoints;
	detector->detect(temp, keypoints);
	Mat im_keypoints;
	// draw them onto the black and white
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
	threshold1 = 220000;
	threshold2 = 600000;
	Mat medianBlurred, OrangeCrop, OrangeCrop2;
	while (1)
	{
		vector<Vec4i> hierarchy;
		vector< vector<Point> > contours;
		//MOTION SEARCH START
		// opening a video  and making sure it opened
		//video.open("RMC.avi");
		//if (!video.isOpened()) {
		//	cout << "ERROR ACQUIRING VIDEO FEED\n";
		//	getchar();
		//	return -1;
		//}

		//reading through the video frames and taking them 2 at a time
		//while (video.get(CV_CAP_PROP_POS_FRAMES) < (video.get(CV_CAP_PROP_FRAME_COUNT) - 1))
		//{
		//	// making sure could read both frames
		//	if (video.read(frame1) && video.read(frame2))
		//	{
		//	//cropping the image to cut out the edges of the frame which had parts outside of the robot arena
		//		int offset_x = 600;
		//		int offset_y = 129;
		//		Rect roi = Rect(offset_x, offset_y, 400, (frame1.size().height - offset_y * 2));
		//		frame1 = frame1(roi);
		//		frame2 = frame2(roi);
		//		// convert color to grayscale
		//		cvtColor(frame1, gray1, CV_RGB2GRAY);
		//		cvtColor(frame2, gray2, CV_RGB2GRAY);
		//		// look at the difference between images, must be in grayscale- this would be the motion
		//		absdiff(gray1, gray2, differential);
		//		// turns this difference to black and white
		//		threshold(differential, bwdifferential, threshold_value, 255, THRESH_BINARY);
		//		// makes the difference more obvious, scales it up
		//		blur(bwdifferential, bwdifferential, (Size(BLUR_SIZE, BLUR_SIZE)));
		//		// blur returns a grayscale, we want black and white
		//		threshold(bwdifferential, bwdifferential, threshold_value, 255, THRESH_BINARY);

		//	}
		//	motionSearch(bwdifferential, frame1);
		// // displays the images for debugging purposes
		//	imshow("Frame1", frame1);
		//	imshow("threshold", bwdifferential);
		//	
		//MOTION SEARCH END

		//FILTERING ORANGE START
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

		//FILTERING ORANGE END

	/*	video.open("dirts.avi");
		if (!video.isOpened()) {
			cout << "ERROR ACQUIRING VIDEO FEED\n";
			getchar();
			return -1;
		}
		if (video.read(frame1))*/

		frame1 = imread("46.jpg");
		if (1)
		{
			// height =240
			// width = 320

			Mat hsv1,gray3,cutImage;
			cvtColor(frame1, gray3, CV_RGB2GRAY);
			cvtColor(frame1, hsv1, CV_RGB2HSV);
			imshow("hsv", hsv1);
			gray3.copyTo(cutImage);
			//Rect roi = Rect(threshold1, threshold2, 298, (frame1.size().height - threshold2));
			//gray1 = gray1(roi);
			//threshold(gray1, gray1, 40, 255, THRESH_BINARY);//111
			adaptiveThreshold(gray3, medianBlurred, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 131, 2);
			adaptiveThreshold(gray3, gray2, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 67, 2);
			Rect roi = Rect(threshold1, threshold2, 298, (frame1.size().height - threshold2));
			//gray1 = gray1(roi);
			cvtColor(gray2, gray2, CV_GRAY2BGR);
			cvtColor(gray2, gray2, CV_BGR2GRAY);
			//findContours(gray3, gray3,hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
			//pickFeatures(medianBlurred, gray3);
			//imshow("Frame1", frame1);
			//imshow("Gray", gray1);
			//motionSearch(gray2, gray1);
			//imshow("BW", gray2);
			//imshow("AdaptiveThreshold", medianBlurred);
			//pickFeatures(medianBlurred, gray3);
			
			//CROPPING OUT NON_ARENA
			CropNonArena(cutImage, gray2, gray3);// may not need to pass all of these by reference in the future... but also may

			
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

