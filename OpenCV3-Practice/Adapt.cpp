#include "stdafx.h"
#include "Adapt.h"

void resizeRaw(Mat image)
{
//	Vector< vector<Point> > contours;
//	vector<Vec4i> hierarchy;

	int threshold1 = 0, threshold2 = 0, threshold3 = 0, threshold4 = 0, threshold5 = 0, threshold6 = 0;
	char TrackbarName3[50], TrackbarName32[50], TrackbarName31[50], TrackbarName34[50], TrackbarName35[50], TrackbarName36[50];

		sprintf(TrackbarName3, "1L %i", 1000);
		sprintf(TrackbarName32, "2L %i", 1000);
		sprintf(TrackbarName31, "3L %i", 255);
		sprintf(TrackbarName34, "1H %i", 255);
		sprintf(TrackbarName35, "2H %i", 255);
		sprintf(TrackbarName36, "3H %i", 255);
		namedWindow("Trackbars", WINDOW_GUI_EXPANDED);
		createTrackbar(TrackbarName3, "Trackbars", &threshold1, 1000);
		createTrackbar(TrackbarName32, "Trackbars", &threshold2, 1000);
		createTrackbar(TrackbarName31, "Trackbars", &threshold3, 255);
		createTrackbar(TrackbarName34, "Trackbars", &threshold4, 255);
		createTrackbar(TrackbarName35, "Trackbars", &threshold5, 255);
		createTrackbar(TrackbarName36, "Trackbars", &threshold6, 255);


		//parameters for the detectro to detect keypoints within the bwdifferential
		SimpleBlobDetector::Params params;
		//params.minThreshold = 10;
		//params.maxThreshold = 200;
		params.filterByArea = true;
		params.minArea = threshold1;
		params.maxArea = threshold2;
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
		detector->detect(image, keypoints);
		drawKeypoints(image, keypoints, image, (255, 0, 0));

		imshow("imshow", image);
	
	return;
}