/*
 * CornerLocator.h
 *
 *  Created on: Sep 12, 2012
 *      Author: landa
 */

#ifndef CORNERLOCATOR_H_
#define CORNERLOCATOR_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

class CornerLocator {
public:
	CornerLocator();
	vector<Point> performDetection(char* file_location);
	virtual ~CornerLocator();
private:
	double angle(Point pt1, Point pt2, Point pt0);
	void findSquares(const Mat& image, vector<vector<Point> >& squares);
	float distanceTo(Point from, Point to);
	vector<Point> findCenterSquare(vector<vector<Point> > squares);
	void drawSquares(Mat& image, vector<vector<Point> >& squares, Scalar color);
	void drawCorners(Mat& image, vector<Point>& square, Scalar color);
	void saveImage(Mat mat_img, int still);
	vector<Point> orderCorners(vector<Point> square);
	double ccw(Point p1, Point p2, Point p3);

	const char* wndname;
	int thresh;
	int N;
};

#endif /* CORNERLOCATOR_H_ */
