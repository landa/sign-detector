/*
 * CornerLocator.cpp
 *
 *  Created on: Sep 12, 2012
 *      Author: landa
 */

#include "CornerLocator.h"

using namespace cv;
using namespace std;

CornerLocator::CornerLocator() {
	thresh = 50;
	N = 11;
	wndname = "Square Detection";
}

CornerLocator::~CornerLocator() {
}

double CornerLocator::angle(Point pt1, Point pt2, Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2)
			/ sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

// modified from the squares.cpp OpenCV sample
void CornerLocator::findSquares(const Mat& image,
		vector<vector<Point> >& squares) {
	squares.clear();

	Mat pyr, timg, gray0(image.size(), CV_8U), gray;

	// down-scale and upscale the image to filter out the noise
	pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
	pyrUp(pyr, timg, image.size());
	vector<vector<Point> > contours;

	// find squares in every color plane of the image
	for (int c = 0; c < 3; c++) {
		int ch[] = { c, 0 };
		mixChannels(&timg, 1, &gray0, 1, ch, 1);

		// try several threshold levels
		for (int l = 0; l < N; l++) {
			// hack: use Canny instead of zero threshold level.
			// Canny helps to catch squares with gradient shading
			if (l == 0) {
				// apply Canny. Take the upper threshold from slider
				// and set the lower to 0 (which forces edges merging)
				Canny(gray0, gray, 0, thresh, 5);
				// dilate canny output to remove potential
				// holes between edge segments
				dilate(gray, gray, Mat(), Point(-1, -1));
			} else {
				// apply threshold if l!=0:
				//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
				gray = gray0 >= (l + 1) * 255 / N;
			}

			// find contours and store them all as a list
			findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

			vector<Point> approx;

			// test each contour
			for (size_t i = 0; i < contours.size(); i++) {
				// approximate contour with accuracy proportional
				// to the contour perimeter
				approxPolyDP(Mat(contours[i]), approx,
						arcLength(Mat(contours[i]), true) * 0.02, true);

				// square contours should have 4 vertices after approximation
				// relatively large area (to filter out noisy contours)
				// and be convex.
				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation
				if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000
						&& isContourConvex(Mat(approx))) {
					double maxCosine = 0;

					for (int j = 2; j < 5; j++) {
						// find the maximum cosine of the angle between joint edges
						double cosine = fabs(
								angle(approx[j % 4], approx[j - 2],
										approx[j - 1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					// if cosines of all angles are small
					// (all angles are ~90 degree) then write quandrange
					// vertices to resultant sequence
					if (maxCosine < 0.3)
						squares.push_back(approx);
				}
			}
		}
	}
}

float CornerLocator::distanceTo(Point from, Point to) {
	float from_pos = sqrt(from.x * from.x + from.y * from.y);
	float to_pos = sqrt(to.x * to.x + to.y * to.y);
	return abs(to_pos - from_pos);
}

vector<Point> CornerLocator::findCenterSquare(vector<vector<Point> > squares) {
	Point middle(640 / 2, 480 / 2);
	Point best(0, 0);
	vector<Point> best_square = squares[0];
	float best_distance = distanceTo(middle, best);
	double best_area = contourArea(best_square);
	for (unsigned int ii = 0; ii < squares.size(); ++ii) {
		float x = 0, y = 0;
		// find the center of the square
		for (unsigned int jj = 0; jj < squares[ii].size(); ++jj) {
			x += squares[ii][jj].x;
			y += squares[ii][jj].y;
		}
		x /= squares[ii].size();
		y /= squares[ii].size();
		Point current(x, y);
		float current_distance = distanceTo(middle, current);
		double current_area = contourArea(squares[ii]);
		if (current_distance <= best_distance && current_area < best_area) {
			best_distance = current_distance;
			best_square = squares[ii];
			best = current;
		}
	}
	return best_square;
}

// the function draws all the squares in the image
void CornerLocator::drawSquares(Mat& image, vector<vector<Point> >& squares,
		Scalar color) {
	for (size_t i = 0; i < squares.size(); i++) {
		const Point* p = &squares[i][0];
		int n = (int) squares[i].size();
		polylines(image, &p, &n, 1, true, color, 1, CV_AA);
	}

	imshow(wndname, image);
}

// From http://en.wikipedia.org/wiki/Graham_scan#Pseudocode
double CornerLocator::ccw(Point p1, Point p2, Point p3) {
	return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

void CornerLocator::drawCorners(Mat& image, vector<Point>& square,
		Scalar color) {
	for (size_t i = 0; i < square.size(); i++) {
		const Point* p = &square[i];
		circle(image, *p, 4, color, -1);
	}
	putText(image, "1", square[0], FONT_HERSHEY_COMPLEX_SMALL, 0.8,
			cvScalar(255, 255, 255), 1, CV_AA);
	putText(image, "2", square[1], FONT_HERSHEY_COMPLEX_SMALL, 0.8,
			cvScalar(255, 255, 255), 1, CV_AA);
	putText(image, "3", square[2], FONT_HERSHEY_COMPLEX_SMALL, 0.8,
			cvScalar(255, 255, 255), 1, CV_AA);
	putText(image, "4", square[3], FONT_HERSHEY_COMPLEX_SMALL, 0.8,
			cvScalar(255, 255, 255), 1, CV_AA);
	imshow(wndname, image);
}

void CornerLocator::saveImage(Mat mat_img, int still) {
	stringstream ss;
	ss << still << ".png";
	imwrite(ss.str(), mat_img);
}

bool sortByY(Point a, Point b) {
	return (a.y < b.y);
}
vector<Point> CornerLocator::orderCorners(vector<Point> square) {
	vector<Point> result;
	Point topleft, topright, bottomleft, bottomright;

	// scan-line algorithm -- scan through interesting y points from top to bottom
	vector<Point> pointsAscendingByY;
	for (unsigned int ii = 0; ii < square.size(); ++ii) {
		pointsAscendingByY.push_back(square[ii]);
	}
	sort(pointsAscendingByY.begin(), pointsAscendingByY.end(), sortByY);
	// the first two are the topleft and topright points
	if (pointsAscendingByY[0].x < pointsAscendingByY[1].x) {
		topleft = pointsAscendingByY[0];
		topright = pointsAscendingByY[1];
	} else {
		topleft = pointsAscendingByY[1];
		topright = pointsAscendingByY[0];
	}
	// the next two are the bottomleft and bottomright points
	if (pointsAscendingByY[2].x < pointsAscendingByY[3].x) {
		bottomleft = pointsAscendingByY[2];
		bottomright = pointsAscendingByY[3];
	} else {
		bottomleft = pointsAscendingByY[3];
		bottomright = pointsAscendingByY[2];
	}

	result.push_back(topleft);
	result.push_back(topright);
	result.push_back(bottomright);
	result.push_back(bottomleft);
	return result;
}

vector<Point> CornerLocator::performDetection(char* file_location) {
	IplImage* img = cvLoadImage(file_location);
	Mat mat_img = Mat(img);
	vector<vector<Point> > squares;
	findSquares(mat_img, squares);
	vector<Point> center_square = findCenterSquare(squares);
	vector<vector<Point> > center_squares;
	center_squares.push_back(center_square);
	vector<Point> ordered_center_square = orderCorners(center_square);
	drawSquares(mat_img, squares, Scalar(0, 255, 0));
	drawSquares(mat_img, center_squares, Scalar(255, 0, 0));
	drawCorners(mat_img, ordered_center_square, Scalar(0, 0, 255));
	cvReleaseImage(&img);
	mat_img.release();
	return ordered_center_square;
}
