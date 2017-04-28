/************************************************************************/
/* STD                                                                  */
/************************************************************************/
#include "Common.hpp"

/************************************************************************/
/* OPENCV                                                               */
/************************************************************************/
#include <opencv2/opencv.hpp>
using cv::Mat;
using cv::Size;
using cv::Point;
using cv::Point2f;
using cv::Point3f;
using cv::Scalar;
using cv::Vec3b;
using cv::Rect;
using cv::RotatedRect;
using cv::Exception;

using cv::waitKey;
using cv::imshow;
using cv::imread;
using cv::imwrite;
using cv::line;
using cv::putText;
using cv::rectangle;
using cv::convexHull;

/************************************************************************/
/* TYPEDEF                                                              */
/************************************************************************/
typedef vector<Point> PointSet;
typedef vector< vector<Point> > SegmentSet;


//Mat construcTransformation(Mat R, Point3f p)// R: 7*1
//{ 
//	Mat XYZ = Mat(3, 1, CV_64FC1);
//	XYZ.at<double>(0, 0) = p.x;
//	XYZ.at<double>(0, 1) = p.y;
//	XYZ.at<double>(0, 2) = p.z;
//	Mat translation = Mat(3, 1, CV_64FC1);
//	translation.at<double>(0, 0) = R.at<double>(0, 0);
//	translation.at<double>(1, 0) = R.at<double>(1, 0);
//	translation.at<double>(2, 0) = R.at<double>(2, 0);
//	double K = R.at<double>(3, 0);
//	Mat rotation = Mat::zeros(3, 3, CV_64FC1);
//	double epsilonX = R.at<double>(4, 0);
//	double epsilonY = R.at<double>(5, 0);
//	double epsilonZ = R.at<double>(6, 0);
//	rotation.at<double>(0, 1) = epsilonZ;
//	rotation.at<double>(1, 0) = -epsilonZ;
//	rotation.at<double>(0, 2) = -epsilonY;
//	rotation.at<double>(2, 0) = epsilonY;
//	rotation.at<double>(1, 2) = epsilonX;
//	rotation.at<double>(2, 1) = -epsilonX;
//	return translation + (1 + K)*XYZ + rotation*XYZ;
//}