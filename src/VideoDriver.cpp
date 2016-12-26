#include "VideoDriver.hpp"
#include "DrawWorld.hpp"
#include "Macro.hpp"
#include "Opencv.hpp"
#include "Realsense.hpp"
#include "HOG-SVM.hpp"
#include "Socket.hpp"
//thread
#include <thread>
using std::thread;

#include <opencv2\core.hpp>

//Press Mouse to select point
void selectPoint(int event, int x, int y, int flags, void* paras)
{
	if (cv::EVENT_LBUTTONDOWN == event){
		Point* pos = (Point*)paras;
		pos->x = x;
		pos->y = y;
	}
	else if (cv::EVENT_RBUTTONDOWN == event){
		MySend("change");
	}
}

//How to build a chessboard
Mat VideoDriver::makeChessBoard(int pixels, int count)
//count: black count
//pixels: for each black or white
{
	int len = 2 * pixels * count;
	Mat chess = Mat::zeros(len, len, CV_8UC3);
	for (int i = 0; i < chess.rows; i++){
		for (int j = 0; j < chess.cols; j++){
			if ((j / pixels + i / pixels) % 2)
				chess.at<Vec3b>(i, j) = { 255, 255, 255 };
		}
	}
	imwrite("chess" + to_string(count) + ".png", chess);
	return chess;
}

//Draw corner points text index
void drawCornerText(const Mat &color, const Mat &depth, const vector<Point2f> &corners)
{
	int i = 1;
	for (auto c : corners){
		short current = depth.at<short>(c);
		putText(color, to_string(i++), c, 1, 2, Scalar(255, 0, 0));
	}
}

//Find chessboard from image
vector<Point2f> VideoDriver::findChessBoardCorners(Mat &color, Mat &depth, Size pattern)
{
	vector<Point2f> corners;
	Mat gray;
	cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
	bool found = cv::findChessboardCorners(color, pattern, corners);
	if (corners.size() > 0){
		cornerSubPix(gray, corners, Size(10, 10), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 500, 0.003));
		cv::drawChessboardCorners(color, pattern, corners, found);
		drawCornerText(color, depth, corners);
		imshow("chess", color);
	}
	return corners;
}

//Calculate arm Coordinate
vector<PXCPoint3DF32> calArmCoordinate(PXCPoint3DF32 origin, float side)
{
	vector<PXCPoint3DF32> corresponding;
	MESSAGE_COUT("Corresponding Points", "");
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			PXCPoint3DF32 tmp = origin;
			tmp.x += j*side;
			tmp.y -= i*side;
			corresponding.push_back(tmp);
			MESSAGE_COUT(i << "," << j, tmp.x << "," << tmp.y << "," << tmp.z);
		}
	}
	int i = 0;
	for (auto c : corresponding)
		MESSAGE_COUT(++i, c.x << "," << c.y << "," << c.z);
	return corresponding;
}

//Create Thread to Show Image 
void myimshow(const string winname, Mat &img)
{
	imshow(winname, img);
}
void threadShow(const string winname, Mat &img)
{
	thread t(myimshow, winname, img);
	t.detach();
}

//Show boundbox and word
inline void drawText(Mat &img, Rect &boundbox, const string content)
{
	putText(img, content, (boundbox.tl() + boundbox.br()) / 2, 3, 0.6, Scalar(0, 0, 255), 2);
}

// Locate windows position
void placeWindows(int topk)
{
	cv::namedWindow("depth");
	cv::namedWindow("color");
	cv::namedWindow("before merging");
	cv::namedWindow("segmentation");
	cv::namedWindow("classification");
	cv::namedWindow("regions");
	cv::moveWindow("depth", 0, 0);
	cv::moveWindow("color", 350, 0);
	cv::moveWindow("segmentation", 1050, 0);
	cv::moveWindow("before-merging", 700, 0);
	cv::moveWindow("classification", 350, 300);
	cv::moveWindow("regions", 0, 300);
	for (int k = 0; k < topk; k++) {
		cv::namedWindow(to_string(k));
		cv::moveWindow(to_string(k), (k + 2) * 350, 300);
	}
}

// Construct
VideoDriver::VideoDriver(int width, int height, float fps)
{
	camera_.width = width;
	camera_.height = height;
}

//Convert RealSense's PXCImage to Opencv's Mat
Mat VideoDriver::PXCImage2Mat(PXCImage* pxc)
{
	if (!pxc)	return Mat(0, 0, 0);
	PXCImage::ImageInfo info = pxc->QueryInfo();
	PXCImage::ImageData data;
	Mat cvt;
	if (info.format & PXCImage::PIXEL_FORMAT_YUY2) {	//color data
		if (pxc->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &data) < PXC_STATUS_NO_ERROR)
			return  Mat(0, 0, 0);
		cvt = Mat(info.height, info.width, CV_8UC3, (void*)data.planes[0],data.pitches[0]/sizeof(uchar));
	}
	else if (info.format & PXCImage::PIXEL_FORMAT_DEPTH) {//depth data
		if (pxc->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &data) < PXC_STATUS_NO_ERROR)
			return  Mat(0, 0, 0);
		cvt = Mat(info.height, info.width, CV_16U, (void*)data.planes[0], data.pitches[0] / sizeof(uchar));	//Mat初始化是按照长宽来定的
	}
	pxc->ReleaseAccess(&data);
	return cvt;
}

//Drive Dobot work
int VideoDriver::dobotCTRL()
{
	// Define variable
	Mat color, depth, display, color2, depth2;
	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
	PXCSession *pxcsession;
	PXCSenseManager *pxcsm;
	PXCCapture::Device *pxcdev;
	PXCProjection *projection;
	PXCCapture::Sample *sample;
	PXCImage *pxcdepth,*pxccolor;
	long framecnt;
	// Configure RealSense
	pxcsession = PXCSession::CreateInstance();
	pxcsm = pxcsession->CreateSenseManager();
	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, camera_.width, camera_.height, fps_);
	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, camera_.width, camera_.height, fps_);
	pxcsm->Init();
	pxcdev = pxcsm->QueryCaptureManager()->QueryDevice();
	if (!pxcdev) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK SenseManager");
		return -1;
	}
	pxcdev->SetDepthConfidenceThreshold(4);
	projection = pxcdev->CreateProjection();
	if (!projection) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK Projection");
		return -2;
	}
	// Configure Point Cloud Show
	DrawWorld dw(pxcsession, camera_);
	PXCPoint3DF32 light = { .5, .5, 1.0 };
	// Configure Segmentation
	unsigned topk = 5;
	short threshold = 2;
	Size segSize = { 320, 240 };
	Segmentation myseg(segSize, topk, threshold);
	// Mouse Click to select point
	Point preClick = { -1, -1 };
	Point click = { 0, 0 };
	cv::namedWindow("color");
	cv::namedWindow("depth");
	cv::setMouseCallback("color", selectPoint, (void*)(&click));
	cv::setMouseCallback("depth", selectPoint, (void*)(&click));
	// Transformation Matrix
	PXCPoint3DF32 origin = { 143.8221f, 4.8719f, -21.0000f };
	float side = 71.0f;
	Size pattern = { 3, 3 };
	vector<Point2f> corners;
	vector<PXCPoint3DF32> corresponding = calArmCoordinate(origin, side);
	Mat trans = Mat::eye(4, 4, CV_32FC1);
	// Calibration Flag
	bool calibrated = false;
	// Detect each video frame
	for (framecnt = 1; true; ++framecnt) {
		if (pxcsm->AcquireFrame(true) < PXC_STATUS_NO_ERROR)	break;
		// Query the realsense color and depth, and project depth to color
		try{
			sample = pxcsm->QuerySample();
			pxcdepth = sample->depth;
			pxccolor = sample->color;
			pxcdepth = projection->CreateDepthImageMappedToColor(pxcdepth, pxccolor);
			// Generate and Show 3D Point Cloud
			pxcStatus sts = projection->QueryVertices(pxcdepth, &vertices[0]);
			if (sts >= PXC_STATUS_NO_ERROR) {
				PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(vertices, pxcdepth, light);
				if (drawVertices){
					Mat display = PXCImage2Mat(drawVertices);
					imshow("display", display);
				}
			}
			depth = PXCImage2Mat(pxcdepth);
			color = PXCImage2Mat(pxccolor);
			if (!depth.cols || !color.cols)	continue;

			//// resize
			//resize(depth, depth2, segSize);
			//resize(color, color2, segSize);
			//// segement
			//myseg.Segment(depth2, color2);

			// judge 
			int key = waitKey(1);
			if (key == ' '){
				corners = findChessBoardCorners(color, depth, pattern);
				if (corners.size() >= 9) {
					Mat src, dst;
					for (int index = 0; index < 9; index++){
						Point2f c = corners[index];
						if (depth.at<short>(c)){
 							PXCPoint3DF32 v = vertices[(int)c.y * camera_.width + (int)c.x];
							Mat tmp = Mat::ones(1, 4, CV_32FC1);
							tmp.at<float>(0, 0) = v.x;
							tmp.at<float>(0, 1) = v.y;
							tmp.at<float>(0, 2) = v.z;
							src.push_back(tmp);
							Mat tmp2 = Mat::ones(1, 4, CV_32FC1);
							PXCPoint3DF32 cor = corresponding[index];
							tmp2.at<float>(0, 0) = cor.x;
							tmp2.at<float>(0, 1) = cor.y;
							tmp2.at<float>(0, 2) = cor.z;
							dst.push_back(tmp2);
						}
						//cout << "[" << c.x << ", " << c.y << "]\t" << "(" << v.x << "," << v.y << "," << v.z << "]" << endl;
					}
					trans = dst.t()*src.t().inv(cv::DECOMP_SVD);
					MESSAGE_COUT("Transformation Matrix", "");
					cout << trans << endl;	
					calibrated = true;
 				}
			}
			else if (key == 'h'){
				MySend("home");
			}
			else if (key == 'q'){
				MySend("quit");
			}
			else if (key == 27){
				break;
			}
			if (preClick != click){
				preClick = click;
				if (calibrated && depth.at<float>(click)) {
					Mat tmp = Mat::ones(1, 4, CV_32FC1);
					PXCPoint3DF32 v = vertices[click.y * camera_.width + click.x];
					tmp.at<float>(0, 0) = v.x;
					tmp.at<float>(0, 1) = v.y;
					tmp.at<float>(0, 2) = v.z;
					Mat res = trans*tmp.t();
					string buf =	to_string(res.at<float>(0, 0)) + " " +
									to_string(res.at<float>(1, 0)) + " " +
									to_string(res.at<float>(2, 0)) + "\n";
					MESSAGE_COUT("Send Msg", buf);
					MySend(buf);
				}
			}


			//draw click point
			cv::circle(color, click, 3, Scalar(255, 0, 0), 5);
			cv::circle(depth, click, 3, Scalar(5000), 5);
			// show
			if (corners.size() >= 9){
				cv::drawChessboardCorners(color, pattern, corners, true);
				drawCornerText(color, depth, corners);
			}
			imshow("depth", 65535 / 1200 * depth);
			imshow("color", color);
			// Clear Segmentation data; 
			myseg.clear();
			// Release Realsense SDK memory and read next frame 
			pxcdepth->Release();
			pxcsm->ReleaseFrame();

		}
		catch (cv::Exception e){
			MESSAGE_COUT("ERROR", e.what());
		}
	}
	return 1;

}

