#include "VideoDriver.hpp"
#include "Macro.hpp"
#include "Opencv.hpp"
#include "HOG-SVM.hpp"
#include "Socket.hpp"

#include <opencv2\core.hpp>

//Press Mouse to select point
void VideoDriver::selectPoint(int event, int x, int y, int flags, void* paras)
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

//Draw corner points text index
void VideoDriver::drawCornerText(const Mat &color, const Mat &depth, const vector<Point2f> &corners)
{
	int i = 1;
	for (auto c : corners){
		short current = depth.at<short>(c);
		putText(color, to_string(i++), c, 1, 2, Scalar(255, 0, 0));
	}
}

//Calculate arm Coordinate
void VideoDriver::calArmCoordinate(PXCPoint3DF32 origin, float side)
{
	MESSAGE_COUT("Corresponding Points", "");
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			PXCPoint3DF32 tmp = origin;
			tmp.x += j*side;
			tmp.y -= i*side;
			corresponding_.push_back(tmp);
			MESSAGE_COUT(i << "," << j, tmp.x << "," << tmp.y << "," << tmp.z);
		}
	}
	int i = 0;
	for (auto c : corresponding_)
		MESSAGE_COUT(++i, c.x << "," << c.y << "," << c.z);
}

//Construct
VideoDriver::VideoDriver(int width, int height, float fps)
{
	camera_.width = width;
	camera_.height = height;
	fps_ = fps;
}

//Configure Realsense parameters
int VideoDriver::configureRealsense()
{
	//Configure RealSense
	pxcsession_ = PXCSession::CreateInstance();
	pxcsm_ = pxcsession_->CreateSenseManager();
	pxcsm_->EnableStream(PXCCapture::STREAM_TYPE_COLOR, camera_.width, camera_.height, fps_);
	pxcsm_->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, camera_.width, camera_.height, fps_);
	//Query Information
	pxcsm_->Init();
	pxcdev_ = pxcsm_->QueryCaptureManager()->QueryDevice();
	if (!pxcdev_) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK SenseManager");
		return -1;
	}
	projection_ = pxcdev_->CreateProjection();
	if (!projection_) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK Projection");
		return -2;
	}
	//Configure Pointscloud
	dw_ = PointsCloud(pxcsession_, camera_);
	return 0;
}

//Release Realsense data
int VideoDriver::releaseRealsense()
{
	this->pxcsm_->Close();
	this->pxcsession_->Release();
	this->pxcsm_->Release();
	this->pxcdev_->Release();
	this->projection_->Release();
	this->sample_->ReleaseImages();
	this->pxcdepth_->Release();
	this->pxccolor_->Release();
	return 1;
}



//Query Realsense data
int VideoDriver::acquireRealsenseData(Mat &color, Mat &depth, vector<PXCPoint3DF32> &pointscloud)
{
	sample_ = pxcsm_->QuerySample();
	pxcdepth_ = sample_->depth;
	pxccolor_ = sample_->color;
	pxcdepth_ = projection_->CreateDepthImageMappedToColor(pxcdepth_, pxccolor_);
	depth = PXCImage2Mat(pxcdepth_);
	color = PXCImage2Mat(pxccolor_);
	//// Generate and Show 3D Point Cloud
	//pxcStatus sts = projection_->QueryVertices(pxcdepth_, &pointscloud[0]);
	//if (sts >= PXC_STATUS_NO_ERROR) {
	//	PXCImage* drawVertices = dw_.DepthToWorldByQueryVertices(pointscloud, pxcdepth_);
	//	if (drawVertices){
	//		Mat display = PXCImage2Mat(drawVertices);
	//		imshow("display", display);
	//	}
	//}
	return 0;
}

// Estimate Realsense to Dobot Transformation Matrix
Mat VideoDriver::calibrationR2D(Mat &color, Mat &depth, vector<PXCPoint3DF32> &pointscloud)
{
	Mat trans = Mat::eye(4, 4, CV_32FC1);
 	corners_ = findChessBoardCorners(color, depth, pattern_);
	if (corners_.size() >= 9) {
		Mat src, dst;
		for (int index = 0; index < 9; index++){
			Point2f c = corners_[index];
			if (depth.at<short>(c)){
				PXCPoint3DF32 v = pointscloud[(int)c.y * camera_.width + (int)c.x];
				Mat tmp = Mat::ones(1, 4, CV_32FC1);
				tmp.at<float>(0, 0) = v.x;
				tmp.at<float>(0, 1) = v.y;
				tmp.at<float>(0, 2) = v.z;
				src.push_back(tmp);
				Mat tmp2 = Mat::ones(1, 4, CV_32FC1);
				PXCPoint3DF32 cor = corresponding_[index];
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
		calibrated_ = true;
	}
	return trans;
}


// Parse commond and  execute
int VideoDriver::commandParse(int key)
{
	// judge 
	switch (key){
		case ' ':
			break;
		case 'e':
			autoLocalization_ = !autoLocalization_;
			MESSAGE_COUT((autoLocalization_ ? "Open" : "Close"), "Automatic");
			break;
		case 'g':
			MySend("grasp");
			break;
		case 'h':
			MySend("home");
			break;
		case 'j':
			MySend("down");
			break;	
		case 'k':
			MySend("up");
			break;
		case 'q':
			MySend("quit"); //Quit server
			break;
		case 27:	//ESC
			return 0;
			break;
	}
	return 1;
}

// Locate windows position
void VideoDriver::placeWindows(int topk)
{
	cv::namedWindow("depth");
	cv::namedWindow("color");
	//cv::namedWindow("before merging");
	cv::namedWindow("segmentation");
	//cv::namedWindow("classification");
	cv::namedWindow("regions");
	cv::moveWindow("color", 0, 0);
	cv::moveWindow("depth", 640, 0);
	cv::moveWindow("segmentation", 0, 520);
	//cv::moveWindow("before-merging", 320, );
	//cv::moveWindow("classification", 350, 300);
	cv::moveWindow("regions", 320, 520);
	for (int k = 0; k < topk; k++) {
		cv::namedWindow(to_string(k));
		cv::moveWindow(to_string(k), (k + 2) * 350, 300);
	}
}

string convert(PXCPoint3DF32 v, Mat &trans)
{
	Mat tmp = Mat::ones(1, 4, CV_32FC1);
	tmp.at<float>(0, 0) = v.x;
	tmp.at<float>(0, 1) = v.y;
	tmp.at<float>(0, 2) = v.z;
	Mat res = trans*tmp.t();
	string buf = to_string(res.at<float>(0, 0)) + " " +
		to_string(res.at<float>(1, 0)) + " " +
		to_string(res.at<float>(2, 0)) + "\n";
	return buf;
}


// Segmentation
vector<Rect> VideoDriver::segmentation(Size segSize, unsigned topk, short threshold)
{
	Mat depth2, color2;
	// Configure Segmentation
	Segmentation myseg(segSize, topk, threshold);
	// Resize
	resize(depth_, depth2, segSize);
	resize(color_, color2, segSize);
	// Segment
	myseg.Segment(depth2, color2);
	return myseg.boundBoxes_;
}

// Classification
vector<Rect> VideoDriver::classification(vector<Rect> &regions)
{
	// Load HOG-SVM model
	vector<Rect> filter;
	HOG_SVM classifier(""); // Load some path
	for (auto r : regions){
		Mat roi = color_(r);
		int p = static_cast<int>(classifier.predict(roi));
		if (p == 1)
			filter.push_back(r);
	}
	return filter;
}

/**
 * @brief TODOLIST
 */
int VideoDriver::registration()
{
	return 0;
}

int VideoDriver::Grasp()
{
	Mat depth2, color2;
	Size segSize(320, 240);
	unsigned topk = 6;
	short threshold = 2;
	Segmentation myseg(segSize, topk, threshold);

	while (1){
		//////////////////////////////////////////////
		std::unique_lock<mutex> lk(myLock_);
		myWait_.wait(lk);
		resize(color_, color2, segSize);
		resize(depth_, depth2, segSize);
		//color_ = depth_ = 0;
		lk.unlock();
		/////////////////////////////////////////////
		myseg.Segment(depth2, color2);
		vector<Rect> regions = myseg.boundBoxes_;
		//vector<Rect> filter = classification(regions);

		for (auto r : regions)
			rectangle(color2, r, Scalar(0, 0, 255), 2);
		imshow("regions", color2);

		myseg.clear();
		waitKey(1);
	}
	//imshow("regions", color2);
	return 1;
}

// Capture Frame
int VideoDriver::captureFrame()
{
	// Define variable
	Mat color, depth;
	vector<PXCPoint3DF32> pointscloud(camera_.height*camera_.width);
	// Configure RealSense
	configureRealsense();
	PointsCloud dw(pxcsession_, camera_);
	long framecnt = 0;
	// Estimate Transformation Matrix
	PXCPoint3DF32 origin = { 143.8221f, 4.8719f, -21.0000f };
	float side = 71.0f;
	calArmCoordinate(origin, side);
	Mat trans = Mat::eye(4, 4, CV_32FC1);
	// Detect each video frame
	for (framecnt = 1; true; ++framecnt) {
		if (pxcsm_->AcquireFrame(true) < PXC_STATUS_NO_ERROR)	
			break;
		// Query the realsense color and depth, and pointscloud
		acquireRealsenseData(color, depth, pointscloud);
		// Critical area  /////////////////////
		std::lock_guard<mutex> lck(myLock_);
		color_ = color.clone();
		depth_ = depth.clone();
		myWait_.notify_all();
		///////////////////////////////////////////

		// Keyboard commands parse
		int key = waitKey(1);
		if (key == ' '){ // calibrate the arm ordinary
			trans = calibrationR2D(color, depth, pointscloud);
			if (corners_.size() >= 9){
				cv::drawChessboardCorners(color, pattern_, corners_, true);
				drawCornerText(color, depth, corners_);
			}
		}
		else if (!commandParse(key))
			break;
		// Point data send
		if (preClick_ != click_){
			preClick_ = click_;
			if (calibrated_ && depth.at<float>(click_)) {
				PXCPoint3DF32 v = pointscloud[click_.y * camera_.width + click_.x];
				string buf = convert(v, trans);
				MESSAGE_COUT("Send Msg", buf);
				MySend(buf);
			}
		}
		if (autoLocalization_ && calibrated_ && depth.at<float>(grasppoint_)) {
			if (grasppoint_.x > camera_.width || grasppoint_.y > camera_.height)
				break;
			PXCPoint3DF32 v = pointscloud[grasppoint_.y * camera_.width + grasppoint_.x];
			string buf = convert(v, trans);
			MESSAGE_COUT("Send Msg", buf);
			MySend(buf);
		}
		// Draw click point on source image
		cv::circle(color, grasppoint_, 3, Scalar(0, 0, 255), 5);
		cv::circle(depth, grasppoint_, 3, Scalar(50000), 5);
		cv::circle(color, click_, 3, Scalar(255, 0, 0), 5);
		cv::circle(depth, click_, 3, Scalar(5000), 5);
		// Image show
		imshow("depth", 65535 / 1200 * depth);
		imshow("color", color);
		// Release Realsense SDK memory and read next frame 
		pxcdepth_->Release();
		pxcsm_->ReleaseFrame();
	}
	return 1;
}


//Drive Dobot work
int VideoDriver::dobotCTRL()
{
	// Preparation
	placeWindows(0);
	cv::setMouseCallback("color", selectPoint, (void*)(&click_));
	cv::setMouseCallback("depth", selectPoint, (void*)(&click_));
	//Thread
	thread task(&VideoDriver::Grasp, this);
	//thread master(&VideoDriver::captureFrame, this);
	captureFrame();
	return 1;

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
		cvt = Mat(info.height, info.width, CV_8UC3, (void*)data.planes[0], data.pitches[0] / sizeof(uchar));
	}
	else if (info.format & PXCImage::PIXEL_FORMAT_DEPTH) {//depth data
		if (pxc->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &data) < PXC_STATUS_NO_ERROR)
			return  Mat(0, 0, 0);
		cvt = Mat(info.height, info.width, CV_16U, (void*)data.planes[0], data.pitches[0] / sizeof(uchar));	//Mat初始化是按照长宽来定的
	}
	pxc->ReleaseAccess(&data);
	return cvt;
}

//Find chessboard from image
vector<Point2f> VideoDriver::findChessBoardCorners(Mat &color, Mat &depth, Size pattern)
{
	vector<Point2f> corners;
	Mat gray;
	cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
	bool found = cv::findChessboardCorners(color, pattern, corners);
	for (auto c : corners){
		cout << c << endl;
	}
	cout << corners.size() << endl;
	if (corners.size() > 0){
		cornerSubPix(gray, corners, Size(10, 10), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 500, 0.003));
		cv::drawChessboardCorners(color, pattern, corners, found);
		drawCornerText(color, depth, corners);
		imshow("chess", color);
	}
	return corners;
}

//How to build a chessboard
Mat VideoDriver::makeChessBoard(int pixels, int count)
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
