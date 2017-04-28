#include "GraspSystem.hpp"
#include "Macro.hpp"
#include "Opencv.hpp"
#include "Classification.hpp"
#include "Socket.hpp" 
#include "RegistrationPCL.hpp"
#include <opencv2\core.hpp>

//#include <pcl/io/pcd_io.h>

using namespace _IDLER_;

#include "Speaker.hpp"


int savePCD(const string& outfilename, PointCloudNT::Ptr &myseg, double scale = 1. / 330)
{
	ofstream ofs(outfilename);
	ofs << "# .PCD v0.7 - Point Cloud Data file format" << endl;
	ofs << "VERSION 0.7" << endl;
	ofs << "FIELDS x y z" << endl;
	ofs << "SIZE 4 4 4" << endl;
	ofs << "TYPE F F F" << endl;
	ofs << "COUNT 1 1 1" << endl;
	ofs << "WIDTH " << myseg->size() << endl;
	ofs << "HEIGHT 1" << endl;
	ofs << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	ofs << "POINTS " << myseg->size() << endl;
	ofs << "DATA ascii" << endl;
	for (auto p : *myseg) 
		ofs << p.x << " " << p.y << " " << p.z << endl;
	ofs.close();
	return 0;
}

// Convert Realsense's PXC to PCL's PointCloud
size_t PXC2PCL(PointSet &pSet, vector<PXCPoint3DF32> &vertices, PointCloudNT::Ptr &scene, double scale = 1.f / 300.f)
{
	for (auto& p : pSet) {
		p += p;
		PXCPoint3DF32 ppp = vertices[p.y * 640 + p.x];
		scene->push_back(PointNT());
		PointNT& ps = scene->back();
		ps.x = ppp.x*scale;
		ps.y = ppp.y*scale;
		ps.z = ppp.z*scale;
	}

	for (auto p : vertices){
		scene->push_back(PointNT());
		PointNT& ps = scene->back();
		ps.x = p.x;
		ps.y = p.y;
		ps.z = p.z;
	}


	return scene->size();
}

PXC3DPointSet PCL2PXC(PointCloudNT::Ptr &pointscloud, double scale)
{
	PXC3DPointSet result;
	for (auto &pc : *pointscloud)
		result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
	return result;
}

PXC3DPointSet PCL2PXC(PointCloudT::Ptr &pointscloud, double scale)
{
	PXC3DPointSet result;
	for (auto &pc : *pointscloud)
		result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
	return result;
}

Rect operator/(const Rect &r, int scale)
{
	return Rect(r.x / 2, r.y / 2, r.width / 2, r.height / 2);
}



//
//// genRegistration
//Reflect_Result genRegistrationResult(	PXCProjection *projection,
//										PointCloudNT::Ptr &model,
//										PointCloudT::Ptr &grasp,
//										PointSet &segment,
//										vector<PXCPoint3DF32> &vertices,
//										double scale,
//										RegisterParameter &para)
//{
//	//generate Point Cloud
//	PointCloudNT::Ptr mesh(new PointCloudNT);
//	PointCloudNT::Ptr model_align(new PointCloudNT);
//	PointCloudT::Ptr grasp_align(new PointCloudT);
//	size_t sz = PXC2PCL(segment, vertices, mesh, 1.0 / scale);
//	MESSAGE_COUT("INFO", "Generate Point Cloud: " << sz);
//	//Alignment
//	Matrix4f transformation;
//	transformation = RegistrationNoShow(model, mesh, model_align, para);
//	if (transformation == Matrix4f::Identity()) //Alignment failed 
//		return{};
//	pcl::transformPointCloud(*grasp, *grasp_align, transformation);
//	//Reflect
//	Reflect_Result show2d;
//	show2d.model.resize(model_align->size());
//	show2d.grasp.resize(grasp_align->size());
//	vector<PXCPoint3DF32> result;
//	for (auto &pc : *model_align) {
//		result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
//	}
//	projection->ProjectCameraToDepth(result.size(), &result[0], &show2d.model[0]);
//	result.clear();
//	for (auto &pc : *grasp_align) {
//		result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
//	}
//	projection->ProjectCameraToDepth(result.size(), &result[0], &show2d.grasp[0]);
//	return show2d;
//}
//
//
//
//// show registration image
//void showRegistrationResult(const vector<PXCPointF32> &show2d, Mat &img, Vec3b color)
//{
//	for (auto p : show2d) {
//		Point pp(p.x, p.y);
//		if (pp.inside(Rect(0, 0, 640, 480))) {
//			img.at<Vec3b>(pp) = color;
//		}
//	}
//	imshow("reflect", img);
//}
//
//bool Reflect(	long framecnt,
//	string name,
//	Mat& img,
//	PXCProjection *projection,
//	PointCloudNT::Ptr &model,
//	PointCloudT::Ptr &grasp,
//	PointSet &segment,
//	vector<PXCPoint3DF32> &vertices,
//	double scale,
//	RegisterParameter &para)
//{
//	MESSAGE_COUT("[" << framecnt << "]", name);
//	Mat color = img.clone();
//	//vector<PXCPointF32> show2d = genRegistrationResult(projection_, model, myseg, vertices, PointCloudScale, leaf);
//
//	Reflect_Result show2d = genRegistrationResult(projection, model, grasp, segment, vertices, scale, para);
//
//	if (!show2d.isEmpty()) {
//		/*static variable*/
//		static Rect __range__ = { 0, 0, 640, 480 };
//		Rect boundbox = myBoundBox(show2d.grasp);
//		boundbox &= __range__;
//		rectangle(color, boundbox, Scalar(255, 0, 0), 2);
//		showRegistrationResult(show2d.model, color, Vec3b(255, 0, 255));
//		showRegistrationResult(show2d.grasp, color, Vec3b(0, 255, 255));
//		return true;
//	}
//}


//Press Mouse to select point
void GraspSystem::selectPoint(int event, int x, int y, int flags, void* paras)
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
void GraspSystem::drawCornerText(const Mat &color, const Mat &depth, const vector<Point2f> &corners)
{
	int i = 1;
	for (auto c : corners){
		short current = depth.at<short>(c);
		putText(color, to_string(i++), c, 1, 2, Scalar(255, 0, 0));
	}
}

//Calculate arm Coordinate
void GraspSystem::calArmCoordinate(PXCPoint3DF32 origin, float side)
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
GraspSystem::GraspSystem(int width, int height, float fps)
{
	camera_.width = width;
	camera_.height = height;
	fps_ = fps;
}

//Configure Realsense parameters
int GraspSystem::configureRealsense()
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
	dw_ = RealsensePointsCloud(pxcsession_, camera_);
	return 0;
}

//Release Realsense data
int GraspSystem::releaseRealsense()
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
int GraspSystem::acquireRealsenseData(Mat &color, Mat &depth, Mat &display, vector<PXCPoint3DF32> &pointscloud)
{
	sample_ = pxcsm_->QuerySample();
	pxcdepth_ = sample_->depth;
	pxccolor_ = sample_->color;
	pxcdepth_ = projection_->CreateDepthImageMappedToColor(pxcdepth_, pxccolor_);
	depth = PXCImage2Mat(pxcdepth_);
	color = PXCImage2Mat(pxccolor_);
	// Generate and Show 3D Point Cloud
	pxcStatus sts = projection_->QueryVertices(pxcdepth_, &pointscloud[0]);
	if (sts >= PXC_STATUS_NO_ERROR) {
		PXCImage* drawVertices = dw_.DepthToWorldByQueryVertices(pointscloud, pxcdepth_);
		if (drawVertices)
			display = PXCImage2Mat(drawVertices);
	}
	return 0;
}

// Estimate Realsense to Dobot Transformation Matrix
Mat GraspSystem::calibrationR2D(Mat &color, Mat &depth, vector<PXCPoint3DF32> &pointscloud)
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
int GraspSystem::commandParse(int key)
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
void GraspSystem::placeWindows(int topk)
{
	cv::namedWindow("depth");
	cv::namedWindow("color");
	cv::namedWindow("pointscloud");
	//cv::namedWindow("before merging");
	cv::namedWindow("segmentation");
	//cv::namedWindow("classification");
	cv::namedWindow("regions");
	cv::moveWindow("color", 0, 0);
	cv::moveWindow("depth", 640, 0);
	cv::moveWindow("segmentation", 0, 520);
	cv::moveWindow("regions", 320, 520);
	cv::moveWindow("pointscloud", 640, 520);
	//cv::moveWindow("before-merging", 320, );
	//cv::moveWindow("classification", 350, 300);
	
	for (int k = 0; k < topk; k++) {
		cv::namedWindow(to_string(k));
		cv::moveWindow(to_string(k), (k + 2) * 350, 300);
	}
}

// Convert coordinate system
string cvtCoordinate(PXCPoint3DF32 v, Mat &trans)
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

/**
 * @brief TODOLIST
 */

PointSet GraspSystem::cvt3Dto2D(PXC3DPointSet &ps3d)
{
	static Rect range = Rect(0, 0, camera_.width, camera_.height);
	PXC2DPointSet ps2d(ps3d.size());
	projection_->ProjectCameraToDepth(ps3d.size(), &ps3d[0], &ps2d[0]);
	PointSet ps;
	for (auto p : ps2d){
		Point2f pt(p.x, p.y);
		if (Point2f(p.x, p.y).inside(range))
			ps.push_back(pt);
	}
	return ps;
}


int GraspSystem::graspLocalization()
{
	// speech
	Speaker speech;
	// decline 
	long framecnt;
	clock_t start, end, sum;
	Mat color, depth, display;
	Mat  color2, depth2, display2;
	vector<PXCPoint3DF32> pointscloud;
	// configure segmentation
	Size segSize(320, 240);
	unsigned topk = 6;
	short threshold = 2;
	Segmentation myseg(segSize, topk, threshold);
	// configure classification
	Classification classifier("..\\classifier\\object.xml");
	classifier.setCategory({ "background", "cup" });
	static vector<Scalar> drawColor = { Scalar(255, 0, 0), Scalar(0, 0, 255) };
	// configure registration
	RegisterParameter para;
	RegistrationPCL ransac(para);
	ransac.Preparation({"papercup"});
	// loop
	for (framecnt = 0; 1; framecnt++) {
		// critical area
		std::unique_lock<mutex> lk(myLock_);
		myWait_.wait(lk);
		color = color_.clone();
		depth = depth_.clone();
		display = pcdisp_.clone();
		pointscloud = pointscloud_;
		lk.unlock();
		// resize
		resize(color, color2, segSize);
		resize(depth, depth2, segSize);
		resize(display, display2, segSize);

		// segmentation
		myseg.Segment(depth2, color2);
		const SegmentSet& mainSeg = myseg.mainSegmentation();
		// classification
		Category categories = classifier.category();
		for (auto ms : mainSeg){
			Rect r = cv::boundingRect(ms);
			Mat roi = color2(r);
			int p = classifier.predict(roi);
			if (p){
				double scale = 1.0 / 350;
				Mat ROI;
				Mat mask = Mat::zeros(segSize, CV_8UC1);
				mask(r).setTo(255);
				display2.copyTo(ROI, mask);
				imshow("pointscloud", ROI);
				// registration
				PointCloudNT::Ptr seg(new PointCloudNT);
				size_t sz = PXC2PCL(ms, pointscloud, seg, scale);
				MESSAGE_COUT("INFO", "Generate Point Cloud: " << sz);
				Matrix4f transformation = ransac.Apply(seg, p);
				// reflect
				PointCloudNT::Ptr model_align(new PointCloudNT);
				PointCloudT::Ptr grasp_align(new PointCloudT);
				ransac.Transform(transformation, model_align, grasp_align, p);
				// convert Points cloud to PXC3DPoint set
				PXC3DPointSet model3d = PCL2PXC(model_align, scale);
				PXC3DPointSet grasp3d = PCL2PXC(grasp_align, scale);
				// query 2D points
				PointSet model2d = cvt3Dto2D(model3d);
				PointSet grasp2d = cvt3Dto2D(grasp3d);
				Rect mRect = boundingRect(model2d);
				Rect gRect = boundingRect(grasp2d);
				//
				double sim = 1.0 * ((mRect / 2)&r).area() / ((mRect / 2) | r).area();
				if (sim > 0.85){
					for (auto m : model2d)
						color.at<Vec3b>(m) = COLOR_MODEL;
					for (auto g : grasp2d)
						color.at<Vec3b>(g) = COLOR_GRASP;
					speech.speak(GRASP_REG_SUCCESS);
					imshow("reflect", color);
					speech.speak(GRASP_ACTION);
				}
			}
			rectangle(color2, r, drawColor[p], 2);
		}
		imshow("regions", color2);
		myseg.clear();
		waitKey(1);
	}
	return 1;
}

// Capture Frame
int GraspSystem::captureFrame()
{
	// Define variable
	clock_t start, end;
	Mat color, depth, display;
	vector<PXCPoint3DF32> pointscloud(camera_.height*camera_.width);
	// Configure RealSense
	configureRealsense();
	RealsensePointsCloud dw(pxcsession_, camera_);
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
		start = clock();
		acquireRealsenseData(color, depth, display, pointscloud);
		// Critical area  /////////////////////
		std::lock_guard<mutex> lck(myLock_);
		color_ = color.clone();
		depth_ = depth.clone();
		pcdisp_ = display.clone();
		pointscloud_ = pointscloud;
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
				string buf = cvtCoordinate(v, trans);
				MESSAGE_COUT("Send Msg", buf);
				MySend(buf);
			}
		}
		if (autoLocalization_ && calibrated_ && depth.at<float>(grasppoint_)) {
			if (grasppoint_.x > camera_.width || grasppoint_.y > camera_.height)
				break;
			PXCPoint3DF32 v = pointscloud[grasppoint_.y * camera_.width + grasppoint_.x];
			string buf = cvtCoordinate(v, trans);
			MESSAGE_COUT("Send Msg", buf);
			MySend(buf);
		}
		// Draw click point on source image

		end = clock();
		double time = 1.0*(end - start) / CLOCKS_PER_SEC;
		string curfps = "FPS:" + to_string((int)(1 / time));
		cv::putText(color, curfps, { 0, 26 }, 2, 1.0, Scalar(0, 0, 0),2);
		
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
int GraspSystem::dobotCTRL()
{
	// Preparation
	placeWindows(0);
	cv::setMouseCallback("color", selectPoint, (void*)(&click_));
	cv::setMouseCallback("depth", selectPoint, (void*)(&click_));
	//
	Speaker speech;
	speech.speak(GRASP_START);
	//Thread
	thread task(&GraspSystem::graspLocalization, this);
	//thread master(&VideoDriver::captureFrame, this);
	captureFrame();
	return 1;

}


//Convert RealSense's PXCImage to Opencv's Mat
Mat GraspSystem::PXCImage2Mat(PXCImage* pxc)
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
vector<Point2f> GraspSystem::findChessBoardCorners(Mat &color, Mat &depth, Size pattern)
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
Mat GraspSystem::makeChessBoard(int pixels, int count)
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
