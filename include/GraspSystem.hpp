#include "RealsensePointsCloud.hpp"
#include "Realsense.hpp"
#include "Directory.hpp"
#include "Segmentation.hpp"
#include "Classification.hpp"

#include <thread>
#include <mutex>
#include <condition_variable> 
using std::thread;
using std::mutex;
using std::condition_variable;

using _IDLER_::RealsensePointsCloud;
using _IDLER_::Segmentation;
using _IDLER_::Classification;


typedef vector<PXCPointF32> PXC2DPointSet;
typedef vector<PXCPoint3DF32> PXC3DPointSet;


class GraspSystem: public Directory
{
public:
	GraspSystem(int width, int height, float fps = 30);
	// realsense driver
	int configureRealsense();
	int releaseRealsense();
	int acquireRealsenseData(Mat &color, Mat &depth, Mat &display, vector<PXCPoint3DF32> &pointscloud);
	int captureFrame();
	// drive dobot 
	int dobotCTRL();
	// localize point 
	int graspLocalization();
	/********************************************/
	int testDataSet(string Dir);
	Rect SCRL(	Mat &color,
				Mat &depth,
				Mat &display,
				PXC3DPointSet& pointscloud,
				Classification& classifier	);
	/*******************************************/
private:
	// Callback
	static void selectPoint(int event, int x, int y, int flags, void* paras);
	// Opencv related
	void drawCornerText(const Mat &color, const Mat &depth, const vector<Point2f> &corners);
	void placeWindows(int topk);
	Mat PXCImage2Mat(PXCImage* pxc);
	PXCImage* Mat2PXCImage(Mat& depth);
	PointSet cvt3Dto2D(PXC3DPointSet &ps3d);
	// Dobot related 
	int commandParse(int key);
	Mat calibrationR2D(Mat &color, Mat &depth, vector<PXCPoint3DF32> &pointscloud);
	vector<Point2f> findChessBoardCorners(Mat &color, Mat &depth, Size pattern = { 3, 3 });
	void calArmCoordinate(PXCPoint3DF32 origin, float side);
	// Create chessboard
	Mat makeChessBoard(int pixels, int count);
private:
	// Realsense
	PXCSession *pxcsession_ = 0;
	PXCSenseManager *pxcsm_ = 0;
	PXCCapture::Device *pxcdev_ = 0;
	PXCProjection *projection_ = 0;
	PXCCapture::Sample *sample_ = 0;
	PXCImage *pxcdepth_ = 0 , *pxccolor_ = 0;
	PXCSizeI32 camera_;
	pxcF32 fps_;
	RealsensePointsCloud dw_;
	//Thread synchronize
	Mat color_;
	Mat depth_;
	Mat pcdisp_;
	vector<PXCPoint3DF32> pointscloud_;
	mutex myLock_;
	condition_variable myWait_;
	// ChessBoard
	bool calibrated_ = false;
	Size pattern_ = { 3, 3 };
	vector<Point2f> corners_;
	vector<PXCPoint3DF32> corresponding_;
	PXCPoint3DF32 origin_;
	float side_;
	// Grasp flag
	bool autoLocalization_ = false;
	// Mouse Click to select point
	Point grasppoint_;
	Point preClick_ = { -1, -1 };
	Point click_ = { 0, 0 };
};
