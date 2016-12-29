#include "PointsCloud.hpp"
#include "Realsense.hpp"
#include "FileOperation.hpp"
#include "Segmentation.hpp"

class VideoDriver: public FileOperation
{
public:
	VideoDriver(int width, int height, float fps = 30);
	int configureRealsense();
	int acquireRealsenseData(Mat &color, Mat &depth, vector<PXCPoint3DF32> &pointscloud);

	int dobotCTRL();
	Mat makeChessBoard(int pixels, int count);
	
private:
	int commandParse(int key);
	void placeWindows(int topk);
	Mat PXCImage2Mat(PXCImage* pxc);
	Mat calibrationR2D(Mat &color, Mat &depth, vector<PXCPoint3DF32> &pointscloud);
	vector<Point2f> findChessBoardCorners(Mat &color, Mat &depth, Size pattern = { 3, 3 });
	void calArmCoordinate(PXCPoint3DF32 origin, float side);
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
	PointsCloud dw_;
	// ChessBoard
	bool calibrated_ = false;
	Size pattern = { 3, 3 };
	vector<Point2f> corners_;
	vector<PXCPoint3DF32> corresponding_;
	PXCPoint3DF32 origin_;
	float side_;
	//Grasp
	bool autoLocalization_ = false;



	//HOG_SVM classification_;
};
