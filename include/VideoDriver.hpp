#include <pxcdefs.h>
#include <pxcimage.h>
#include "FileOperation.hpp"
#include "Segmentation.hpp"

class VideoDriver: public FileOperation
{
public:
	VideoDriver(int width, int height, float fps = 30);
	int dobotCTRL();
	Mat makeChessBoard(int pixels, int count);
	
private:
	void placeWindows(int topk);
	Mat PXCImage2Mat(PXCImage* pxc);
	vector<Point2f> findChessBoardCorners(Mat &color, Mat &depth, Size pattern = { 3, 3 });
	void calArmCoordinate(PXCPoint3DF32 origin, float side);
private:
	PXCSizeI32 camera_;
	pxcF32 fps_;
	vector<PXCPoint3DF32> vertices_;

	vector<PXCPoint3DF32> corresponding_;
	PXCPoint3DF32 origin_;
	float side_;

	//HOG_SVM classification_;
};
