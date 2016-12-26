#include <pxcdefs.h>
#include <pxcimage.h>
#include "FileOperation.hpp"
#include "Segmentation.hpp"

class VideoDriver: public FileOperation
{
public:
	VideoDriver(int width, int height, float fps = 30);
	int dobotCTRL();

	Mat PXCImage2Mat(PXCImage* pxc);
	Mat makeChessBoard(int pixels, int count);
	vector<Point2f> findChessBoardCorners(Mat &color, Mat &depth, Size pattern = { 3, 3 });



private:
	PXCSizeI32 camera_;
	pxcF32 fps_;
	vector<PXCPoint3DF32> vertices_;
	//HOG_SVM classification_;
};
