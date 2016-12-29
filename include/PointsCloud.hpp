#include <pxcimage.h>
#include <pxcprojection.h>
#include <pxcsession.h>

#include <opencv2\core\types.hpp>
#include <vector>

#define MAXBYTE 0xff

class PointsCloud{
public:
	PointsCloud();//Potential Danger !!!!!!
	PointsCloud(PXCSession* s, PXCSizeI32 user_size, PXCPoint3DF32 l = { .5, .5, 1.0 });
	~PointsCloud();
public:
	PXCImage* DepthToWorldByQueryVertices(std::vector<PXCPoint3DF32>& vertices, PXCImage *depth);
	PXCImage* SegmentationWorld(std::vector<PXCPoint3DF32>& vertices, PXCImage *depth, std::vector<cv::Point> seg);
private:
	void init();
	void norm(PXCPoint3DF32 &v);
	float dot(PXCPoint3DF32 &v0, PXCPoint3DF32 &v1);
	PXCPoint3DF32 cross(PXCPoint3DF32 &v0, PXCPoint3DF32 &v1);
private:
	PXCSession* session;
	PXCImage *drawVertices;
	std::vector<PXCPoint3DF32> vertices; 
	PXCSizeI32 depthSize;
	PXCPoint3DF32 light;
};