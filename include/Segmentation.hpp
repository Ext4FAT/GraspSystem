#include "Opencv.hpp"

//#define imshow(_W_, _M_) imwrite(_W_+std::string(".png"), _M_)

namespace _IDLER_{
	class Segmentation
	{
	public:
		Segmentation(int width, int height, unsigned topk, short t);
		Segmentation(Size sz, unsigned topk, short t);

		void Segment(Mat& depth, Mat& color);
		void DFS(Mat& depth, Mat& visit, Point cur, short& threshold, vector<Point>& v);
		void NonRecursive(Mat& depth, Mat& visit, Point& cur, PointSet& pSet);
		void completeDepth(Mat& depth);
		void clear();
	private:
		void regionMerge(Mat& depth, SegmentSet& segment, SegmentSet& blackRegions, unsigned topk, double minSim);
		inline void calculateConvexHulls();
		inline void calculateBoundBoxes();
		inline Rect hullBoundBox(PointSet& hull);

		inline bool isRegionInsideHull(PointSet& pSet, PointSet& hull, double minSim);
		inline bool isRegionInsideHull(PointSet& pSet, PointSet& hull, PointSet& seg, double minSim);

		void randColor();

		Mat Segmentation::draw();

	public:
		//
		inline const SegmentSet& mainSegmentation() { return mainSeg_; }
	private:
		// 
		SegmentSet mainSeg_;
		SegmentSet blackSeg_;
		vector<PointSet> convexHulls_;
		vector<Rect> boundBoxes_;
		vector<double> distance_;
		//
		static const vector<Point> _DIRECTIONS_;
		static const vector<Point> _DIR_;
		vector<Vec3b> colors_;
		Rect RANGE_;
		short threshold_;
		unsigned topk_;
	};
}