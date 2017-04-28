#include "Segmentation.hpp"
using namespace _IDLER_;

const vector<Point> Segmentation::_DIRECTIONS_ = {	Point(1, 0), Point(0, -1),
													Point(-1, 0), Point(0, 1),
													Point(1, 1), Point(-1, 1),
													Point(1, -1), Point(-1, -1) }; // search directions
const vector<Point> Segmentation::_DIR_ = { Point(1, 0), Point(0, 1), Point(1, 1) }; // complete depth directions


Segmentation::Segmentation(int width, int height, unsigned k, short t)
{
	RANGE_.width = width;
	RANGE_.height = height;
	threshold_ = t;
	topk_ = k;
	//Generate random colors
	this->randColor();
}

Segmentation::Segmentation(Size sz, unsigned k, short t)
{
	RANGE_.width = sz.width;
	RANGE_.height = sz.height;
	threshold_ = t;
	topk_ = k;
	//Generate random colors
	this->randColor();
}

void Segmentation::Segment(Mat& depth, Mat& color)
{
	Mat visit = Mat::zeros(depth.size(), CV_8U);
	//Region Growing
	for (int i = 0; i < RANGE_.width; i++)
		for (int j = 0; j < RANGE_.height; j++) {
			Point current(i, j);
			if (!visit.at<char>(current)) {
				short value = depth.at<short>(current);
				//insert segment
				if (value) {
					PointSet pSet;
					NonRecursive(depth, visit, current, pSet);
					mainSeg_.push_back(pSet);
				}
			}
		}
	sort(mainSeg_.begin(), mainSeg_.end(),
		[](const vector<Point>& v1, const vector<Point>& v2){return v1.size() > v2.size(); });
	// just select top-k
	mainSeg_.resize(topk_);
	// show segmentations
	Mat disp = draw();
	imshow("segmentation", disp);
	//// calc boundbox
	//for (auto mr : mainRegions_) {
	//	Rect r = boundingRect(mr);
	//	boundBoxes_.push_back(r);
	//}
}

void Segmentation::DFS(Mat &depth, Mat &visit, Point cur, short &threshold, vector<Point> &v)
{
	for (auto d : _DIRECTIONS_){
		Point next = cur + d;
		if (next.inside(RANGE_))
			if (!visit.at<char>(next))
				if (abs(depth.at<short>(next) -depth.at<short>(cur)) < threshold) {
					v.push_back(next);
					visit.at<uchar>(next) = 255;
					DFS(depth, visit, next, threshold, v);
				}
	}
}

void Segmentation::NonRecursive(Mat &depth, Mat& visit, Point& current, PointSet& pSet)
{
	stack<Point> pstack;
	Point p, next;

	pstack.push(current);
	pSet.push_back(current);
	visit.at<uchar>(current) = 255;

	while (!pstack.empty()) {
		p = pstack.top();
		pstack.pop();

		for (auto d : _DIRECTIONS_) {
			next = p + d;
			if (!next.inside(RANGE_))
				continue;
			if (visit.at<char>(next))
				continue;
			if (abs(depth.at<short>(next)-depth.at<short>(p)) >= threshold_)
				continue;
			visit.at<uchar>(next) = 255;
			pstack.push(next);
			pSet.push_back(next);
		}
	}
}

void Segmentation::completeDepth(Mat &depth)
{
	static int WIDTH = RANGE_.width * 2 - 1, HEIGHT = RANGE_.height * 2 - 1;
	short v;
	for (int i = 1; i < WIDTH; i++)
		for (int j = 1; j < HEIGHT; j++) {
			Point current(i, j);
			short& value = depth.at<short>(current);
			if (value)	continue;
			for (auto d : _DIR_) {
				v = depth.at<short>(d + current);
				if (v) {
					//depth.at<short>(current) = v;
					value = v;
					break;
				}
			}
		}
}


void Segmentation::regionMerge(Mat& depth, SegmentSet& segment, SegmentSet& blackRegions, unsigned topk, double minSim)
{
	unsigned i, j;
	topk = topk > segment.size() ? segment.size() : topk;
	//
	convexHulls_.clear();
	boundBoxes_.clear();
	distance_.clear();
	//find convexHull of each region
	for (auto seg : segment) {
		//calculate convex hull
		vector<Point> hull;
		convexHull(seg, hull, false);
		convexHulls_.push_back(hull);
		//find convex hull bounding box
		Rect r = boundingRect(hull);
		boundBoxes_.push_back(r);
		////calculate distance
		//double dis = 0;
		//for (auto p : seg)
		//	dis += depth.at<short>(p);
		//distance_.push_back(dis / seg.size());
	}
	//fill black regions
	for (int k = 0; k < topk; k++){
		Rect r = boundBoxes_[k];
		PointSet& hull = convexHulls_[k];
		PointSet& ms = mainSeg_[k];
		Point tl = r.tl();
		Point br = r.br();
		for (int i = tl.x; i <= br.x; i++){
			for (int j = tl.y; j <= br.y; j++){
				Point p = Point(i, j);
				if (pointPolygonTest(hull, p, false) >= 0) {
					ms.push_back(p);
				}
			}
		}
	}
	//merge small regions
	for (j = topk; j < segment.size(); j++)
		for (i = 0; i < topk; i++)
			isRegionInsideHull(segment[j], hullSet[i], segment[i], minSim);
	//remain topk segment
	segment.resize(topk);
}


inline Rect Segmentation::hullBoundBox(PointSet& hull)
{
	static Point extend(5, 5);
	Point pmax(0, 0), pmin(0x7fffffff, 0x7fffffff);
	for (auto p : hull) {
		if (p.x > pmax.x) pmax.x = p.x;
		if (p.x < pmin.x) pmin.x = p.x;
		if (p.y > pmax.y) pmax.y = p.y;
		if (p.y < pmin.y) pmin.y = p.y;
	}
	return Rect(pmin - extend, pmax + extend) & RANGE_;
}

inline void Segmentation::calculateConvexHulls()
{
	for (auto &seg : mainSeg_) {
		convexHulls_.push_back(PointSet());
		convexHull(seg, convexHulls_.back(), false);
	}
}

inline void Segmentation::calculateBoundBoxes()
{
	for (auto &hull : convexHulls_)
		boundBoxes_.push_back(hullBoundBox(hull));
}

inline bool Segmentation::isRegionInsideHull(PointSet& pSet, PointSet& hull, double minSim)
{
	unsigned sum = 0;
	for (auto p : pSet)
		if (pointPolygonTest(hull, p, false) >= 0)
			sum++;
	return sum >= minSim * pSet.size();
}

inline bool Segmentation::isRegionInsideHull(PointSet& pSet, PointSet& hull, PointSet& seg, double minSim)
{
	unsigned sum = 0;
	for (auto p : pSet)
		if (pointPolygonTest(hull, p, false) >= 0) {
			seg.push_back(p);
			sum++;
		}
	return sum >= minSim * pSet.size();
}

void Segmentation::randColor()
{
	colors_.push_back(Vec3b(255, 0, 0));
	colors_.push_back(Vec3b(0, 255, 0));
	colors_.push_back(Vec3b(0, 0, 255));
	colors_.push_back(Vec3b(0, 255, 255));
	colors_.push_back(Vec3b(255, 0, 255));
	colors_.push_back(Vec3b(255, 255, 0));
	for (int i = 0; i < 10000; i++)
		colors_.push_back(Vec3b(rand() % 255, rand() % 255, rand() % 255));
}

void Segmentation::clear()
{
	this->mainSeg_.clear();
	this->blackSeg_.clear();
	this->distance_.clear();
	this->convexHulls_.clear();
	this->boundBoxes_.clear();
}

Mat _IDLER_::Segmentation::draw()
{
	Mat disp = Mat::zeros(RANGE_.size(), CV_8UC3);
	int count = 0;
	for (auto seg : mainSeg_) {
		for (auto p : seg)
			disp.at<Vec3b>(p) = colors_[count];
		if (++count >= topk_)
			break;
	}
	return disp;
}
