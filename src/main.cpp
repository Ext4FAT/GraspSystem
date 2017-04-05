#include "GraspSystem.hpp"
#include "Classification.hpp"
#include <windows.h>




int main(int argc, char** argv)
{
	// Grasping demo
	GraspSystem data(640, 480, 30);
	data.dobotCTRL();
	///////////////////////////////////////////////////////////////////////////



	/////////////////////////////////////////////////////////////////////////////

	//Mat color =  imread("C:\\Users\\IDLER\\Desktop\\modelSet\\chessboard.jpg");
	//cv::imshow("xxx", color);
	//waitKey(-1);
	//vector<Point2f> corners;
	//Mat gray;
	//cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
	//bool found = cv::findChessboardCorners(gray, { 11, 11 }, corners, 1);
	//for (auto c : corners){
	//	cout << c << endl;
	//}



	//_IDLER_::Classification classifier;
	//classifier.crossValidation(10, "..\\dataset\\Object\\");






	return 1;
}


////ensemble testing
//std::ofstream fcout("xxx.csv");
//string datasetp = "..\\dataset\\Body\\";
//string classifierp = "..\\classifier\\";
//
//vector<_IDLER_::Classification> classifiers(10);
//int cnt = 0;
//
//for (auto &c : classifiers)
//c.loadModel(classifierp + to_string(++cnt) + ".xml");
//FileOperation fo;
//vector<string> objects = fo.getSubdirName(datasetp);
//for (auto o : objects){
//	MESSAGE_COUT("INFO", o);
//	vector<string> paths = fo.getCurdirFilePath(datasetp + o + "\\");
//	int samplecnt = 0;
//	for (auto p : paths){
//		Mat sample = imread(p);
//		vector<int> res;
//		for (auto &c : classifiers){
//			int p = c.predict(sample);
//			res.push_back(p);
//		}
//		for (auto r : res)
//			fcout << r << ",";
//		fcout << endl;
//	}
//}
//fcout.close();


