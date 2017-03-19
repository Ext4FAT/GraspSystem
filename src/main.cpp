#include "GraspSystem.hpp"
#include "Classification.hpp"
#include <windows.h>




int main(int argc, char** argv)
{
	//GraspSystem data(640, 480, 30);
	//data.dobotCTRL();


	_IDLER_::Classification classifier;
	classifier.crossValidation(10, "..\\dataset\\Body\\");






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


