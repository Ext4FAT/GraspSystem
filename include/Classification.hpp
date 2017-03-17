#pragma once

#include "FileOperation.hpp"
#include "Opencv.hpp"

using cv::HOGDescriptor;
using cv::Algorithm;
using cv::TermCriteria;
using cv::Ptr;
using cv::ml::TrainData;
using cv::ml::ROW_SAMPLE;
using cv::ml::SVM;

typedef HOGDescriptor MyFeature;
typedef SVM MyClassifier;

/*******************************************************************************
*   Label image data with path when testing, output which image error          *
*******************************************************************************/
namespace _IDLER_{
	class GroundTruth {
	public:
		int label;
		string imgname;
		GroundTruth(int c, string path){
			label = c;
			imgname = path;
		};
	};

	/*******************************************************************************
	*   Mapping category name and index 										   *
	*******************************************************************************/
	class Category {
	public:
		Category(){}

		Category(vector<string> names) {
			int index = 1;
			sort(names.begin(), names.end());
			for (auto &sd : names) {
				name2index[sd] = index;
				index2name[index] = sd;
				index++;
			}
		}
		int operator()(vector<string> names){
			clear();
			int index = 1;
			sort(names.begin(), names.end());
			for (auto &sd : names) {
				name2index[sd] = index;
				index2name[index] = sd;
				index++;
			}
		}

		const string& operator[](int index){
			return index2name[index];
		}
		int operator[](string name){
			return name2index[name];
		}
		int size(){
			return index2name.size() == name2index.size() ? static_cast<int>(index2name.size()) : -1;
		}
		void clear() {
			index2name.clear();
			name2index.clear();
		}

	private:
		map<int, string> index2name;
		map<string, int> name2index;
	};


	class DataSeq {

	};

	/*******************************************************************************
	*   Utilize HOG as feature, and SVM as machine learning model.				   *
	*******************************************************************************/
	class Classification : public FileOperation
	{
	public:
		/** @brief default constructor */
		Classification();
		/**
		 * @brief Classification: load xml file as model
		 * @param model_path    xml loacation
		 */
		Classification(const string model_path);
		/**
		 * @brief loadModel: load xml file as model
		 * @param model_path    xml loacation
		 * @return success/ fail
		 */
		bool loadModel(const string model_path);
		/**
		 * @brief extractFeature: extract HOG feature
		 * @param Img   input source
		 * @param mrs   scaled size
		 * @return      feature vector
		 */
		Mat extractFeature(Mat Img, Size mrs);
		/**
		* @brief getCategory: get category from sub dir names
		* @param subdirs	categorynames
		* @return			category number
		*/
		int getCategory(vector<string> &subdirs);
		/**
		 * @brief getDataSet: get dataset
		 * @param data_path     image location
		 * @return      feature matrix
		 */
		int getDataSet(string dir, int gt);
		int getDataSet(vector<string> &data_path, double gt);
		int getDataSet(vector<string> &data_path, vector<int> &seq, int num, int k, double gt);
		Mat getDataSet(vector<string> &data_path, vector<GroundTruth>& gt, int c);
		/**
		 * @brief setSvmParameter: set training Parameter
		 * @param sv_num    max support vectors number
		 * @param c_r_type  classification/ regression
		 * @param kernel    linear / gussian / ploy ...
		 * @param gamma     if gussian need to set
		 * @return          1
		 */
		int setSvmParameter(int sv_num, int c_r_type, int kernel, double gamma);
		/**
		 * @brief training: training with svm
		 * @param trainSet  train data mat
		 * @param label     label mat
		 * @param save      save model or not
		 * @param dir       save path
		 * @return          1
		 */
		int training(Mat& trainSet, Mat& label, bool save, string dir);
		/**
		 * @brief testing: if need to test, you can use this function
		 * @param testSet   test matirx
		 * @param gt        groundtruth, the othe impletementation is to show which are predicted error
		 * @return          1
		 */
		int testing(Mat& testSet, float gt);
		int testing(Mat& testSet, vector<GroundTruth> gt);
		/**
		 * @brief EndToEnd: the whole process, training and testing
		 * @param data_path datapath
		 * @return  error rate in the test data
		 */
		float EndToEnd(string data_path);
		/**
		 * @brief predict: predict label in practical application
		 * @param image    image / region need to classify
		 * @return label
		 */
		float predict(Mat& image);
		/**
		* @brief crossValidation: predict label in practical application
		* @param k		k-cross-validation
		*/
		void crossValidation(int k);
		/**
		* @brief releaseTrainSet: clear trainMat_ and labels_
		*/
		inline void releaseTrainSet();
		/**
		* @brief clearALL: release Pointer svm_, clear catergory_, trainMat_ and labels_
		*/
		inline void clearALL();
		/**
		* @brief get members const
		*/
		const Category& category(){
			return catergory_;
		}

		const Ptr<SVM>& svm() {
			return svm_;
		}

	private:
		Ptr<SVM> svm_;
		Category catergory_;
		Mat trainMat_;
		Mat labels_;
		Size hog_ = { 64, 64 };
	};
}