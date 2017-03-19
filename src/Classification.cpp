#include "Classification.hpp"
#include "Macro.hpp"
using namespace _IDLER_;


Classification::Classification()
{
    svm_ = SVM::create();
	TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS, 200, FLT_EPSILON);
	svm_->setType(cv::ml::SVM::C_SVC);
	svm_->setKernel(cv::ml::SVM::LINEAR);
	svm_->setTermCriteria(criteria);
}

Classification::Classification(const string model_path)
{
    svm_ = Algorithm::load<SVM>(model_path);
}

inline void Classification::releaseTrainSet()
{
	trainMat_.release();
	labels_.release();
}

inline void Classification::clearALL()
{
	svm_.release();
	catergory_.clear();
	releaseTrainSet();
}

inline bool Classification::loadModel(const string xmlpath)
{
    bool flag = true;
    try{
		svm_ = Algorithm::load<SVM>(xmlpath);
    }
    catch (std::exception e){
		MESSAGE_COUT("ERROR", e.what());
        flag = false;
    }
    return flag;
}
inline bool Classification::operator<< (string xmlPath){
	return loadModel(xmlPath);
}

inline void Classification::saveModel(const string xmlpath)
{
	svm_->save(xmlpath);
}
inline void Classification::operator>> (string xmlPath){
	saveModel(xmlPath);
}



Mat Classification::extractFeature(Mat Img, Size mrs)
{
    resize(Img, Img, mrs);
    HOGDescriptor *hog = new HOGDescriptor(hog_, cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);
    std::vector<float> descriptors;
    hog->compute(Img, descriptors, Size(1, 1), Size(0, 0));
    return Mat(descriptors).t();
}

int Classification::getCategory(vector<string> &subdirs)
{
	catergory_(subdirs);
	return catergory_.size();
}

int Classification::getDataSet(string dir, int gt)
{
	Mat label;
	vector<string> imgNames = getCurdirFilePath(dir);
	for (auto name : imgNames) {
		Mat img = imread(name);
		Mat descriptors = extractFeature(img, hog_);
		trainMat_.push_back(descriptors);
	}
	label = Mat::ones(imgNames.size(), 1, CV_32SC1) * gt;
	labels_.push_back(label);
	return labels_.rows;
}

int Classification::getDataSet(vector<string> &data_path, double gt)
{
	int nImgNum = static_cast<int>(data_path.size());
    int success = 0;
	MESSAGE_COUT("GET DATA ", gt);
    for (auto &path: data_path){
        Mat src = imread(path);
        if (src.cols && src.rows){
            Mat post = extractFeature(src, hog_);
            trainMat_.push_back(post);
			MESSAGE_COUT("PROCESS " << ++success, findFileName(path));
        }
    }
	Mat tmp = Mat::ones(success, 1, CV_32SC1) * gt;
	labels_.push_back(tmp);
    return success;
}

int Classification::getDataSet(vector<string> &data_path, vector<int> &seq, int num, int k, double gt)
{
	int nImgNum = static_cast<int>(data_path.size());
	int each = nImgNum %k ? nImgNum / k + 1: nImgNum / k;
	int start = num*each;
	int end = std::max((num + 1)*each, nImgNum);
	MESSAGE_COUT("GET DATA ", gt);
	int success = 0;
	for (int i = 0; i < nImgNum; i++)
		if (!(i >= start && i < end)) {
			int p = seq[i];
			Mat src = imread(data_path[p]);
			if (src.cols && src.rows){
				Mat post = extractFeature(src, hog_);
				trainMat_.push_back(post);
				MESSAGE_COUT("PROCESS " << ++success, findFileName(data_path[p]));
			}
		}
	Mat tmp = Mat::ones(success, 1, CV_32SC1) * gt;
	labels_.push_back(tmp);
	return success;
}

Mat Classification::getDataSet(std::vector<std::string> &data_path, std::vector<GroundTruth>& gt, int c)
{
	int nImgNum = static_cast<int>(data_path.size());
    int success = 0;
    Mat data_mat;	//feature matrix
    Mat src;
    string imgname;
    for (int i = 0; i < nImgNum; i++){
        src = imread(data_path[i]);
        if (src.cols && src.rows){
            imgname = FileOperation::findFileName(data_path[i]);
			MESSAGE_COUT("PROCESS", imgname << "\t" << success++);
			Mat post = extractFeature(src, hog_);
            data_mat.push_back(post);
            gt.push_back(GroundTruth(c, imgname));
        }
    }
    return data_mat;
}

int Classification::setSvmParameter(int sv_num, int c_r_type, int kernel, double gamma)
{
    TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS, sv_num, FLT_EPSILON);	//support vectors max is 200
    svm_->setType(c_r_type);
    svm_->setKernel(kernel);
    if (kernel == SVM::RBF)	svm_->setGamma(gamma);
    svm_->setTermCriteria(criteria);
    return 1;
}

int Classification::training(Mat& trainSet, Mat& label, bool save,std::string dir)
{
    setSvmParameter(200, SVM::C_SVC, SVM::LINEAR, 0);
    Ptr<TrainData> traindata = cv::ml::TrainData::create(trainSet, ROW_SAMPLE, label);
    svm_->train(traindata);
    if (save){
		svm_->save(dir + "Classification-MODEL.xml");
    }
    return 1;
}

int Classification::testing(Mat& testSet, float gt)
{
    int error = 0;
    int postnum = testSet.rows;
    Mat res = Mat::zeros(postnum, 1, CV_32FC1);
    svm_->predict(testSet, res);
    for (int i = 0; i < postnum; i++)
        if (res.at<float>(i, 0) != gt)
            error++;
    std::cout << error << "/" << postnum << std::endl;
    return error;
}

int Classification::testing(Mat& testSet, std::vector<GroundTruth> gt)
{
    int error = 0;
    int postnum = testSet.rows;
    Mat res = Mat::zeros(postnum, 1, CV_32FC1);
    svm_->predict(testSet, res);
    for (int i = 0; i < postnum; i++)
        if (res.at<float>(i, 0) != gt[i].label){
			MESSAGE_COUT("ERROR", gt[i].imgname << "\t" << gt[i].label);
            error++;
        }
	MESSAGE_COUT("RESULT", error << "/" << postnum);
    return error;
}

float Classification::predict(Mat& image)
{
    if (!image.rows)	return	-1;
    Mat gray;
    cvtColor(image, gray, CV_BGR2GRAY);
    Mat post = extractFeature(gray, hog_);
    gray.release();
    return svm_->predict(post);
}

float Classification::EndToEnd(string data_path)
{
	//Trainset path
	vector<string> subdirs = getSubdirName(data_path);
	getCategory(subdirs);
	//Get trainset
	for (auto &subdir : subdirs) {
		vector<string> imgpaths = getCurdirFilePath(data_path + subdir + "\\");
		getDataSet(imgpaths, catergory_[subdir]);
	}
	//training model
	training(trainMat_, labels_, true, data_path);
    return 1.0f;
}

// Generate random sequence 0 ~ n-1
vector<int> getRand(int n)
{
	vector<int> random(n);
	for (int i = 0; i < n; i++)
		random[i] = i;
	random_shuffle(random.begin(), random.end());
	return random;
}


void Classification::crossValidation(int k, string dataset, string savedir) 
{
	// get names
	clock_t start, end;
	vector<string> objects = getSubdirName(dataset);
	// extract HOG feature from data
	MESSAGE_COUT("INFO", "Loading data and extract " << hog_ << " HOG feature ...");
	start = clock();
	map<string, int> name2num;
	int cnt = 0;
	for (auto o : objects){
		name2num[o] = cnt;
		getDataSet(dataset + o, cnt++);
	}
	end = clock();
	MESSAGE_COUT("INFO", trainMat_.rows << " samples take " << 1.0*(end - start) / CLOCKS_PER_SEC << "s");
	// get random sequence
	int total = 0;
	vector<int> myseq(trainMat_.rows);
	generate(myseq.begin(), myseq.end(), [&total](){return total++; });
	random_shuffle(myseq.begin(), myseq.end());
	int batch = total / k + (total%k != 0);
	// k-validation
	vector<double> correct;
	for (int epoch = 0; epoch < k; epoch++){
		//training
		Mat traintmp, labeltrain, testtmp, labeltest;
		Mat trainM, labelM;
		trainM = trainMat_.clone();
		labelM = labels_.clone();
		int st = epoch*batch;
		int ed = st + batch;
		for (int r = 0; r < total; r++){
			int p = myseq[r];
			if (r >= st&&r < ed) {
				testtmp.push_back(trainM.row(p));
				labeltest.push_back(labelM.row(p));
			}
			else{
				traintmp.push_back(trainM.row(p));
				labeltrain.push_back(labelM.row(p));
			}
		}
		Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(traintmp, cv::ml::ROW_SAMPLE, labeltrain);
		bool flag = svm_->train(trainData);
		if (savedir.size())
			saveModel(savedir + to_string(epoch + 1) + ".xml");
		// testing
		start = clock();
		int error = 0;
		Mat res = Mat::zeros(testtmp.rows, 1, CV_32FC1);
		svm_->predict(testtmp, res);
		vector<int> category(name2num.size(), 0);
		vector<int> errors(name2num.size(), 0);
		for (int i = 0; i < res.rows; i++){
			int groundtruth = labeltest.at<int>(i, 0);
			float p = res.at<float>(i, 0);
			category[groundtruth]++;
			if (p != groundtruth){
				errors[groundtruth]++;
				error++;
			}
		}
		end = clock();
		// calculate errors
		MESSAGE_COUT(epoch + 1, "");
		for (int i = 0; i < category.size(); i++)
			MESSAGE_COUT(objects[i], errors[i] << "/" << category[i]);
		MESSAGE_COUT("Total", error << "/" << res.rows << ", take " << 1.0*(end - start) / CLOCKS_PER_SEC << "s");
		correct.push_back(1 - 1.0*error / res.rows);
	}
	double avg = accumulate(correct.begin(), correct.end(), 0.0) / correct.size();
	double ex2 = accumulate(correct.begin(), correct.end(), 0.0, [](double part, double x){return part + x*x; }) / correct.size();
	double var = ex2 - avg*avg;
	printf("[%d-Validation]\t%.3lf+/-%.3lf\n", k, avg, sqrt(var));//±
}
