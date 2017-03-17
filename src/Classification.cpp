#include "Classification.hpp"
#include "Macro.hpp"

Classification::Classification()
{
    svm_ = SVM::create();
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

bool Classification::loadModel(const string model_path)
{
    bool flag = true;
    try{
        svm_ = Algorithm::load<SVM>(model_path);
    }
    catch (std::exception e){
		MESSAGE_COUT("ERROR", e.what());
        flag = false;
    }
    return flag;
}

Mat Classification::extractFeature(Mat Img, Size mrs)
{
    /**
     * @brief Classification::extractFeature
        The story behind 1764
        For example
        window size is 64x64, block size is 16x16 and block setp is 8x8£¬cell size is 8x8,
        the block number window contained is (£¨64-16£©/8+1)*((64-16)/8+1) = 7*7 = 49,
        the cell number each block contained is (16/8)*(16/8) = 4
        every cell can project 9 bin, and each bin related to 9 vector
        so feature_dim  = B x C x N, and caulated result is  1764
        (B is each window's blocks number, C is every block's cell number, n is bin number)
     */
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
    TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS, sv_num, FLT_EPSILON);	//max support vectocr 200
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


void Classification::crossValidation(int k) 
{
	//string dir(".\\object\\");
	string dir("C:\\Users\\IDLER\\Documents\\HUMAN++\\DATASET\\Body\\");
	vector<string> objects = { "human", "background" };
	//vector<string> objects = { "Background", "bottle", "cup", "teapot", "cupnoodle","can" };
	//vector<string> objects = {  "bottle", "cup", "teapot", "cupnoodle", "can" };
	map<string, int> name2num;
	int cnt = 0;
	for (auto o : objects){
		name2num[o] = cnt;
		getDataSet(dir + o, cnt++);
	}
	// Construct TrainData
	TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS, 200, FLT_EPSILON);
	svm_->setType(cv::ml::SVM::C_SVC);
	svm_->setKernel(cv::ml::SVM::LINEAR);
	svm_->setTermCriteria(criteria);
	// get random sequence
	vector<int> myseq;
	for (int i = 0; i < trainMat_.rows; i++)
		myseq.push_back(i);
	random_shuffle(myseq.begin(), myseq.end());
	int total = myseq.size();
	int batch = total / k;
	// batch
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
		svm_->save(".\\classifier\\" + to_string(epoch) + ".xml");
		// testing
		int error = 0;
		Mat res = Mat::zeros(testtmp.rows, 1, CV_32FC1);
		svm_->predict(testtmp, res);
		//cout << labeltest << endl;
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
		for (auto o : objects)
			printf("[%-10s]\t", o.c_str());
		cout << endl;
		for (int i = 0; i < category.size(); i++)
			printf("%4d/%-4d\t", errors[i], category[i]);
		correct.push_back(1 - 1.0*error / res.rows);
		printf("\n[%2d]\t total:%5d/%-5d\n", epoch + 1, error, res.rows);
	}
	double avg = 0.0;
	for (auto c : correct)
		avg += c;
	avg /= correct.size();
	double var = 0;
	for (auto c : correct)
		var += (c - avg)*(c - avg);
	printf("[%d-Validation]\t%.3lf±%.3lf\n", k, 100 * avg, 100 * sqrt(var));
}