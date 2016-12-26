#include "VideoDriver.hpp"
#include "HOG-SVM.hpp"



int main(int argc, char** argv)
{
	string save_dir_path = ".\\savedData\\";
	VideoDriver data(save_dir_path, 640, 480, 30);
	data.dobotCTRL();

	return 1;
}



