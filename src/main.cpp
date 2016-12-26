#include "VideoDriver.hpp"
#include "HOG-SVM.hpp"



int main(int argc, char** argv)
{
	VideoDriver data(640, 480, 30);
	data.dobotCTRL();

	return 1;
}



