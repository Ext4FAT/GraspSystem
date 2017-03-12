#include "VideoDriver.hpp"
#include "HOG-SVM.hpp"
#include <windows.h>


int main(int argc, char** argv)
{
	//WinExec(("C:\\Users\\IDLER\\Documents\\GitHub\\MechanicalArm\\Release\\MechanicalArm.exe"), SW_MAXIMIZE);

	VideoDriver data(640, 480, 30);
	//data.dobotCTRL();
	data.test();

	return 1;
}



