#include "GraspSystem.hpp"
#include "HOG-SVM.hpp"
#include <windows.h>


int main(int argc, char** argv)
{
	//WinExec(("C:\\Users\\IDLER\\Documents\\GitHub\\MechanicalArm\\Release\\MechanicalArm.exe"), SW_MAXIMIZE);

	GraspSystem data(640, 480, 30);
	data.dobotCTRL();

	return 1;
}



