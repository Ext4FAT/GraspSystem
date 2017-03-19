#include "GraspSystem.hpp"
#include "Classification.hpp"
#include <windows.h>





int main(int argc, char** argv)
{
	//WinExec(("C:\\Users\\IDLER\\Documents\\GitHub\\MechanicalArm\\Release\\MechanicalArm.exe"), SW_MAXIMIZE);

	//GraspSystem data(640, 480, 30);
	//data.dobotCTRL();

	_IDLER_::Classification classifier;
	classifier.crossValidation(10, "..\\dataset\\Body\\");

	return 1;
}



