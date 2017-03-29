#include "Directory.hpp"
#include "Common.hpp"
#include <windows.h>

//Get current dir filepath
std::vector<std::string> Directory::getCurdirFilePath(std::string dirPath)
{
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	std::vector<std::string> files;
	hFind = FindFirstFile((dirPath + "\\*").c_str(), &ffd);
	do
	{
		if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			files.push_back(dirPath + "\\" + ffd.cFileName);
	} while (FindNextFile(hFind, &ffd) != 0);
	FindClose(hFind);
	return files;
}

//Get current dir filename
std::vector<std::string> Directory::getCurdirFileName(std::string dirPath)
{
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	std::vector<std::string> files;
	hFind = FindFirstFile((dirPath + "\\*").c_str(), &ffd);
	do
	{
		if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			files.push_back(ffd.cFileName);
	} while (FindNextFile(hFind, &ffd) != 0);
	FindClose(hFind);
	return files;
}

//Get subdir 
std::vector<std::string> Directory::getSubdirName(std::string dirPath)
{
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	std::vector<std::string> files;
	hFind = FindFirstFile((dirPath + "\\*").c_str(), &ffd);
	do
	{
		if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			files.push_back(ffd.cFileName);
			if (files.back() == "." || files.back() == "..")
				files.pop_back();
		} 
	} while (FindNextFile(hFind, &ffd) != 0);
	FindClose(hFind);
	return files;
}

//Get file name from file path;
std::string Directory::findFileName(std::string path)
{
	int pos = path.find_last_of('\\');
	return path.substr(pos + 1);
}