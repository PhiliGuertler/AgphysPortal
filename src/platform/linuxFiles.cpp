// linuxFiles.cpp
// @author Philipp Guertler, 2019

#include "linuxFiles.h"

#ifndef _WINDOWS
//#error This File is Linux only! It should not be included in a Windows build

#include <string>
#include <iostream>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <sys/stat.h>

namespace Platform {

	LinuxPlatform::LinuxPlatform() : Platform() {}

	LinuxPlatform::~LinuxPlatform() {}

	std::string LinuxPlatform::openFileDialog(const std::string& fileExtensions) {
		char file[1024];
		FILE *f = popen("zenity --file-selection", "r");

		auto err = fgets(file, 1024, f);
		if(err == NULL) {
			std::cerr << "File could not be opened!" << std::endl;
		}

		int result = pclose(f);

		std::string filePath = std::string(file);
		std::cout << "Result: " << result << ", FilePath: " << filePath << std::endl;
		if (result != 0) {
			// operation cancelled
			filePath = std::string();
		} else {
			filePath = filePath.substr(0, filePath.size() - 1);
		}

		return filePath;
	}

	std::string LinuxPlatform::saveAsFileDialog(const std::string& fileExtensions) {
		char file[1024];
		FILE *f = popen("zenity --file-selection --save", "r");

		auto err = fgets(file, 1024, f);
		if(err == NULL) {
			std::cerr << "File could not be opened!" << std::endl;
		}

		std::string filePath = std::string(file);
		if (filePath == "-") {
			// operation cancelled
			filePath = std::string();
		} else {
			filePath = filePath.substr(0, filePath.size() - 1);
		}

		std::cout << filePath << std::endl;

		return filePath;
	}

	bool LinuxPlatform::makeDirectory(const std::string& dirName) {
		const int error = mkdir(dirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if (error == -1) {
			if (errno != EEXIST) {
				std::cerr << "Error creating directory \"" << dirName << "\" [" << errno << "]: " << strerror(errno) << std::endl;
				return false;
			}
		}
		return true;
	}

	void LinuxPlatform::splitFilePath(const std::string& fullPath, std::string& outFolderName, std::string& outFileName) {
		auto position = fullPath.rfind("/");

		if (position >= fullPath.size()) {
			outFileName = "";
			outFileName = fullPath;
		}
		else {
			outFolderName = fullPath.substr(0, position + 1);
			outFileName = fullPath.substr(position + 1, fullPath.size() - position);
		}
	}
}
#endif
