// linuxFiles.h
// @author Philipp Guertler, 2019

#pragma once

#ifndef _WINDOWS
//#error This File is Linux only! It should not be included in a Windows build

#include "platform.h"

#include <memory>
#include <string>

namespace Platform {
	
	class LinuxPlatform : public Platform {
	public:
		LinuxPlatform();
		~LinuxPlatform();

		std::string openFileDialog(const std::string& fileExtensions) override;
		std::string saveAsFileDialog(const std::string& fileExtensions) override;
		bool makeDirectory(const std::string& dirName) override;
		void splitFilePath(const std::string& fullPath, std::string& outFolderName, std::string& outFileName) override;
	};
}
#endif