// windowsFiles.h
// @author Philipp Guertler, 2019

#pragma once

#ifdef _WINDOWS
//#error This File contains Windows Only Implementations, it should not be included if WINDOWS is not defined!

#include "platform.h"

#include <string>

namespace Platform {

	class WindowsPlatform : public Platform {
	public:
		WindowsPlatform();
		~WindowsPlatform();

		std::string openFileDialog(const std::string& fileExtensions) override;
		std::string saveAsFileDialog(const std::string& fileExtnsions) override;
		bool makeDirectory(const std::string& dirName) override;
		void splitFilePath(const std::string& fullPath, std::string& outFolderName, std::string& outFileName) override;
	};

}
#endif // _WINDOWS