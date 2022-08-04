// platform.h
// @author Philipp Guertler, 2019

#pragma once

#include <string>
#include <memory>

namespace Platform {
	class Platform {
	public:
		static Platform& get();

	public:
		Platform();
		virtual ~Platform();

		// fileExtensions must be in this format: "*.a;*.b"
		virtual std::string openFileDialog(const std::string& fileExtensions) = 0;
		virtual std::string saveAsFileDialog(const std::string& fileExtensions) = 0;
		virtual bool makeDirectory(const std::string& dirName) = 0;

		virtual void splitFilePath(const std::string& fullPath, std::string& outFolderName, std::string& outFileName) = 0;

	protected:
		static std::unique_ptr<Platform> m_platform;
	};
}

#ifdef _WINDOWS
	#include "windowsFiles.h"
#else
	#include "linuxFiles.h"
#endif