// windowsFiles.cpp
// @author Philipp Guertler, 2019

#include "windowsFiles.h"

#ifdef _WINDOWS
//#error This File contains Windows Only Implementations, it should not be included if WINDOWS is not defined!

#include <string>
#include <iostream>
#include <Windows.h>
#include <direct.h>

namespace Platform {

	#define BSG_FILES "BSG-Files (*.bsg)\0*.bsg\0All\0*.*\0"
	#define OBJ_FILES "Obj-Files (*.obj)\0*.obj\0All\0*.*\0"
	#define OFF_FILES "Off-Files (*.off)\0*.off\0All\0*.*\0"

	WindowsPlatform::WindowsPlatform()
		: Platform()
	{}

	WindowsPlatform::~WindowsPlatform() {}

	std::string WindowsPlatform::openFileDialog(const std::string& fileExtensions) {
		OPENFILENAME openFileName;	// common dialog box structure
		char fileName[260];			// buffer for file name
		HWND hwnd = GetConsoleWindow();					// owner window
		HANDLE handle;				// file handle
		
									//Init OPENFILENAME
		ZeroMemory(&openFileName, sizeof(openFileName));
		openFileName.lStructSize = sizeof(openFileName);
		openFileName.hwndOwner = hwnd;
		openFileName.lpstrFile = fileName;
		openFileName.lpstrFile[0] = '\0';
		openFileName.nMaxFile = sizeof(fileName);
		if (fileExtensions == ".bsg") {
			openFileName.lpstrFilter = BSG_FILES;
		}
		else if (fileExtensions == ".obj") {
			openFileName.lpstrFilter = OBJ_FILES;
		}
		else if (fileExtensions == ".off") {
			openFileName.lpstrFilter = OFF_FILES;
		}
		openFileName.nFilterIndex = 1;
		openFileName.lpstrFileTitle = NULL;
		openFileName.nMaxFileTitle = 0;
		openFileName.lpstrInitialDir = NULL;
		openFileName.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

		//display the open dialog box.
		if (GetOpenFileName(&openFileName) == TRUE) {
			std::string ret = std::string(openFileName.lpstrFile);
			return std::string(openFileName.lpstrFile);
		}
		else {
			// operation has been cancelled
			return std::string();
		}
	}

	std::string WindowsPlatform::saveAsFileDialog(const std::string& fileExtensions) {
		OPENFILENAME openFileName;	// common dialog box structure
		char fileName[260];			// buffer for file name
		std::string defaultName = "outputMesh.off";
		strncpy(fileName, defaultName.c_str(), defaultName.size());
		HWND hwnd = GetConsoleWindow();					// owner window
		HANDLE handle;				// file handle

									//Init OPENFILENAME
		ZeroMemory(&openFileName, sizeof(openFileName));
		openFileName.lStructSize = sizeof(openFileName);
		openFileName.hwndOwner = hwnd;
		openFileName.lpstrFile = fileName;
		openFileName.lpstrFile[0] = '\0';
		openFileName.nMaxFile = sizeof(fileName);
		//openFileName.lpstrFilter = "Meshes\0*.bsg;*.obj;*.off\0All\0*.*\0";
		if (fileExtensions == ".bsg") {
			std::cout << "BSG" << std::endl;
			openFileName.lpstrFilter = BSG_FILES;
		}
		else if (fileExtensions == ".obj") {
			std::cout << "OBJ" << std::endl;
			openFileName.lpstrFilter = OBJ_FILES;
		}
		else if (fileExtensions == ".off") {
			std::cout << "OFF" << std::endl;
			openFileName.lpstrFilter = OFF_FILES;
		}
		openFileName.nFilterIndex = 1;
		openFileName.lpstrFileTitle = NULL;
		openFileName.nMaxFileTitle = 0;
		openFileName.lpstrInitialDir = NULL;
		openFileName.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

		//display the save file as dialog box.
		if (GetSaveFileName(&openFileName) == TRUE) {
			return std::string(openFileName.lpstrFile);
		}
		else {
			// operation has been cancelled
			return std::string();
		}
	}

	bool WindowsPlatform::makeDirectory(const std::string& dirName) {
		const int error = _mkdir(dirName.c_str());
		if (error == -1) {
			if (errno != EEXIST) {
				std::cerr << "Error creating directory [" << errno << "]: " << strerror(errno) << std::endl;
				return false;
			}
		}
		return true;
	}

	void WindowsPlatform::splitFilePath(const std::string& fullPath, std::string& outFolderName, std::string& outFileName) {
		auto position = fullPath.rfind("\\");

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