// platform.cpp
// @author Philipp Guertler, 2019

#include "platform.h"

namespace Platform {

	std::unique_ptr<Platform> Platform::m_platform(nullptr);

	Platform::Platform() {}

	Platform::~Platform() {}

	Platform& Platform::get() {
		if (m_platform == nullptr) {
#ifdef _WINDOWS
			m_platform = std::make_unique<WindowsPlatform>();
#else
			m_platform = std::make_unique<LinuxPlatform>();
#endif
		}
		return *m_platform;
	}

}