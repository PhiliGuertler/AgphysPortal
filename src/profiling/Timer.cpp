#include "Timer.h"

#include "Profiler.h"

#include <thread>

namespace Profiling {

	// ######################################################################## //
	// ### Timer ############################################################## //
	// ######################################################################## //

	Timer::Timer()
		: m_startTimePoint()
	{
		// empty
	}

	Timer::~Timer() {
		// empty
	}

	void Timer::start() {
		m_startTimePoint = TimePoint();
	}

	TimeStep Timer::stop() {
		return TimeStep(m_startTimePoint, TimePoint());
	}


	// ######################################################################## //
	// ### ScopedProfilerTimer ################################################ //
	// ######################################################################## //

	ScopedProfilerTimer::ScopedProfilerTimer(const std::string& name) 
		: m_start()
		, m_name(name)
	{
		// remove "__cdecl" from the name if it is part of name
		size_t pos = m_name.find("__cdecl ");
		if (pos != std::string::npos) {
			m_name.erase(pos, 8);
		}
	}

	ScopedProfilerTimer::~ScopedProfilerTimer() {
		Profiler& profiler = Profiler::getInstance();

		// do nothing if there is no active session
		if (!profiler.sessionIsRunning()) return;

		TimePoint end;
		
		// hash the thread id to get a uint
		uint32_t threadID = static_cast<uint32_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
		
		// create the profile result
		auto highResStart = std::chrono::duration<double, std::micro>(m_start.getTimePoint().time_since_epoch());
		auto elapsedTime = std::chrono::time_point_cast<std::chrono::microseconds>(end.getTimePoint()).time_since_epoch() - std::chrono::time_point_cast<std::chrono::microseconds>(m_start.getTimePoint()).time_since_epoch();
		ProfileResult result = {m_name, highResStart, elapsedTime, threadID};
		
		Profiler::getInstance().writeProfile(result);
	}
}
