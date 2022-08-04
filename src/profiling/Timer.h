#pragma once

#include <string>

#include "Time.h"

namespace Profiling {

	/**
	 *	A timer that can be started and stopped
	 */
	class Timer {
	public:
		/**
		 *	constructor
		 */
		Timer();
		/**
		 *	destructor
		 */
		~Timer();

		/**
		 *	starts the timer
		 */
		void start();
		/**
		 *	returns the time passed since the last call of start()
		 */
		TimeStep stop();

	protected:
		// stores the time point of the last call of start()
		TimePoint m_startTimePoint;
	};

	/**
	 *	A timer class to be used with the profiler defined in "Profiler.h".
	 *	It is a variant of a scoped timer, which means it starts a profile
	 *	measurement on creation and sends its results to the Profiler on
	 *	destruction.
	 */
	class ScopedProfilerTimer {
	public:
		/**
		 *	constructor
		 *	@param name: name of the profiled section, e.g. the function name
		 */
		ScopedProfilerTimer(const std::string& name);

		/**
		 *	destructor
		 *	sends the profiling result to the Profiler defined in "Profiler.h"
		 */
		~ScopedProfilerTimer();

	private:
		// time of creation
		TimePoint m_start;
		// name of the timer.
		std::string m_name;
	};
}
