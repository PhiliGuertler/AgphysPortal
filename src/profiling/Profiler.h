#pragma once

#include <fstream>
#include <memory>
#include <string>

#include "Time.h"
#include "Timer.h"

namespace Profiling {

	/**
	 *	Data that represents one profiled function call
	 */
	struct ProfileResult {
		std::string name;
		std::chrono::duration<double, std::micro> start;
		std::chrono::microseconds elapsedTime;
		//TimePoint start;
		//TimePoint end;
		uint32_t threadID;
	};

	/**
	 *	Data representing a whole session
	 */
	struct ProfilerSession {
		std::string name;
	};

	/**
	 *	Singleton class creating profiles of the performance of a given program portion.
	 *	It creates a json-output of a session that can be visualized with chromes "chrome://tracing" page.
	 */
	class Profiler {
	public:
		/**
		 *	static getter for the singleton instance of this class
		 */
		static Profiler& getInstance() {
			static Profiler profiler;
			return profiler;
		}

	public:
		/**
		 *	starts a new session that will be written to filePath.
		 *	@param sessionName: The name of the session.
		 *	@param filePath: The path of the resulting file that can be loaded into chrome://tracing
		 *		The default path is "results.json"
		 */
		void beginSession(const std::string& sessionName, const std::string& filePath = "results.json");

		/**
		 *	ends a session and clears up its resources.
		 */
		void endSession();

		/**
		 *	Prepares a session that will only profile the next n frames.
		 *	This prepared session can be started by calling "beginNSession()"
		 *	@param n: The amount of frames to be profiled
		 *	@param sessionName: the name of the session. The dafault name is "quickProfile"
		 *	@param filePath: the path of the resulting file that can be loaded into chrome://tracing.
		 *		The default filePath if "quickProfile.json"
		 */
		void profileNextNFrames(int n, const std::string& sessionName = "quickProfile", const std::string& filePath = "quickProfile.json");

		/**
		 *	starts a session that will profile the next n frames that have previously been set
		 *	by "profileNextNFrames()".
		 */
		void beginNSession();

		/**
		 *	writes a profile result to the file specified in beginSession().
		 *	@param result: The profile result to be written.
		 */
		void writeProfile(const ProfileResult& result);

		/**
		 *	writes the header of a chrome://tracing-compatible json file to the file specified in beginSession().
		 */
		void writeHeader();

		/**
		 *	writes the footer of a chrome://tracing-compatible json file to the file specified in beginSession().
		 */
		void writeFooter();

		inline long long getNProfileEndFrame() { return m_endFrame; }

		inline bool sessionIsRunning() { return m_currentSession != nullptr; }

	private:
		/**
		 *	private constructor to ensure Singleton nature
		 */
		Profiler();

		std::ofstream m_outputStream;
		int m_profileCount;
		std::unique_ptr<ProfilerSession> m_currentSession;

		long long lastStart;

		// members for profileNextNFrames
		long long m_endFrame;
		std::string m_profileSessionName;
		std::string m_profileSessionFilePath;

	};

}


#define ENABLE_PROFILING

// to be defined by a client application
#ifdef ENABLE_PROFILING
	#define PROFILE_BEGIN_SESSION(sessionName, filePath) ::Profiling::Profiler::getInstance().beginSession(sessionName, filePath)
	#define PROFILE_END_SESSION() ::Profiling::Profiler::getInstance().endSession()

	#define PROFILE_SCOPE(name) ::Profiling::ScopedProfilerTimer timer##__LINE__(name)
	#define PROFILE_FUNCTION() PROFILE_SCOPE(__PRETTY_FUNCTION__)
#else
	#define PROFILE_BEGIN_SESSION(sessionName, filePath)
	#define PROFILE_END_SESSION()
	
	#define PROFILE_SCOPE(name)
	#define PROFILE_FUNCTION()
#endif
