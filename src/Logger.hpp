#pragma once
// Error and crash logging to a text file, because Fusion often shows no message when an extension fails.
//
//  - Soft problems (bad parameters, objects that cannot be found, files that cannot be written) are logged as they
//    happen, a few times each, so a repeating problem cannot flood the file.
//  - Every action, condition and expression leaves a "breadcrumb" in a small ring buffer (cheap: a few numbers, no
//    text). If the extension crashes, the crash report lists the last breadcrumbs: what was running, and with what.
//  - C++ exceptions inside the extension are caught and logged instead of taking Fusion down.
//  - On Windows, crashes (access violations and the like) are written to the log as well: the exception, where it
//    happened (module + offset), the call stack and the breadcrumbs. Fusion's own handling then carries on as usual.
//
// The log goes next to the running application, or into the Temp folder if that is not writable (the action
// "Set log file" and the expression LogPath$ show or change it).

#include <string>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <ctime>
#include <exception>
#include <unordered_map>

namespace FlowLog
{
	// What was running. Keep CrumbNames in the same order.
	enum Crumb : int
	{
		C_None = 0,
		C_CreateGrid, C_CreateGridFit, C_ClearObstacles, C_AddFusion, C_AddFusionLayer, C_AddObjects, C_RemoveObjects,
		C_SetBudget, C_AddAvoid, C_AddAvoidPos, C_RemoveAvoid, C_ClearAvoid, C_MakeAvoid, C_StopAvoid,
		C_DebugGrid, C_DebugBoundary, C_RescanArea, C_RescanAll, C_SetPrecision, C_SetSizePercent, C_SetLogPath,
		C_CondBlocked, C_CondPathObjects, C_CondPathPositions, C_CondReached,
		C_Direction, C_DirectionAt, C_PathDistance, C_PathDistanceAt, C_HasPath, C_HasPathAt, C_IsBlocked, C_GridInfo, C_Text, C_HasReached,
		C_Handle, C_RefreshAgents, C_Construct, C_Destruct,
		C_Count
	};
	static const char* const CrumbNames[C_Count] = {
		"-",
		"Action CreateGrid", "Action CreateGridFitFrame", "Action ClearObstacles", "Action AddFusionObstacles", "Action AddFusionObstaclesAtLayer", "Action AddObjectsAsObstacles", "Action RemoveObjectsAsObstacles",
		"Action SetCellsPerFrame", "Action AddObjectsToAvoid", "Action AddPositionToAvoid", "Action RemoveObjectsToAvoid", "Action ClearAreasToAvoid", "Action MakeObjectsAvoidEachOther", "Action StopObjectsAvoidingEachOther",
		"Action DebugSaveGridImage", "Action DebugSaveBoundaryImage", "Action RescanFusionObstaclesInArea", "Action RescanAllFusionObstacles", "Action SetFusionObstaclePrecision", "Action SetPathfindingSizePercent", "Action SetLogFile",
		"Condition PositionIsBlocked", "Condition PathBetweenObjectsIsClear", "Condition PathBetweenPositionsIsClear", "Condition ObjectReachedDestination",
		"Expression Direction", "Expression DirectionAt", "Expression PathDistance", "Expression PathDistanceAt", "Expression HasPath", "Expression HasPathAt", "Expression IsBlocked", "Expression grid information", "Expression text", "Expression HasReached",
		"Handle (per-frame update)", "RefreshAgents", "Extension created", "Extension destroyed"
	};

	struct CrumbEntry { int code; int a, b; float x, y; unsigned tick; };

	// Formats one breadcrumb without allocating memory (also used while crashing).
	inline int FormatCrumb(char* buf, size_t size, const CrumbEntry& e)
	{
		const char* name = (e.code >= 0 && e.code < C_Count) ? CrumbNames[e.code] : "?";
		return std::snprintf(buf, size, "  tick %u  %s  (a=%d b=%d x=%.1f y=%.1f)\n", e.tick, name, e.a, e.b, e.x, e.y);
	}

	// What the extension looked like when it was last updated (shown in crash reports).
	struct Snapshot { int gridW = 0, gridH = 0; float cell = 0.f; int fields = 0, agents = 0, groups = 0, obstacleObjects = 0, areas = 0; };

	class Logger
	{
	public:
		static Logger& Get() { static Logger l; return l; }

		static constexpr int RingSize = 64;
		static constexpr int MaxRepeats = 5; // how often one warning is written before it is silenced

		void SetPath(const std::string& utf8) { path = utf8; lastDecidedAutomatically = false; PrepareWidePath(); }
		const std::string& Path() { if (path.empty()) ChooseDefaultPath(); return path; }
		const std::string& LastError() const { return lastError; }
		void SetSnapshot(const Snapshot& s) { snap = s; }
		const Snapshot& GetSnapshot() const { return snap; }

		// Breadcrumbs: a handful of numbers per call, nothing else.
		void Mark(int code, int a = 0, int b = 0, float x = 0.f, float y = 0.f)
		{
			ring[next % RingSize] = CrumbEntry{ code, a, b, x, y, NowTick() };
			++next;
		}
		// Latest first, up to max entries.
		int Recent(CrumbEntry* out, int max) const
		{
			int n = 0;
			for (unsigned i = 0; i < (unsigned)RingSize && (int)i < max && i < next; ++i)
				out[n++] = ring[(next - 1 - i) % RingSize];
			return n;
		}

		void Info(const char* fmt, ...)
		{
			va_list ap; va_start(ap, fmt); Write("INFO ", fmt, ap); va_end(ap);
		}
		// key identifies the place in the code: the same key is only written MaxRepeats times.
		void Warn(const char* key, const char* fmt, ...)
		{
			va_list ap; va_start(ap, fmt);
			if (Allowed(key)) Write("WARN ", fmt, ap);
			va_end(ap);
		}
		void Error(const char* key, const char* fmt, ...)
		{
			va_list ap; va_start(ap, fmt);
			char msg[512]; std::vsnprintf(msg, sizeof msg, fmt, ap);
			va_end(ap);
			lastError = msg;
			if (Allowed(key)) WriteLine("ERROR", msg);
		}

		// Raw line (used for session headers and crash reports written by the portable part).
		void WriteLine(const char* level, const char* text)
		{
			const std::string& p = Path();
			std::FILE* f = OpenForAppend(p);
			if (!f) return;
			char stamp[40]; Timestamp(stamp, sizeof stamp);
			std::fprintf(f, "%s [%s] %s\n", stamp, level, text);
			std::fclose(f);
		}

		// Starts a session in the log (and rotates a very large file).
		void BeginSession(const char* what)
		{
			RotateIfLarge();
			char line[256]; std::snprintf(line, sizeof line, "---- %s (build %s %s) ----", what, __DATE__, __TIME__);
			WriteLine("INFO ", line);
			std::snprintf(line, sizeof line, "log file: %s", Path().c_str());
			WriteLine("INFO ", line);
		}

		void ResetRepeatCounts() { repeats.clear(); }

		// ---- crash reporting (Windows only; defined below) ----
		void InstallCrashHandlers();
		void RemoveCrashHandlers();

	private:
		std::string path, lastError;
		bool lastDecidedAutomatically = true;
		Snapshot snap;
		CrumbEntry ring[RingSize] = {};
		unsigned next = 0;
		std::unordered_map<const char*, int> repeats;

		bool Allowed(const char* key)
		{
			int& n = repeats[key];
			++n;
			if (n <= MaxRepeats) return true;
			if (n == MaxRepeats + 1) WriteLine("INFO ", "(the same message will not be repeated any more)");
			return false;
		}
		void Write(const char* level, const char* fmt, va_list ap)
		{
			char msg[512]; std::vsnprintf(msg, sizeof msg, fmt, ap);
			WriteLine(level, msg);
		}

		static unsigned NowTick();
		static void Timestamp(char* buf, size_t size);
		void ChooseDefaultPath();
		void RotateIfLarge();
		static std::FILE* OpenForAppend(const std::string& utf8Path);
		void PrepareWidePath();

		friend struct CrashAccess;
	};

	// Convenience wrappers
	inline void Mark(int code, int a = 0, int b = 0, float x = 0.f, float y = 0.f) { Logger::Get().Mark(code, a, b, x, y); }
	template<class... A> inline void Info(const char* fmt, A... args) { Logger::Get().Info(fmt, args...); }
	template<class... A> inline void Warn(const char* key, const char* fmt, A... args) { Logger::Get().Warn(key, fmt, args...); }
	template<class... A> inline void Error(const char* key, const char* fmt, A... args) { Logger::Get().Error(key, fmt, args...); }
}

// Wrap the body of an action/condition/expression: C++ exceptions are logged and the call returns `fallback`.
#define FP_TRY try {
#define FP_CATCH(tag, fallback) \
	} catch (const std::exception& ex) { FlowLog::Error(tag, "%s: C++ exception: %s", tag, ex.what()); return fallback; } \
	catch (...) { FlowLog::Error(tag, "%s: unknown C++ exception", tag); return fallback; }
#define FP_CATCH_VOID(tag) \
	} catch (const std::exception& ex) { FlowLog::Error(tag, "%s: C++ exception: %s", tag, ex.what()); } \
	catch (...) { FlowLog::Error(tag, "%s: unknown C++ exception", tag); }

// ----------------------------------------------------------------------------------------------------------------
// Implementation of the parts that touch the operating system
// ----------------------------------------------------------------------------------------------------------------
#ifdef _WIN32

#include <cwchar>

namespace FlowLog
{
	namespace detail
	{
		inline wchar_t* WidePath() { static wchar_t p[1040] = {}; return p; }
		inline std::wstring Widen(const std::string& utf8)
		{
			if (utf8.empty()) return std::wstring();
			const int n = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
			std::wstring w((size_t)(n > 0 ? n - 1 : 0), L'\0');
			if (n > 1) MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, &w[0], n);
			return w;
		}
		inline std::string Narrow(const wchar_t* w)
		{
			const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
			std::string s((size_t)(n > 0 ? n - 1 : 0), '\0');
			if (n > 1) WideCharToMultiByte(CP_UTF8, 0, w, -1, &s[0], n, nullptr, nullptr);
			return s;
		}
	}

	inline unsigned Logger::NowTick() { return (unsigned)GetTickCount(); }
	inline void Logger::Timestamp(char* buf, size_t size)
	{
		SYSTEMTIME t; GetLocalTime(&t);
		std::snprintf(buf, size, "%04d-%02d-%02d %02d:%02d:%02d.%03d", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond, t.wMilliseconds);
	}
	inline std::FILE* Logger::OpenForAppend(const std::string& utf8Path) { return _wfopen(detail::Widen(utf8Path).c_str(), L"ab"); }
	inline void Logger::PrepareWidePath()
	{
		const std::wstring w = detail::Widen(path);
		wchar_t* dst = detail::WidePath();
		std::wcsncpy(dst, w.c_str(), 1039); dst[1039] = 0;
	}
	inline void Logger::ChooseDefaultPath()
	{
		// next to the running application, if we can write there; otherwise the Temp folder
		wchar_t exe[MAX_PATH] = {};
		GetModuleFileNameW(nullptr, exe, MAX_PATH);
		std::wstring dir(exe);
		const size_t slash = dir.find_last_of(L"\\/");
		dir = slash == std::wstring::npos ? L"" : dir.substr(0, slash + 1);
		std::wstring candidate = dir + L"FlowFieldPathfinding.log";
		std::FILE* f = _wfopen(candidate.c_str(), L"ab");
		if (f) std::fclose(f);
		else
		{
			wchar_t tmp[MAX_PATH] = {};
			GetTempPathW(MAX_PATH, tmp);
			candidate = std::wstring(tmp) + L"FlowFieldPathfinding.log";
		}
		path = detail::Narrow(candidate.c_str());
		PrepareWidePath();
	}
	inline void Logger::RotateIfLarge()
	{
		const std::wstring w = detail::Widen(Path());
		WIN32_FILE_ATTRIBUTE_DATA info;
		if (GetFileAttributesExW(w.c_str(), GetFileExInfoStandard, &info) && info.nFileSizeLow > (1u << 20) && info.nFileSizeHigh == 0)
			MoveFileExW(w.c_str(), (w + L".old").c_str(), MOVEFILE_REPLACE_EXISTING);
	}

	// ---- crash reporting ----
	struct CrashAccess
	{
		static Logger& L() { return Logger::Get(); }
		static PVOID& Veh() { static PVOID v = nullptr; return v; }
		static LPTOP_LEVEL_EXCEPTION_FILTER& Previous() { static LPTOP_LEVEL_EXCEPTION_FILTER p = nullptr; return p; }
		static int& Installs() { static int n = 0; return n; }
		static bool& Written() { static bool w = false; return w; }

		static const char* CodeName(DWORD code)
		{
			switch (code)
			{
			case EXCEPTION_ACCESS_VIOLATION: return "access violation";
			case EXCEPTION_STACK_OVERFLOW: return "stack overflow";
			case EXCEPTION_INT_DIVIDE_BY_ZERO: return "integer divide by zero";
			case EXCEPTION_ILLEGAL_INSTRUCTION: return "illegal instruction";
			case EXCEPTION_ARRAY_BOUNDS_EXCEEDED: return "array bounds exceeded";
			case EXCEPTION_IN_PAGE_ERROR: return "in-page error";
			case EXCEPTION_PRIV_INSTRUCTION: return "privileged instruction";
			case EXCEPTION_FLT_DIVIDE_BY_ZERO: return "float divide by zero";
			case 0xC0000374: return "heap corruption";
			case 0xC0000409: return "fast-fail (security check)";
			default: return "exception";
			}
		}
		static bool Fatal(DWORD code)
		{
			return code == EXCEPTION_ACCESS_VIOLATION || code == EXCEPTION_STACK_OVERFLOW || code == EXCEPTION_ILLEGAL_INSTRUCTION ||
				code == EXCEPTION_IN_PAGE_ERROR || code == EXCEPTION_INT_DIVIDE_BY_ZERO || code == EXCEPTION_ARRAY_BOUNDS_EXCEEDED ||
				code == 0xC0000374 || code == 0xC0000409;
		}
		// module name and offset of an address, written into buf without allocating memory
		static void Where(void* addr, char* buf, size_t size)
		{
			HMODULE mod = nullptr; wchar_t name[MAX_PATH] = {};
			if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)addr, &mod) && mod)
			{
				GetModuleFileNameW(mod, name, MAX_PATH);
				const wchar_t* base = name; for (const wchar_t* p = name; *p; ++p) if (*p == L'\\' || *p == L'/') base = p + 1;
				char nb[MAX_PATH] = {}; WideCharToMultiByte(CP_UTF8, 0, base, -1, nb, (int)sizeof nb - 1, nullptr, nullptr);
				std::snprintf(buf, size, "%s+0x%llx", nb, (unsigned long long)((const char*)addr - (const char*)mod));
			}
			else std::snprintf(buf, size, "0x%llx (unknown module)", (unsigned long long)(size_t)addr);
		}
		static bool InOurModule(void* addr)
		{
			HMODULE self = nullptr, other = nullptr;
			GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)&CrashAccess::InOurModule, &self);
			GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)addr, &other);
			return self && self == other;
		}

		// Writes the report using only stack buffers and Win32 file calls, as memory may be corrupt.
		static void Report(EXCEPTION_POINTERS* ep, const char* kind)
		{
			if (Written()) return;
			Written() = true;
			const wchar_t* wp = detail::WidePath();
			if (!*wp) L().Path(); // make sure a path has been chosen
			HANDLE h = CreateFileW(detail::WidePath(), FILE_APPEND_DATA, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
			if (h == INVALID_HANDLE_VALUE) return;
			char line[600];
			auto put = [&](const char* s) { DWORD n; WriteFile(h, s, (DWORD)std::strlen(s), &n, nullptr); };
			char stamp[40]; Logger::Timestamp(stamp, sizeof stamp);
			const DWORD code = ep->ExceptionRecord->ExceptionCode;
			char where[300]; Where(ep->ExceptionRecord->ExceptionAddress, where, sizeof where);
			std::snprintf(line, sizeof line, "\r\n%s [CRASH] %s: %s (0x%08lX) at %s\r\n", stamp, kind, CodeName(code), (unsigned long)code, where);
			put(line);
			if (code == EXCEPTION_ACCESS_VIOLATION && ep->ExceptionRecord->NumberParameters >= 2)
			{
				std::snprintf(line, sizeof line, "  while %s address 0x%llx\r\n", ep->ExceptionRecord->ExceptionInformation[0] == 0 ? "reading" : (ep->ExceptionRecord->ExceptionInformation[0] == 1 ? "writing" : "executing"),
					(unsigned long long)ep->ExceptionRecord->ExceptionInformation[1]);
				put(line);
			}
			std::snprintf(line, sizeof line, "  in our extension: %s   build %s %s\r\n", InOurModule(ep->ExceptionRecord->ExceptionAddress) ? "YES (this extension's own code)" : "no (inside Fusion or another module)", __DATE__, __TIME__);
			put(line);
			void* frames[24]; const USHORT nf = RtlCaptureStackBackTrace(0, 24, frames, nullptr);
			put("  call stack:\r\n");
			for (USHORT i = 0; i < nf; ++i) { char w[300]; Where(frames[i], w, sizeof w); std::snprintf(line, sizeof line, "    #%u %s\r\n", (unsigned)i, w); put(line); }
			const Snapshot& s = L().snap;
			std::snprintf(line, sizeof line, "  state: grid %dx%d cells of %.0f px, %d cached fields, %d crowd agents, %d avoid groups, %d objects as obstacles, %d areas to avoid\r\n", s.gridW, s.gridH, s.cell, s.fields, s.agents, s.groups, s.obstacleObjects, s.areas);
			put(line);
			put("  last actions (newest first):\r\n");
			CrumbEntry recent[24]; const int n = L().Recent(recent, 24);
			for (int i = 0; i < n; ++i) { char c[300]; FormatCrumb(c, sizeof c, recent[i]); put(c); }
			CloseHandle(h);
		}

		static LONG WINAPI Unhandled(EXCEPTION_POINTERS* ep)
		{
			Report(ep, "unhandled");
			return Previous() ? Previous()(ep) : EXCEPTION_CONTINUE_SEARCH;
		}
		// First-chance: only serious errors that happen inside this extension's own code.
		static LONG CALLBACK FirstChance(EXCEPTION_POINTERS* ep)
		{
			if (Fatal(ep->ExceptionRecord->ExceptionCode) && InOurModule(ep->ExceptionRecord->ExceptionAddress))
				Report(ep, "error in extension code");
			return EXCEPTION_CONTINUE_SEARCH;
		}
	};

	inline void Logger::InstallCrashHandlers()
	{
		if (CrashAccess::Installs()++ > 0) return;
		CrashAccess::Written() = false;
		CrashAccess::Veh() = AddVectoredExceptionHandler(1, &CrashAccess::FirstChance);
		CrashAccess::Previous() = SetUnhandledExceptionFilter(&CrashAccess::Unhandled);
		(void)Path(); // choose the log file now, so the crash path never has to
	}
	inline void Logger::RemoveCrashHandlers()
	{
		if (--CrashAccess::Installs() > 0) return;
		CrashAccess::Installs() = 0;
		if (CrashAccess::Veh()) { RemoveVectoredExceptionHandler(CrashAccess::Veh()); CrashAccess::Veh() = nullptr; }
		// give Fusion's own filter back; if somebody replaced ours in the meantime, put theirs back
		LPTOP_LEVEL_EXCEPTION_FILTER current = SetUnhandledExceptionFilter(CrashAccess::Previous());
		if (current != &CrashAccess::Unhandled) SetUnhandledExceptionFilter(current);
	}
}

#else // not Windows: the portable parts only (used by the tests)

namespace FlowLog
{
	inline unsigned Logger::NowTick() { return (unsigned)(std::clock() * 1000.0 / CLOCKS_PER_SEC); }
	inline void Logger::Timestamp(char* buf, size_t size)
	{
		std::time_t t = std::time(nullptr); std::tm tmv; localtime_r(&t, &tmv);
		std::strftime(buf, size, "%Y-%m-%d %H:%M:%S", &tmv);
	}
	inline std::FILE* Logger::OpenForAppend(const std::string& utf8Path) { return std::fopen(utf8Path.c_str(), "ab"); }
	inline void Logger::PrepareWidePath() {}
	inline void Logger::ChooseDefaultPath() { path = "FlowFieldPathfinding.log"; }
	inline void Logger::RotateIfLarge() {}
	inline void Logger::InstallCrashHandlers() {}
	inline void Logger::RemoveCrashHandlers() {}
}

#endif
