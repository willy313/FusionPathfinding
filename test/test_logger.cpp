#include "../src/Logger.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdlib>
using namespace FlowLog;

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); ++fails; } else printf("ok:   %s\n", msg); } while (0)
static std::string Slurp(const char* p) { std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); return ss.str(); }
static int Count(const std::string& hay, const std::string& needle) { int n = 0; for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + 1)) ++n; return n; }

static int Throws() { FP_TRY throw std::runtime_error("boom"); return 1; FP_CATCH("Test Throws", -7) }
static void ThrowsVoid(int& ran) { FP_TRY ++ran; throw 42; FP_CATCH_VOID("Test ThrowsVoid") }
static float Fine() { FP_TRY return 2.5f; FP_CATCH("Test Fine", -1.f) }

int main()
{
	const char* path = "/tmp/fp_logger_test.log";
	std::remove(path);
	Logger& L = Logger::Get();
	L.SetPath(path);

	L.BeginSession("test session");
	Info("hello %d", 5);
	for (int i = 0; i < 12; ++i) Warn("test.warn", "same warning %d", i);
	Error("test.error", "bad thing %s", "happened");
	std::string log = Slurp(path);
	CHECK(log.find("test session") != std::string::npos && log.find("log file: /tmp/fp_logger_test.log") != std::string::npos, "a session header names the build and the log file");
	CHECK(log.find("[INFO ] hello 5") != std::string::npos, "info lines are written with their text");
	CHECK(Count(log, "same warning") == Logger::MaxRepeats, "the same warning is only written a few times");
	CHECK(Count(log, "will not be repeated any more") == 1, "a note says when a warning is silenced (once)");
	CHECK(log.find("[ERROR] bad thing happened") != std::string::npos && L.LastError() == "bad thing happened", "errors are written and remembered as the last error");

	// breadcrumbs: newest first, and the ring wraps without losing order
	for (int i = 1; i <= 100; ++i) Mark(C_Direction, i, i * 2, (float)i, 0.f);
	Mark(C_HasPath, 7);
	CrumbEntry recent[8]; int n = L.Recent(recent, 8);
	CHECK(n == 8 && recent[0].code == C_HasPath && recent[1].a == 100 && recent[2].a == 99, "recent breadcrumbs come back newest first");
	CrumbEntry all[200]; n = L.Recent(all, 200);
	CHECK(n == Logger::RingSize && all[n - 1].a == 100 - (Logger::RingSize - 2), "the ring keeps the last 64 breadcrumbs and drops older ones");
	char text[200]; FormatCrumb(text, sizeof text, recent[1]);
	CHECK(std::string(text).find("Expression Direction") != std::string::npos && std::string(text).find("a=100") != std::string::npos, "a breadcrumb is printed with the name of what ran and its numbers");
	CHECK(std::string(CrumbNames[C_Count - 1]) == "Extension destroyed", "every breadcrumb code has a name");

	// guards
	int ran = 0;
	CHECK(Throws() == -7, "a C++ exception inside a guarded function is caught and the fallback returned");
	ThrowsVoid(ran);
	CHECK(ran == 1, "a guarded action that throws is caught too");
	CHECK(Fine() == 2.5f, "a guarded function that does not throw returns normally");
	log = Slurp(path);
	CHECK(log.find("Test Throws: C++ exception: boom") != std::string::npos && log.find("unknown C++ exception") != std::string::npos, "caught exceptions are written to the log");

	Snapshot s; s.gridW = 40; s.gridH = 30; s.cell = 16.f; s.fields = 3; L.SetSnapshot(s);
	CHECK(L.GetSnapshot().gridW == 40 && L.GetSnapshot().fields == 3, "the state snapshot is kept for crash reports");

	printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
	return fails ? 1 : 0;
}
