// Measures where the time goes in realistic setups. Not a pass/fail test.
#include "../src/FlowField.hpp"
#include <cstdio>
#include <chrono>
#include <cstdlib>
using namespace FlowPath;
using Clock = std::chrono::steady_clock;
static double Ms(Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double, std::milli>(b - a).count(); }
static CellRect Cells(int a, int b, int c, int d) { CellRect r; r.cx0 = a; r.cy0 = b; r.cx1 = c; r.cy1 = d; return r; }

static void Scatter(System& s, int percent, unsigned seed)
{
	srand(seed);
	const int n = (s.grid.width * s.grid.height * percent / 100) / 9; // 3x3 blocks, like walls and crates
	for (int i = 0; i < n; ++i) { int x = rand() % (s.grid.width - 3), y = rand() % (s.grid.height - 3); s.grid.AddCells(Cells(x, y, x + 2, y + 2), +1); }
	s.grid.Refresh();
}

int main()
{
	printf("== A. Building one field, 8%% obstacles, radius 16 ==\n");
	for (int dim : { 40, 80, 160, 320, 512 })
	{
		int h = dim * 3 / 4; System s; s.ResizeGrid(dim, h, 16.f, 0, 0); Scatter(s, 8, 1);
		const int reps = dim <= 160 ? 50 : 5; auto t0 = Clock::now();
		for (int i = 0; i < reps; ++i) { s.fields.clear(); s.Acquire(dim * 8.f, h * 8.f, 16.f); }
		printf("  %3d x %3d cells (%6d): %.3f ms\n", dim, h, dim * h, Ms(t0, Clock::now()) / reps);
	}

	printf("== B. A frame where the player moved one cell: 3 enemy sizes => 3 fields rebuilt, 300 enemies ask ==\n");
	for (int dim : { 40, 80, 160, 320 })
	{
		int h = dim * 3 / 4; System s; s.ResizeGrid(dim, h, 16.f, 0, 0); Scatter(s, 8, 2);
		struct E { float x, y, r; }; std::vector<E> en;
		for (int i = 0; i < 300; ++i) en.push_back({ (float)(rand() % (dim * 16)), (float)(rand() % (h * 16)), (i % 3) * 8.f + 8.f });
		const int frames = dim <= 80 ? 200 : 40; double acc = 0; auto t0 = Clock::now();
		for (int f = 0; f < frames; ++f)
		{
			float px = 100.f + (f % 40) * 16.f, py = 120.f + (f / 40 % 10) * 16.f; // target walks through the map
			s.Update();
			for (auto& e : en) acc += s.Query(e.x, e.y, px, py, e.r).vec.x;
		}
		printf("  %3d x %3d cells: %.3f ms per frame [%g]\n", dim, h, Ms(t0, Clock::now()) / frames, acc);
	}

	printf("== C. Lookups only (target still, fields cached) ==\n");
	{
		System s; s.ResizeGrid(80, 60, 16.f, 0, 0); Scatter(s, 8, 3); s.Query(0, 0, 640.f, 400.f, 16.f);
		const int N = 1000000; double acc = 0; srand(5); auto t0 = Clock::now();
		for (int i = 0; i < N; ++i) acc += s.Query((float)(rand() % 1280), (float)(rand() % 960), 640.f, 400.f, 16.f).vec.x;
		double total = Ms(t0, Clock::now());
		Field* f = s.Acquire(640.f, 400.f, 16.f); srand(5); auto t1 = Clock::now();
		for (int i = 0; i < N; ++i) acc += SampleField(s.grid, *f, (float)(rand() % 1280), (float)(rand() % 960), 640.f, 400.f).vec.x;
		double sampleOnly = Ms(t1, Clock::now());
		printf("  Query (lookup + sample): %.0f ns; sampling alone: %.0f ns [%g]\n", total * 1e6 / N, sampleOnly * 1e6 / N, acc);
	}

	printf("== D. Crowd avoidance: rebuild + one separation query per unit, per frame ==\n");
	for (int n : { 100, 500, 2000 })
	{
		Crowd c; srand(7); std::vector<Agent> ag;
		for (int i = 0; i < n; ++i) { Agent a; a.id = i + 1; a.x = (float)(rand() % 2000); a.y = (float)(rand() % 1500); a.r = 12.f; ag.push_back(a); }
		const int frames = 100; double acc = 0; auto t0 = Clock::now();
		for (int f = 0; f < frames; ++f)
		{
			c.Clear(); for (auto& a : ag) c.Add(a); c.Rebuild();
			for (auto& a : ag) acc += c.Separation(*c.Find(a.id)).x;
		}
		printf("  %4d units: %.3f ms per frame [%g]\n", n, Ms(t0, Clock::now()) / frames, acc);
	}

	printf("== E. Moving one obstacle per frame (Refresh + one rebuilt field) ==\n");
	for (int dim : { 80, 160, 320 })
	{
		int h = dim * 3 / 4; System s; s.ResizeGrid(dim, h, 16.f, 0, 0); Scatter(s, 8, 4);
		CellRect prev = Cells(5, 5, 7, 7); s.grid.AddCells(prev, +1);
		const int frames = 40; auto t0 = Clock::now();
		for (int f = 0; f < frames; ++f)
		{
			s.grid.AddCells(prev, -1); prev = Cells(5 + f % 20, 5, 7 + f % 20, 7); s.grid.AddCells(prev, +1);
			s.Query(50.f, 50.f, dim * 8.f, h * 8.f, 16.f);
		}
		printf("  %3d x %3d cells: %.3f ms per frame\n", dim, h, Ms(t0, Clock::now()) / frames);
	}

	printf("== F. Every enemy chases its own target (cache holds up to %d fields on this grid) ==\n", (int)[]{ System s; s.ResizeGrid(80, 60, 16.f, 0, 0); return s.FieldLimit(); }());
	for (int n : { 8, 16, 40 })
	{
		System s; s.ResizeGrid(80, 60, 16.f, 0, 0); Scatter(s, 8, 6);
		const int frames = 50; double acc = 0; srand(9); auto t0 = Clock::now();
		for (int f = 0; f < frames; ++f)
		{
			s.Update();
			for (int i = 0; i < n; ++i) acc += s.Query(100.f + i * 7, 100.f, 100.f + i * 29 % 1100, 200.f + i * 13 % 600, 16.f).vec.x;
		}
		printf("  %2d enemies with %2d different targets: %.3f ms per frame [%g]\n", n, n, Ms(t0, Clock::now()) / frames, acc);
	}

	printf("== G. \"Has reached\" for a whole crowd around one target: one Query per unit + clump search, once per tick ==\n");
	for (int n : { 100, 500, 2000 })
	{
		System s; s.ResizeGrid(80, 60, 16.f, 0, 0); Scatter(s, 8, 11);
		const float tx = 640.f, ty = 480.f; srand(13);
		// units gathered in a dense blob around the target, like a real crowd
		for (int i = 0; i < n; ++i)
		{
			Agent a; a.id = i + 1; a.r = 12.f; a.pathR = 12.f;
			float ang = (float)(rand() % 6283) / 1000.f, rad = 14.f * std::sqrt((float)(rand() % 1000) / 1000.f * (float)n * 0.6f);
			a.x = tx + std::cos(ang) * rad; a.y = ty + std::sin(ang) * rad; s.crowd.Add(a);
		}
		s.crowd.Rebuild();
		const int frames = 50; size_t reachedCount = 0; auto t0 = Clock::now();
		for (int f = 0; f < frames; ++f)
		{
			std::vector<char> core(s.crowd.agents.size(), 0);
			for (size_t i = 0; i < core.size(); ++i) { const Agent& a = s.crowd.agents[i]; Sample q = s.Query(a.x, a.y, tx, ty, a.pathR); core[i] = q.reachable && q.distance <= 30.f; }
			std::vector<char> r = s.crowd.ReachedByClump(core);
			reachedCount = 0; for (char c : r) reachedCount += c;
		}
		printf("  %4d units: %.3f ms per tick (%zu of them count as arrived)\n", n, Ms(t0, Clock::now()) / frames, reachedCount);
	}
}
