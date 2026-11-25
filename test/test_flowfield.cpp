#include "../src/FlowField.hpp"
#include <cstdio>
#include <chrono>
#include <cstdlib>
using namespace FlowPath;

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); ++fails; } else printf("ok:   %s\n", msg); } while (0)

static CellRect Cells(int cx0, int cy0, int cx1, int cy1) { CellRect r; r.cx0 = cx0; r.cy0 = cy0; r.cx1 = cx1; r.cy1 = cy1; return r; }
static void Block(System& s, int cx0, int cy0, int cx1, int cy1) { s.grid.AddCells(Cells(cx0, cy0, cx1, cy1), +1); }
static float C(int cell, float cs = 16.f) { return cell * cs + cs * 0.5f; } // centre of a cell in pixels

// Walk an agent along the field; returns steps taken or -1 if it never arrived / lost the path.
static int Walk(System& s, float x, float y, float bx, float by, float R, float speed, int maxSteps, bool* hitWall)
{
	*hitWall = false;
	for (int i = 0; i < maxSteps; ++i)
	{
		Sample sm = s.Query(x, y, bx, by, R);
		if (!sm.reachable) return -1;
		float dx = bx - x, dy = by - y;
		if (std::sqrt(dx * dx + dy * dy) < speed) return i;
		x += sm.vec.x * speed; y += sm.vec.y * speed;
		if (s.grid.IsRawBlocked(s.grid.CellX(x), s.grid.CellY(y))) *hitWall = true;
	}
	return -1;
}

int main()
{
	// 1) Wall with a gap: agent must go around.
	{
		System s; s.ResizeGrid(40, 30, 16.f, 0, 0);
		Block(s, 20, 0, 20, 24); Block(s, 20, 26, 20, 29); // wall at x=20, gap at y=25
		bool hit;
		int steps = Walk(s, C(3), C(5), C(35), C(5), 0.f, 3.f, 5000, &hit);
		CHECK(steps > 0, "agent reaches target around wall");
		CHECK(!hit, "agent never enters an obstacle cell");
		CHECK(steps * 3.f > 520.f, "path is longer than straight line (detour taken)");
	}
	// 2) Unreachable target.
	{
		System s; s.ResizeGrid(20, 20, 16.f, 0, 0);
		Block(s, 10, 0, 10, 19); // full wall
		Sample sm = s.Query(C(3), C(10), C(15), C(10), 0.f);
		CHECK(!sm.reachable, "unreachable reported when a wall fully separates");
		CHECK(FormatDirection(sm, 32) == -1.f, "direction is -1 when there is no path");
	}
	// 3) No corner cutting: diagonal squeeze between touching blocks is not allowed.
	{
		System s; s.ResizeGrid(5, 5, 16.f, 0, 0);
		Block(s, 2, 1, 2, 1); Block(s, 1, 2, 1, 2); // diagonal pair
		Sample sm = s.Query(C(1), C(1), C(3), C(3), 0.f);
		CHECK(sm.reachable && sm.distance > 3.9f * 16.f, "diagonal squeeze between touching blocks is rejected");
	}
	// 4) Direction angle convention.
	{
		Vec2 right{ 1, 0 }, up{ 0, -1 }, left{ -1, 0 }, down{ 0, 1 };
		CHECK(std::fabs(VecToFusionAngle(right) - 0.f) < 0.01f, "angle right = 0");
		CHECK(std::fabs(VecToFusionAngle(up) - 90.f) < 0.01f, "angle up(screen) = 90");
		CHECK(std::fabs(VecToFusionAngle(left) - 180.f) < 0.01f, "angle left = 180");
		CHECK(std::fabs(VecToFusionAngle(down) - 270.f) < 0.01f, "angle down(screen) = 270");
	}
	// 5) Continuity + accuracy in open space.
	{
		System s; s.ResizeGrid(60, 60, 16.f, 0, 0);
		float tx = C(50), ty = C(20);
		float prev = -1, maxJump = 0, maxErr = 0, maxDistErr = 0;
		for (float x = 100; x < 400; x += 1.f)
		{
			Sample sm = s.Query(x, 700.f, tx, ty, 0.f);
			float a = VecToFusionAngle(sm.vec);
			float trueA = VecToFusionAngle({ tx - x, ty - 700.f });
			float err = std::fabs(a - trueA); if (err > 180) err = 360 - err;
			maxErr = (std::max)(maxErr, err);
			float trueD = std::sqrt((tx - x) * (tx - x) + (ty - 700.f) * (ty - 700.f));
			maxDistErr = (std::max)(maxDistErr, std::fabs(sm.distance - trueD) / trueD);
			if (prev >= 0) maxJump = (std::max)(maxJump, std::fabs(a - prev));
			prev = a;
		}
		printf("      max jump between 1px steps: %.3f deg; max error vs true bearing: %.2f deg; max distance error: %.1f%%\n", maxJump, maxErr, maxDistErr * 100.f);
		CHECK(maxJump < 3.f, "angle changes smoothly between neighbouring pixels");
		CHECK(maxErr < 6.f, "open-space direction within 6 degrees of the true bearing");
		CHECK(maxDistErr < 0.06f, "open-space distance within 6% of Euclidean");
	}
	// 6) Incremental build (per-frame budget) gives the same answer as a full build.
	{
		System a, b;
		for (System* s : { &a, &b }) { s->ResizeGrid(100, 100, 8.f, 0, 0); Block(*s, 50, 10, 50, 89); }
		a.cellsPerFrame = 0; b.cellsPerFrame = 500;
		Field* fa = a.Acquire(90 * 8.f, 50 * 8.f, 0.f);
		Field* fb = b.Acquire(90 * 8.f, 50 * 8.f, 0.f);
		CHECK(fa->ready && !fb->ready, "unlimited budget builds at once; limited budget builds over several frames");
		int ticks = 0; while (!fb->ready && ++ticks < 100000) b.Update();
		printf("      incremental build took %d ticks at 500 expansions/tick\n", ticks);
		CHECK(fa->dist == fb->dist, "incremental build identical to full build");
	}
	// 7) Obstacle change triggers a rebuild.
	{
		System s; s.ResizeGrid(30, 30, 16.f, 0, 0);
		float before = s.Query(C(3), C(15), C(25), C(15), 0.f).distance;
		Block(s, 15, 0, 15, 28);
		float after = s.Query(C(3), C(15), C(25), C(15), 0.f).distance;
		CHECK(after > before + 100.f, "answers route around an obstacle added later (longer distance)");
	}
	// 8) Padding: units keep away from walls and the path stays smooth.
	{
		auto run = [](float R, float* minClear, bool* arrived, float* maxTurnDeg) {
			System s; s.ResizeGrid(40, 30, 16.f, 0, 0);
			Block(s, 20, 0, 20, 19); Block(s, 20, 28, 20, 29); // wall with an 8-cell gap
			float bx = C(35), by = C(5);
			float x = C(3), y = C(5), px = 0, py = 0; *minClear = 1e9f; *arrived = false; *maxTurnDeg = 0;
			for (int steps = 0; steps < 8000; ++steps)
			{
				Sample sm = s.Query(x, y, bx, by, R);
				if (!sm.reachable) break;
				float dx = bx - x, dy = by - y;
				if (std::sqrt(dx * dx + dy * dy) < 3.f) { *arrived = true; break; }
				float nx = x + sm.vec.x * 2.f, ny = y + sm.vec.y * 2.f;
				if (steps > 0) { float a1 = VecToFusionAngle({ x - px, y - py }), a2 = VecToFusionAngle({ nx - x, ny - y }); float d = std::fabs(a2 - a1); if (d > 180) d = 360 - d; *maxTurnDeg = (std::max)(*maxTurnDeg, d); }
				px = x; py = y; x = nx; y = ny;
				int cx = s.grid.CellX(x), cy = s.grid.CellY(y);
				for (int j = -4; j <= 4; ++j) for (int i = -4; i <= 4; ++i)
				{
					int ox = cx + i, oy = cy + j;
					if (!s.grid.InBounds(ox, oy) || !s.grid.raw[s.grid.Index(ox, oy)]) continue;
					float rx0 = ox * 16.f, ry0 = oy * 16.f;
					float ddx = (std::max)((std::max)(rx0 - x, x - (rx0 + 16.f)), 0.f), ddy = (std::max)((std::max)(ry0 - y, y - (ry0 + 16.f)), 0.f);
					*minClear = (std::min)(*minClear, std::sqrt(ddx * ddx + ddy * ddy));
				}
			}
		};
		float c0, c1, t0, t1; bool a0, a1;
		run(0.f, &c0, &a0, &t0); run(24.f, &c1, &a1, &t1);
		printf("      closest approach to a wall: radius 0 -> %.1f px, radius 24 -> %.1f px; max turn per 2px step %.1f / %.1f deg\n", c0, c1, t0, t1);
		CHECK(a0 && a1, "agent reaches target with and without padding");
		CHECK(c1 >= 16.f, "a mover of radius 24 stays at least 16 px from walls");
		CHECK(c1 > c0 + 10.f, "radius clearly increases clearance");
		CHECK(t1 < 20.f, "padded path is smooth (no sharp turns/vibration)");
	}
	// 9) Different sizes: small movers fit through a gap that large ones cannot use; fields are shared per size class.
	{
		System s; s.ResizeGrid(40, 30, 16.f, 0, 0);
		Block(s, 20, 0, 20, 23); Block(s, 20, 27, 20, 29); // gap rows 24..26 = 48 px
		float bx = C(35), by = C(5);
		CHECK(s.Query(C(3), C(5), bx, by, 8.f).reachable, "small mover (radius 8) fits through a 48 px gap");
		CHECK(!s.Query(C(3), C(5), bx, by, 30.f).reachable, "large mover (radius 30) does not fit through a 48 px gap");
		CHECK(s.fields.size() == 2, "two size classes -> two fields");
		s.Query(C(4), C(6), bx, by, 8.f); s.Query(C(5), C(7), bx, by, 7.f);
		CHECK(s.fields.size() == 2, "movers of the same size class share one field");
	}
	// 10) Cache: shared by everyone heading to the same cell, bounded, and idle fields are dropped.
	{
		System s; s.ResizeGrid(40, 30, 16.f, 0, 0);
		for (int i = 0; i < 10; ++i) s.Query(C(2 + i), C(3), C(35), C(20), 0.f);
		CHECK(s.fields.size() == 1, "ten movers heading to the same target cell share one field");
		s.maxFields = 4;
		for (int i = 0; i < 10; ++i) s.Query(C(1), C(1), C(10 + i), C(20), 0.f);
		CHECK(s.fields.size() <= 4, "cache never grows past its limit (least recently used is dropped)");
		s.idleTicks = 5; for (int i = 0; i < 10; ++i) s.Update();
		CHECK(s.fields.empty(), "fields nobody asks for any more are dropped");
	}
	// 11) Obstacles are counted: removing one object only frees cells nothing else covers.
	{
		System s; s.ResizeGrid(20, 20, 16.f, 0, 0);
		CellRect a = Cells(5, 5, 8, 5), b = Cells(7, 5, 10, 5);
		s.grid.AddCells(a, +1); s.grid.AddCells(b, +1); s.grid.Refresh();
		s.grid.AddCells(a, -1); s.grid.Refresh();
		CHECK(!s.grid.IsRawBlocked(5, 5) && !s.grid.IsRawBlocked(6, 5), "cells only covered by the removed object are freed");
		CHECK(s.grid.IsRawBlocked(7, 5) && s.grid.IsRawBlocked(8, 5), "cells still covered by another object stay blocked");
		s.grid.AddCells(b, -1); s.grid.Refresh();
		CHECK(!s.grid.IsRawBlocked(8, 5) && !s.grid.IsRawBlocked(10, 5), "removing the last object frees everything");
		s.grid.MarkImported(2, 2); s.grid.AddCells(Cells(2, 2, 2, 2), +1); s.grid.AddCells(Cells(2, 2, 2, 2), -1); s.grid.Refresh();
		CHECK(s.grid.IsRawBlocked(2, 2), "imported Fusion obstacles stay when an object over them is removed");
		s.grid.ClearAll(); s.grid.Refresh();
		CHECK(!s.grid.IsRawBlocked(2, 2), "clear all obstacles removes imported and added ones");
	}
	// 12) A unit inside a wall zone is pushed OUT of it, not left to vibrate.
	{
		System s; s.ResizeGrid(40, 30, 16.f, 0, 0);
		s.grid.AddCells(s.grid.RectToCells(288, 128, 352, 352), +1);
		float tx = C(35), ty = C(15), R = 24.f;
		Sample inCore = s.Query(283.f, 240.f, tx, ty, R); // 5 px left of the block: inside the hard core
		CHECK(inCore.reachable && inCore.vec.x < -0.3f, "unit in the hard core next to a wall is pushed away from it");
		Sample deep = s.Query(320.f, 240.f, tx, ty, R);   // centre of the obstacle itself
		CHECK(deep.reachable && (std::fabs(deep.vec.x) > 0.1f || std::fabs(deep.vec.y) > 0.1f), "unit inside the obstacle gets an exit direction");
		float x = 320.f, y = 240.f; bool out = false;
		for (int i = 0; i < 400; ++i)
		{
			Sample sm = s.Query(x, y, tx, ty, R); x += sm.vec.x * 2.f; y += sm.vec.y * 2.f;
			if (!s.grid.IsBlockedFor(s.grid.CellX(x), s.grid.CellY(y), R)) { out = true; break; }
		}
		CHECK(out, "following the exit direction from inside the obstacle reaches free space");
	}
	// 13) Target standing inside a wall: the agent goes to the wall's edge and stops, no pushing/vibrating.
	{
		System s; s.ResizeGrid(40, 30, 16.f, 0, 0);
		s.grid.AddCells(s.grid.RectToCells(288, 128, 352, 352), +1);
		float x = C(5), y = C(15); bool settled = false;
		for (int i = 0; i < 4000; ++i)
		{
			Sample sm = s.Query(x, y, 320.f, 240.f, 16.f);
			float nx = x + sm.vec.x * 2.f, ny = y + sm.vec.y * 2.f;
			float mv = std::sqrt((nx - x) * (nx - x) + (ny - y) * (ny - y));
			x = nx; y = ny;
			if (mv < 0.01f) { settled = true; break; }
		}
		printf("      agent stopped at (%.0f, %.0f); raw block spans x 288..352\n", x, y);
		CHECK(settled, "agent comes to rest when the target is unreachable inside a wall");
		CHECK(!s.grid.IsRawBlocked(s.grid.CellX(x), s.grid.CellY(y)), "agent rests outside the raw obstacle");
	}
	// 14) Direction formats.
	{
		auto mk = [](float deg) { Sample s; s.reachable = true; float r = deg * 3.14159265f / 180.f; s.vec = { std::cos(r), -std::sin(r) }; return s; };
		Sample a100 = mk(100.f), a355 = mk(355.f);
		CHECK(std::fabs(FormatDirection(a100, 360) - 100.f) < 0.01f, "360 format returns degrees");
		CHECK(FormatDirection(a100, 4) == 8.f, "4 directions: 100 deg -> 8 (up) on the 0-31 scale");
		CHECK(FormatDirection(a100, 8) == 8.f, "8 directions: 100 deg -> 8");
		CHECK(FormatDirection(a100, 16) == 8.f, "16 directions: 100 deg -> 8");
		CHECK(FormatDirection(a100, 32) == 9.f, "32 directions: 100 deg -> 9");
		CHECK(FormatDirection(a355, 4) == 0.f, "4 directions wraps 355 deg back to 0 (right)");
		CHECK(FormatDirection(Sample(), 32) == -1.f, "no sample -> -1");
	}
	// 15) Performance: 512x512 grid, many movers asking for the same target.
	{
		System s; s.ResizeGrid(512, 512, 8.f, 0, 0);
		srand(1); for (int i = 0; i < 40000; ++i) { int x = rand() % 512, y = rand() % 512; s.grid.AddCells(Cells(x, y, x, y), +1); }
		auto t0 = std::chrono::steady_clock::now();
		s.Query(0, 0, 256 * 8.f, 256 * 8.f, 16.f); // first query builds the field
		auto t1 = std::chrono::steady_clock::now();
		double acc = 0;
		for (int i = 0; i < 100000; ++i) acc += s.Query((float)(rand() % 4096), (float)(rand() % 4096), 256 * 8.f, 256 * 8.f, 16.f).vec.x;
		auto t2 = std::chrono::steady_clock::now();
		printf("      512x512 first query (build): %.2f ms; 100k further queries: %.2f ms (%.0f ns each) [%g]\n",
			std::chrono::duration<double, std::milli>(t1 - t0).count(), std::chrono::duration<double, std::milli>(t2 - t1).count(),
			std::chrono::duration<double, std::nano>(t2 - t1).count() / 100000.0, acc);
		CHECK(true, "performance measured");
	}
	printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
	return fails ? 1 : 0;
}
