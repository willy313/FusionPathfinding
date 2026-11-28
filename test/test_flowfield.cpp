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
	// 16) Areas to avoid (soft obstacles): paths go around them when there is room, but they never block.
	{
		auto closest = [](float zonePad, bool* arrived) {
			System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
			const float zx = C(30), zy = C(20);
			if (zonePad > 0.f) s.grid.AddPointZone(zx, zy, zonePad);
			float x = C(3), y = C(20), bx = C(55), by = C(20), best = 1e9f; *arrived = false;
			for (int i = 0; i < 4000; ++i)
			{
				Sample sm = s.Query(x, y, bx, by, 0.f);
				if (!sm.reachable) break;
				if (std::sqrt((bx - x) * (bx - x) + (by - y) * (by - y)) < 3.f) { *arrived = true; break; }
				x += sm.vec.x * 2.f; y += sm.vec.y * 2.f;
				best = (std::min)(best, std::sqrt((x - zx) * (x - zx) + (y - zy) * (y - zy)));
			}
			return best;
		};
		bool a0, a1; float c0 = closest(0.f, &a0), c1 = closest(80.f, &a1);
		printf("      closest approach to the area's centre: without the area %.1f px, with an 80 px area %.1f px\n", c0, c1);
		CHECK(a0 && a1, "agent reaches the target with and without an area to avoid");
		CHECK(c1 > c0 + 30.f, "the path bends away from an area to avoid when there is room");
	}
	{
		System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
		Block(s, 10, 0, 40, 19); Block(s, 10, 23, 40, 39); // a 3-cell corridor (rows 20..22) from x=10 to 40
		s.grid.AddPointZone(C(25), C(21), 100.f);          // an area to avoid filling the whole corridor
		bool hit; int steps = Walk(s, C(3), C(21), C(55), C(21), 0.f, 2.f, 6000, &hit);
		CHECK(steps > 0 && !hit, "an area to avoid never blocks: the agent still crosses it when there is no other way");
	}
	{
		System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
		float base = s.Query(C(3), C(20), C(55), C(20), 0.f).distance;
		int k = s.grid.AddPointZone(C(30), C(20), 80.f);
		float with = s.Query(C(3), C(20), C(55), C(20), 0.f).distance;
		s.grid.SetZone(k, SoftZone{ C(30), C(5), C(30), C(5), 80.f }); // same key = the area moves away
		float moved = s.Query(C(3), C(20), C(55), C(20), 0.f).distance;
		s.grid.RemoveZone(k);
		float removed = s.Query(C(3), C(20), C(55), C(20), 0.f).distance;
		CHECK(with > base + 1.f, "an area to avoid raises the cost of the way through it");
		CHECK(moved < with, "moving an area to avoid away makes the way through cheaper again");
		CHECK(std::fabs(removed - base) < 0.5f, "removing the area restores the original distance");
		s.grid.AddPointZone(C(30), C(20), 80.f); s.grid.ClearZones();
		CHECK(std::fabs(s.Query(C(3), C(20), C(55), C(20), 0.f).distance - base) < 0.5f, "clearing all areas to avoid restores the original distance");
	}
	// 17) Crowd avoidance: neighbours push each other apart; the mover itself is excluded.
	{
		Crowd c;
		auto mk = [](int id, float x, float y) { Agent a; a.id = id; a.x = x; a.y = y; a.r = 14.f; return a; };
		c.Add(mk(1, 100, 100)); c.Add(mk(2, 110, 100)); c.Add(mk(3, 400, 100)); c.Rebuild();
		Vec2 p1 = c.Separation(*c.Find(1)), p2 = c.Separation(*c.Find(2)), p3 = c.Separation(*c.Find(3));
		CHECK(p1.x < -0.1f && std::fabs(p1.y) < 1e-3f, "overlapping neighbour pushes the agent away from it");
		CHECK(p2.x > 0.1f && std::fabs(p1.x + p2.x) < 1e-3f, "the two agents are pushed in opposite directions, equally");
		CHECK(p3.x == 0.f && p3.y == 0.f, "a far-away agent is not pushed");
		Crowd one; one.Add(mk(7, 5, 5)); one.Rebuild();
		CHECK(one.Separation(*one.Find(7)).x == 0.f, "an agent is never pushed by itself");
		Crowd same; same.Add(mk(1, 50, 50)); same.Add(mk(2, 50, 50)); same.Rebuild();
		Vec2 q1 = same.Separation(*same.Find(1)), q2 = same.Separation(*same.Find(2));
		CHECK((q1.x != 0.f || q1.y != 0.f) && (q1.x != q2.x || q1.y != q2.y), "agents exactly on top of each other split in different directions");
		Crowd edgeBucket; edgeBucket.Add(mk(1, 63.9f, 40)); edgeBucket.Add(mk(2, 64.1f, 40)); edgeBucket.Rebuild();
		CHECK(edgeBucket.Separation(*edgeBucket.Find(1)).x < 0.f, "neighbours on either side of a bucket boundary still see each other");
		Sample s; s.reachable = true; s.vec = { 1.f, 0.f };
		Sample none = s; ApplySeparation(none, Vec2{});
		CHECK(none.vec.x == 1.f && none.vec.y == 0.f, "no neighbours: the path direction is untouched");
		Sample pushed = s; ApplySeparation(pushed, Vec2{ 0.f, 0.8f });
		CHECK(pushed.vec.y > 0.3f && pushed.vec.x > 0.f, "neighbours bend the path direction without reversing it");
		Sample arrived; arrived.reachable = true; ApplySeparation(arrived, Vec2{ 0.05f, 0.f });
		CHECK(arrived.vec.x == 0.f && arrived.vec.y == 0.f, "a unit that has arrived stays put when barely touched");
		Sample noPath; ApplySeparation(noPath, Vec2{ 1.f, 0.f });
		CHECK(!noPath.reachable && noPath.vec.x == 0.f, "no path stays no path");
	}
	// 18) Crowd simulation: a group sent to one spot piles up without avoidance and spreads out with it.
	{
		auto run = [](bool avoid, float* minDist, float* maxLeft) {
			System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
			struct U { float x, y; }; std::vector<U> u;
			for (int i = 0; i < 12; ++i) u.push_back({ C(3) + (float)(i % 4) * 12.f, C(10) + (float)(i / 4) * 12.f });
			const float tx = C(40), ty = C(25), R = 10.f;
			for (int step = 0; step < 1500; ++step)
			{
				s.crowd.Clear();
				for (int i = 0; i < 12; ++i) { Agent a; a.id = i + 1; a.x = u[i].x; a.y = u[i].y; a.r = 14.f; s.crowd.Add(a); }
				s.crowd.Rebuild();
				for (int i = 0; i < 12; ++i)
				{
					Sample sm = s.Query(u[i].x, u[i].y, tx, ty, R);
					if (avoid) ApplySeparation(sm, s.crowd.Separation(*s.crowd.Find(i + 1)));
					u[i].x += sm.vec.x * 2.f; u[i].y += sm.vec.y * 2.f;
				}
			}
			*minDist = 1e9f; *maxLeft = 0.f;
			for (int i = 0; i < 12; ++i)
			{
				*maxLeft = (std::max)(*maxLeft, std::sqrt((u[i].x - tx) * (u[i].x - tx) + (u[i].y - ty) * (u[i].y - ty)));
				for (int j = i + 1; j < 12; ++j) *minDist = (std::min)(*minDist, std::sqrt((u[i].x - u[j].x) * (u[i].x - u[j].x) + (u[i].y - u[j].y) * (u[i].y - u[j].y)));
			}
		};
		float m0, f0, m1, f1; run(false, &m0, &f0); run(true, &m1, &f1);
		printf("      12 units sent to one spot: closest pair without avoidance %.1f px, with avoidance %.1f px (personal space radius 14 px, circles touch at 28 px); farthest from the spot %.0f / %.0f px\n", m0, m1, f0, f1);
		CHECK(m1 > m0 + 8.f, "avoidance clearly keeps units further apart");
		CHECK(m1 >= 14.f, "with avoidance personal-space circles overlap by less than half");
		CHECK(m1 < 28.f + 4.f, "with avoidance units can overlap a little: they are not held at a full circle apart");
		CHECK(f1 < 160.f, "with avoidance the whole group still gathers around the target");
	}
	// 19) Exact symmetry: start, obstacle centre and target on one line must not leave the unit without a direction.
	{
		for (float R : { 0.f, 16.f })
		{
			System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
			Block(s, 28, 16, 31, 23); // rows 16..23 span y 256..384, symmetric around y = 320
			bool hit; int steps = Walk(s, C(3), 320.f, C(55), 320.f, R, 2.f, 6000, &hit);
			char msg[112]; snprintf(msg, sizeof msg, "block exactly between start and target (radius %.0f): agent still gets round it%s", R, R > 0.f ? " without touching it" : "");
			CHECK(steps > 0 && (R == 0.f || !hit), msg); // a point-sized mover may graze the corner it goes round
		}
		System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
		s.grid.AddPointZone(C(30), 320.f, 80.f);
		bool hit; int steps = Walk(s, C(3), 320.f, C(55), 320.f, 0.f, 2.f, 6000, &hit);
		CHECK(steps > 0, "area to avoid exactly between start and target: agent still gets round it");
	}
	// 20) Crowd pushes must never drive units into walls (a crowd squeezed into a corner pocket).
	{
		auto run = [](bool guard, int* enteredWall, float* maxX, float* maxY) {
			System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
			Block(s, 40, 0, 59, 39);                       // wall on the right (x >= 640)
			Block(s, 0, 30, 59, 39);                       // wall below (y >= 480)
			const int N = 30;
			struct U { float x, y; }; std::vector<U> u;
			for (int i = 0; i < N; ++i) u.push_back({ C(8) + (float)(i % 8) * 9.f, C(12) + (float)(i / 8) * 9.f });
			const float tx = 620.f, ty = 468.f, R = 12.f;  // the target sits in the corner, inside both clearance zones
			*enteredWall = 0; *maxX = 0.f; *maxY = 0.f;
			for (int step = 0; step < 3000; ++step)
			{
				s.crowd.Clear();
				for (int i = 0; i < N; ++i) { Agent a; a.id = i + 1; a.x = u[i].x; a.y = u[i].y; a.r = 14.f; s.crowd.Add(a); }
				s.crowd.Rebuild();
				for (int i = 0; i < N; ++i)
				{
					Sample sm = s.Query(u[i].x, u[i].y, tx, ty, R);
					Vec2 push = s.crowd.Separation(*s.crowd.Find(i + 1));
					if (guard) ApplyCrowd(s.grid, R, u[i].x, u[i].y, sm, push); else ApplySeparation(sm, push);
					u[i].x += sm.vec.x * 2.f; u[i].y += sm.vec.y * 2.f;
					if (s.grid.IsRawBlocked(s.grid.CellX(u[i].x), s.grid.CellY(u[i].y))) ++*enteredWall;
					*maxX = (std::max)(*maxX, u[i].x); *maxY = (std::max)(*maxY, u[i].y);
				}
			}
		};
		int in0, in1; float mx0, my0, mx1, my1; run(false, &in0, &mx0, &my0); run(true, &in1, &mx1, &my1);
		printf("      30 units in a corner pocket (walls at x = 640 and y = 480): without the guard %d steps inside walls (furthest %.0f, %.0f); with it %d (furthest %.0f, %.0f)\n", in0, mx0, my0, in1, mx1, my1);
		CHECK(in0 > 0, "(the scenario is a real test: without the guard units do end up inside the walls)");
		CHECK(in1 == 0, "with the guard crowd pushes never carry a unit into a wall");
		CHECK(mx1 <= 640.f - 6.f && my1 <= 480.f - 6.f, "with the guard units keep clear of both walls while crowding");
	}
	// 21) Debug images: PNG encoding and what the picture shows.
	{
		const char* crcIn = "123456789"; const char* adIn = "Wikipedia";
		CHECK(Crc32((const std::uint8_t*)crcIn, 9) == 0xCBF43926u, "CRC-32 matches the standard check value");
		CHECK(Adler32((const std::uint8_t*)adIn, 9) == 0x11E60398u, "Adler-32 matches the standard check value");
		System s; s.ResizeGrid(10, 8, 16.f, 0, 0); Block(s, 3, 3, 3, 3);
		s.grid.AddPointZone(C(8), C(5), 40.f); s.grid.Refresh();
		std::vector<DebugRect> rects = { DebugRect{ 100.f, 90.f, 130.f, 120.f, Rgba{ 160, 0, 0, 255 } } };
		std::vector<DebugCircle> circles = { DebugCircle{ 80.f, 64.f, 30.f, Rgba{ 0, 200, 255, 255 } } };
		DebugImage img = RenderDebugImage(s.grid, 24.f, true, rects, circles);
		auto px = [&](int x, int y) { return &img.rgba[((size_t)y * img.width + x) * 4]; };
		CHECK(img.width == 160 && img.height == 128, "image is one pixel per frame pixel when the grid is small");
		CHECK(px(56, 56)[0] == 220 && px(56, 56)[3] == 170, "obstacle cells are drawn red");
		CHECK(px(72, 56)[0] == 255 && px(72, 56)[1] == 140 && px(72, 56)[3] == 150, "cells too close to the wall for the radius are drawn orange");
		CHECK(px(8 + 3, 8 + 3)[3] == 0, "free cells far from everything stay transparent");
		CHECK(px(8 * 16 + 5, 5 * 16 + 5)[2] > 200 && px(8 * 16 + 5, 5 * 16 + 5)[3] > 100, "areas to avoid are drawn blue");
		CHECK(px(110, 64)[1] == 200 && px(110, 64)[2] == 255, "the radius circle is drawn");
		CHECK(px(115, 90)[0] == 160, "bounding rectangles are drawn");
		std::vector<std::uint8_t> png = EncodePng(img.width, img.height, img.rgba);
		CHECK(png.size() > 100 && png[0] == 0x89 && png[1] == 'P' && png[2] == 'N' && png[3] == 'G', "PNG data starts with the PNG signature");
		FILE* f = fopen("/tmp/fp_debug_test.png", "wb"); if (f) { fwrite(png.data(), 1, png.size(), f); fclose(f); }
		System big; big.ResizeGrid(512, 512, 16.f, 0, 0);
		DebugImage bigImg = RenderDebugImage(big.grid, 0.f, false, {}, {});
		CHECK((double)bigImg.width * bigImg.height <= 4.1e6, "a huge frame is shrunk so the image stays small");
	}
	// 22) Crowd robustness: odd objects must never stall or break the crowd.
	{
		Crowd c; Agent good; good.id = 1; good.x = 100.f; good.y = 100.f; good.r = 14.f;
		Agent nan; nan.id = 2; nan.x = std::nanf(""); nan.y = 0.f; nan.r = 14.f;
		Agent inf; inf.id = 3; inf.x = 0.f; inf.y = INFINITY; inf.r = 14.f;
		Agent huge; huge.id = 4; huge.x = 5000.f; huge.y = 5000.f; huge.r = 1e9f;
		Agent neg; neg.id = 5; neg.x = 110.f; neg.y = 100.f; neg.r = -50.f;
		c.Add(good); c.Add(nan); c.Add(inf); c.Add(huge); c.Add(neg); c.Rebuild();
		CHECK(c.agents.size() == 3, "agents with non-finite positions are dropped");
		Vec2 p = c.Separation(*c.Find(1));
		CHECK(std::isfinite(p.x) && std::isfinite(p.y), "separation stays finite with odd neighbours around");
		CHECK(c.Find(4) && c.Find(4)->r <= 2048.f, "an absurd personal space is clamped");
		Crowd empty; empty.Rebuild();
		CHECK(empty.Find(1) == nullptr, "an empty crowd finds nobody");
	}
	// 23) Time-sliced builds: while a new field is still being built, units keep using the previous one.
	{
		System s; s.ResizeGrid(80, 60, 16.f, 0, 0); Block(s, 30, 10, 33, 45);
		s.cellsPerFrame = 300;
		const float R = 16.f, ax = 100.f, ay = 100.f;
		Field* fa = s.Acquire(1100.f, 700.f, R); int guard = 0; while (!fa->ready && ++guard < 10000) s.Update();
		Sample before = s.Query(ax, ay, 1100.f, 700.f, R);
		Sample during = s.Query(ax, ay, 1000.f, 300.f, R); // the target moved to another cell: its field has not been built yet
		CHECK(before.reachable && during.reachable, "while the new field is still building, the previous one keeps answering");
		CHECK(!s.Acquire(1000.f, 300.f, R)->ready, "(the new field really was still building)");
		guard = 0; while (!s.Acquire(1000.f, 300.f, R)->ready && ++guard < 10000) s.Update();
		System fresh; fresh.ResizeGrid(80, 60, 16.f, 0, 0); fresh.grid.AddCells(Cells(30, 10, 33, 45), +1);
		CHECK(std::fabs(s.Query(ax, ay, 1000.f, 300.f, R).distance - fresh.Query(ax, ay, 1000.f, 300.f, R).distance) < 1e-3f, "once built, the new field takes over with the exact answer");
	}
	// 24) The cache sizes itself from memory: lots of small fields, fewer large ones, never a thrash on a normal frame.
	{
		System s; s.ResizeGrid(80, 60, 16.f, 0, 0);
		for (int i = 0; i < 40; ++i) s.Query(C(1), C(1), C(2 + i), C(20), 0.f);
		CHECK(s.fields.size() == 40, "40 different targets on a normal-sized grid all stay cached");
		System big; big.ResizeGrid(512, 512, 16.f, 0, 0);
		for (int i = 0; i < 60; ++i) big.Query(C(1), C(1), C(2 + i), C(20), 0.f);
		const size_t perField = 512u * 512u * 12u;
		CHECK(big.fields.size() == (std::max)((size_t)4, (size_t)((64u << 20) / perField)) && big.fields.size() < 40, "on a huge grid the cache holds fewer fields so memory stays bounded");
	}
	// 25) "Clear" path: it must lead to the target itself, not just to the nearest usable spot.
	{
		System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
		Sample open = s.Query(C(3), C(20), C(40), C(20), 16.f);
		CHECK(open.Clear() && !open.targetBlocked, "an open path is clear");
		Block(s, 38, 17, 42, 23);                                  // the target stands inside a wall
		Sample inWall = s.Query(C(3), C(20), C(40), C(20), 16.f);
		CHECK(inWall.reachable && inWall.targetBlocked && !inWall.Clear(), "a target inside a wall is approachable but the path is not clear");
		Sample none; none.reachable = false;
		CHECK(!none.Clear(), "no path is not clear");
		System narrow; narrow.ResizeGrid(60, 40, 16.f, 0, 0);       // player inside a corridor too narrow for a big mover
		narrow.grid.AddCells(Cells(0, 12, 59, 17), +1); narrow.grid.AddCells(Cells(0, 21, 59, 26), +1); // corridor of 3 cells = 48 px
		bool smallClear = narrow.Query(C(3), C(19), C(30), C(19), 8.f).Clear();
		bool bigClear = narrow.Query(C(3), C(31), C(30), C(19), 30.f).Clear();
		CHECK(smallClear, "a small mover has a clear path into a 48 px corridor");
		CHECK(!bigClear, "a large mover (radius 30) has no clear path into the same corridor");
	}
	// 26) Rescanning: forgetting imported obstacles in an area removes erased walls and keeps the rest.
	{
		System s; s.ResizeGrid(20, 20, 16.f, 0, 0);
		for (int x = 4; x <= 12; ++x) s.grid.MarkImported(x, 10);   // a wall of imported cells
		s.grid.AddCells(Cells(6, 10, 6, 10), +1);                    // an object also sits on one of those cells
		s.grid.Refresh();
		s.grid.ClearImported(s.grid.RectToCells(5 * 16.f, 9 * 16.f, 10 * 16.f, 12 * 16.f)); s.grid.Refresh();
		CHECK(!s.grid.IsRawBlocked(5, 10) && !s.grid.IsRawBlocked(8, 10), "imported cells inside the rescanned area are forgotten");
		CHECK(s.grid.IsRawBlocked(4, 10) && s.grid.IsRawBlocked(12, 10), "imported cells outside the area are untouched");
		CHECK(s.grid.IsRawBlocked(6, 10), "an object added as an obstacle survives a rescan of the area under it");
	}
	// 27) A mover with no path is pushed apart from neighbours instead of stacking.
	{
		Sample none; Sample gentle = none; Vec2 hard{ 0.9f, 0.f }, light{ 0.02f, 0.f };
		CHECK(SeparateWithoutPath(none, hard) && none.vec.x == 1.f && !none.Clear(), "neighbours push a pathless mover apart, without making the path clear");
		CHECK(!SeparateWithoutPath(gentle, light) && !gentle.reachable, "a light touch does not move a pathless mover");
		Sample hasPath; hasPath.reachable = true;
		CHECK(!SeparateWithoutPath(hasPath, hard), "movers that have a path are left to the normal crowd blending");
	}
	// 28) A crowd that clumps up around its destination has arrived as a whole.
	{
		System s; s.ResizeGrid(60, 40, 16.f, 0, 0);
		const int N = 30; const float tx = C(40), ty = C(25), pr = 14.f, reach = 24.f;
		struct U { float x, y; }; std::vector<U> u;
		for (int i = 0; i < N; ++i) u.push_back({ C(3) + (float)(i % 6) * 12.f, C(10) + (float)(i / 6) * 12.f });
		auto fill = [&](const std::vector<U>& pos, const std::vector<std::pair<float, float>>& extra) {
			s.crowd.Clear();
			int id = 1; for (auto& p : pos) { Agent a; a.id = id++; a.x = p.x; a.y = p.y; a.r = pr; a.pathR = 8.f; s.crowd.Add(a); }
			for (auto& p : extra) { Agent a; a.id = id++; a.x = p.first; a.y = p.second; a.r = pr; a.pathR = 8.f; s.crowd.Add(a); }
			s.crowd.Rebuild();
		};
		for (int step = 0; step < 1500; ++step)
		{
			fill(u, {});
			for (int i = 0; i < N; ++i)
			{
				Sample sm = s.Query(u[i].x, u[i].y, tx, ty, 8.f);
				ApplyCrowd(s.grid, 8.f, u[i].x, u[i].y, sm, s.crowd.Separation(*s.crowd.Find(i + 1)));
				u[i].x += sm.vec.x * 2.f; u[i].y += sm.vec.y * 2.f;
			}
		}
		// two outsiders: one far away on its own, one a little way outside the clump but not touching it
		float farthest = 0.f; for (auto& p : u) farthest = (std::max)(farthest, std::sqrt((p.x - tx) * (p.x - tx) + (p.y - ty) * (p.y - ty)));
		const std::pair<float, float> lone{ C(5), C(5) }, nearby{ tx + farthest + 3.f * pr, ty };
		fill(u, { lone, nearby });
		std::vector<char> core(s.crowd.agents.size(), 0); int inCore = 0;
		for (size_t i = 0; i < core.size(); ++i)
		{
			const Agent& a = s.crowd.agents[i]; Sample q = s.Query(a.x, a.y, tx, ty, a.pathR);
			core[i] = q.reachable && q.distance <= reach; inCore += core[i];
		}
		std::vector<char> reached = s.crowd.ReachedByClump(core);
		int clumped = 0; for (int i = 0; i < N; ++i) clumped += reached[i];
		printf("      %d units gathered: %d are within %.0f px of the target, %d count as having arrived (farthest clump member %.0f px away)\n", N, inCore, reach, clumped, farthest);
		CHECK(inCore < N, "(the crowd really did spread out: not everybody is within the distance)");
		CHECK(clumped == N, "the whole clump counts as having reached the destination");
		CHECK(!reached[N] && !reached[N + 1], "a lone unit far away and one near but not touching the clump have not arrived");
		fill(u, {});
		std::vector<char> none(s.crowd.agents.size(), 0);
		const std::vector<char> nobody = s.crowd.ReachedByClump(none);
		CHECK(std::count(nobody.begin(), nobody.end(), (char)1) == 0, "without anybody at the destination nobody has arrived");
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
