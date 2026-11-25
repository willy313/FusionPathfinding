// Simulates a unit with a round body (like a Fusion sprite) that STOPS when it touches a wall.
#include "../src/FlowField.hpp"
#include <cstdio>
using namespace FlowPath;

struct Result { bool arrived; int steps; int stalledSteps; float x, y; };

static bool BodyHitsWall(const Grid& g, float x, float y, float R)
{
	int cx = g.CellX(x), cy = g.CellY(y), span = (int)std::ceil(R / g.cellSize) + 1;
	for (int j = -span; j <= span; ++j) for (int i = -span; i <= span; ++i)
	{
		int ox = cx + i, oy = cy + j;
		if (!g.InBounds(ox, oy)) continue;
		if (!g.raw[g.Index(ox, oy)]) continue;
		float rx0 = g.originX + ox * g.cellSize, ry0 = g.originY + oy * g.cellSize;
		float dx = (std::max)((std::max)(rx0 - x, x - (rx0 + g.cellSize)), 0.f), dy = (std::max)((std::max)(ry0 - y, y - (ry0 + g.cellSize)), 0.f);
		if (std::sqrt(dx * dx + dy * dy) < R) return true;
	}
	return false;
}

static Result Run(System& s, float sx, float sy, float tx, float ty, float R, float radiusForField, float speed, int maxSteps)
{
	float x = sx, y = sy; int stalled = 0;
	for (int i = 0; i < maxSteps; ++i)
	{
		float dx = tx - x, dy = ty - y;
		if (std::sqrt(dx * dx + dy * dy) < 12.f) return { true, i, stalled, x, y };
		Sample sm = s.Query(x, y, tx, ty, radiusForField);
		// Fusion's Set direction only knows 32 directions
		float a = VecToFusionAngle(sm.vec); int d32 = (int)std::lround(a / 11.25f) % 32; float qa = d32 * 11.25f * 3.14159265f / 180.f;
		Vec2 v = sm.reachable && (sm.vec.x != 0.f || sm.vec.y != 0.f) ? Vec2{ std::cos(qa), -std::sin(qa) } : Vec2{ 1.f, 0.f };
		float nx = x + v.x * speed, ny = y + v.y * speed;
		if (BodyHitsWall(s.grid, nx, ny, R)) { ++stalled; continue; } // stop when colliding with the wall
		x = nx; y = ny;
	}
	return { false, maxSteps, stalled, x, y };
}

int main()
{
	int fails = 0;
	for (float pad : { 16.f, 24.f }) for (float cell : { 16.f, 8.f })
	{
		System s; int W = (int)(640 / cell), H = (int)(480 / cell);
		s.ResizeGrid(W, H, cell, 0, 0);
		s.grid.AddCells(s.grid.RectToCells(288, 32, 352, 352), +1);   // tall wall (like the test frame)
		s.grid.AddCells(s.grid.RectToCells(288, 400, 352, 430), +1);  // a second bar below it, leaving a gap
		s.grid.Refresh();
		struct Case { const char* name; float sx, sy; } cases[] = { { "right of wall, touching it", 352 + 14.5f, 200 }, { "right of wall, 40px away", 400, 200 }, { "right of wall, near the top", 352 + 14.5f, 60 } };
		for (auto& c : cases)
		{
			Result r = Run(s, c.sx, c.sy, 100, 300, 14.f, pad, 2.f, 6000);
			printf("radius %2.0f cell %2.0f  %-28s -> %s after %4d steps, stalled %4d, ended (%.0f,%.0f)\n", pad, cell, c.name, r.arrived ? "ARRIVED" : "STUCK  ", r.steps, r.stalledSteps, r.x, r.y);
			if (!r.arrived) ++fails;
		}
	}
	printf("\n%s\n", fails ? "SOME RUNS GOT STUCK" : "ALL RUNS ARRIVED");
	return fails ? 1 : 0;
}
