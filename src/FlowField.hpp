#pragma once
// Platform-independent flow-field pathfinding core.
// No Fusion/DarkEdif dependencies, so it can be unit tested as plain C++.
//
// Model:
//  - A grid of cells with obstacles. Obstacles come from two sources that are combined: an imported mask (Fusion's
//    own obstacles) and reference-counted shapes (objects added as obstacles), so removing one object only frees
//    cells that nothing else covers.
//  - From the obstacles the grid keeps, once, the distance from each cell to the nearest obstacle edge.
//  - "Areas to avoid" are soft obstacles: cells inside (or near) them cost extra to cross, so paths go around when
//    that is cheap and through when it is the only way. They are part of the shared fields.
//  - Objects that should keep their distance from each other (crowds) are handled per query instead, with a push away
//    from nearby agents that skips the asking object itself. Shared fields cannot do that: each object would be
//    repelled by its own area.
//  - A mover has a radius (the distance from its hotspot to its farthest edge). For that radius:
//      * hard core: cells closer than the radius to an obstacle are blocked (the body cannot fit),
//      * soft zone: cells out to twice the radius cost more to cross, which pushes paths (and units that end up
//        in the zone) away from walls instead of letting them hug corners.
//  - A "field" is a cost-to-target map for ONE target cell and ONE size class, solved with fast marching
//    (eikonal equation): near-Euclidean, wraps around walls, cannot squeeze between diagonal blocks. Each cell also
//    stores a unit direction vector (the downhill gradient).
//  - Fields are created on demand when somebody asks "which way from A to B?", cached, and shared: any number of
//    movers heading for the same target cell with the same size class read the same field.
//    Radii are rounded up to size classes so mixed sizes need only a few fields.
//  - A query bilinearly blends the surrounding cell vectors, giving a continuous direction.
//  - Soft obstacles ("areas to avoid") add traversal cost around a rectangle or point: paths go around them when there
//    is a reasonable way round, and cross them when there is not.
//  - Crowd avoidance keeps movers from piling onto each other. It is applied per mover at query time (the mover
//    itself is excluded), which a shared field cannot do.

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <limits>

namespace FlowPath
{
	static constexpr float Inf = std::numeric_limits<float>::infinity();
	static constexpr float Sqrt2 = 1.41421356237f;

	struct Vec2 { float x = 0.f, y = 0.f; };

	// Inclusive range of cells.
	struct CellRect
	{
		int cx0 = 0, cy0 = 0, cx1 = -1, cy1 = -1;
		bool Valid() const { return cx1 >= cx0 && cy1 >= cy0; }
	};

	// A soft obstacle: units prefer to stay `padding` pixels away from this rectangle (a point has x0 == x1, y0 == y1).
	struct SoftZone { float x0 = 0.f, y0 = 0.f, x1 = 0.f, y1 = 0.f, padding = 0.f; };

	class Grid
	{
	public:
		int width = 0, height = 0;
		float cellSize = 16.f;
		float originX = 0.f, originY = 0.f;
		std::vector<std::uint16_t> counts;  // how many added objects cover each cell
		std::vector<std::uint8_t> imported; // cells marked by importing Fusion's obstacles
		std::vector<std::uint8_t> raw;      // effective obstacles = imported or counts > 0
		std::vector<float> edge;            // pixels from a cell centre to the nearest obstacle edge (0 inside, Inf if none)
		std::unordered_map<int, SoftZone> zones; // areas to avoid, by key (an object's Fixed value, or a negative key for positions)
		std::vector<float> soft;            // extra traversal cost per cell from the areas to avoid (empty when there are none)
		std::uint32_t version = 0;          // bumped whenever obstacles or areas to avoid change
		bool dirty = false;                 // obstacles changed; call Refresh()
		bool zonesDirty = false;            // areas to avoid changed; call Refresh()
		int nextPointKey = -1000;

		static constexpr float SoftCost = 6.f;  // extra cost multiplier at the edge of a mover's hard core (wall clearance)
		static constexpr float AvoidCost = 8.f; // extra cost multiplier in the middle of an area to avoid

		void Resize(int w, int h, float cell, float ox, float oy)
		{
			width = (std::max)(w, 1);
			height = (std::max)(h, 1);
			cellSize = cell > 0.f ? cell : 16.f;
			originX = ox;
			originY = oy;
			const size_t n = (size_t)width * height;
			counts.assign(n, 0);
			imported.assign(n, 0);
			raw.assign(n, 0);
			edge.assign(n, Inf);
			zones.clear();
			soft.clear();
			dirty = zonesDirty = false;
			++version;
		}
		bool InBounds(int cx, int cy) const { return cx >= 0 && cy >= 0 && cx < width && cy < height; }
		int Index(int cx, int cy) const { return cy * width + cx; }
		int CellX(float wx) const { return (int)std::floor((wx - originX) / cellSize); }
		int CellY(float wy) const { return (int)std::floor((wy - originY) / cellSize); }
		float CenterX(int cx) const { return originX + (cx + 0.5f) * cellSize; }
		float CenterY(int cy) const { return originY + (cy + 0.5f) * cellSize; }
		bool IsRawBlocked(int cx, int cy) const { return !InBounds(cx, cy) || raw[Index(cx, cy)] != 0; }

		// Obstacle test for a mover of the given radius (cells outside the grid count as blocked).
		bool BlockedFor(int idx, float radius) const { return raw[idx] != 0 || (radius > 0.f && edge[idx] < radius); }
		bool IsBlockedFor(int cx, int cy, float radius) const { return !InBounds(cx, cy) || BlockedFor(Index(cx, cy), radius); }
		// Extra traversal cost near walls for a mover of the given radius (0 outside the soft zone).
		float WallExtraFor(int idx, float radius) const
		{
			if (radius <= 0.f) return 0.f;
			const float e = edge[idx];
			if (e >= 2.f * radius || e < radius) return 0.f;
			const float t = 1.f - (e - radius) / radius;
			return SoftCost * t * t;
		}
		// Extra traversal cost from the areas to avoid.
		float AreaExtraFor(int idx) const { return soft.empty() ? 0.f : soft[idx]; }
		// Traversal cost multiplier (>= 1) for a mover of the given radius: wall clearance plus areas to avoid.
		float SlowFor(int idx, float radius) const { return 1.f + WallExtraFor(idx, radius) + AreaExtraFor(idx); }

		// World rectangle [x0,x1) x [y0,y1) -> the cells it overlaps (clamped to the grid; invalid if none).
		CellRect RectToCells(float x0, float y0, float x1, float y1) const
		{
			CellRect r;
			if (x1 <= x0 || y1 <= y0) return r;
			r.cx0 = (std::max)(CellX(x0), 0); r.cy0 = (std::max)(CellY(y0), 0);
			// subtract a hair so a rect ending exactly on a cell edge doesn't spill into the next cell
			r.cx1 = (std::min)(CellX(x1 - 0.001f), width - 1); r.cy1 = (std::min)(CellY(y1 - 0.001f), height - 1);
			return r;
		}
		// Adds (delta = +1) or removes (delta = -1) one shape covering these cells.
		void AddCells(const CellRect& r, int delta)
		{
			if (!r.Valid()) return;
			for (int cy = r.cy0; cy <= r.cy1; ++cy)
				for (int cx = r.cx0; cx <= r.cx1; ++cx)
				{
					std::uint16_t& c = counts[Index(cx, cy)];
					if (delta > 0) { if (c < 65535) ++c; } else if (c > 0) --c;
				}
			dirty = true;
		}
		void MarkImported(int cx, int cy)
		{
			if (!InBounds(cx, cy)) return;
			std::uint8_t& c = imported[Index(cx, cy)];
			if (!c) { c = 1; dirty = true; }
		}
		void ClearAll()
		{
			std::fill(counts.begin(), counts.end(), (std::uint16_t)0);
			std::fill(imported.begin(), imported.end(), (std::uint8_t)0);
			dirty = true;
		}

		// Areas to avoid. Setting the same key again moves the area.
		void SetZone(int key, const SoftZone& z) { zones[key] = z; zonesDirty = true; }
		void RemoveZone(int key) { if (zones.erase(key)) zonesDirty = true; }
		int AddPointZone(float x, float y, float padding)
		{
			const int key = nextPointKey--;
			SoftZone z; z.x0 = z.x1 = x; z.y0 = z.y1 = y; z.padding = padding;
			SetZone(key, z);
			return key;
		}
		void ClearZones() { if (!zones.empty()) { zones.clear(); zonesDirty = true; } }

		// Recomputes whatever changed. Cheap when nothing did.
		void Refresh()
		{
			if (!dirty && !zonesDirty) return;
			if (dirty) { dirty = false; RebuildObstacles(); }
			zonesDirty = false;
			RebuildSoft();
			++version;
		}

	private:
		// Effective obstacles and the distance-to-obstacle map.
		void RebuildObstacles()
		{
			const size_t n = (size_t)width * height;
			std::vector<float> d(n, Inf);
			bool any = false;
			for (size_t i = 0; i < n; ++i)
			{
				raw[i] = (imported[i] || counts[i] > 0) ? 1 : 0;
				if (raw[i]) { d[i] = 0.f; any = true; }
			}
			if (any)
			{
				// Two-pass chamfer distance transform (cells, centre to centre).
				auto relax = [&](int x, int y, int nx, int ny, float w) {
					if (!InBounds(nx, ny)) return;
					float v = d[Index(nx, ny)] + w;
					float& c = d[Index(x, y)];
					if (v < c) c = v;
				};
				for (int y = 0; y < height; ++y)
					for (int x = 0; x < width; ++x)
					{
						relax(x, y, x - 1, y, 1.f); relax(x, y, x, y - 1, 1.f);
						relax(x, y, x - 1, y - 1, Sqrt2); relax(x, y, x + 1, y - 1, Sqrt2);
					}
				for (int y = height - 1; y >= 0; --y)
					for (int x = width - 1; x >= 0; --x)
					{
						relax(x, y, x + 1, y, 1.f); relax(x, y, x, y + 1, 1.f);
						relax(x, y, x + 1, y + 1, Sqrt2); relax(x, y, x - 1, y + 1, Sqrt2);
					}
			}
			for (size_t i = 0; i < n; ++i)
				edge[i] = (d[i] == Inf) ? Inf : (d[i] == 0.f ? 0.f : (std::max)(0.f, d[i] - 0.5f) * cellSize);
		}

		// Extra cost around each area to avoid: full strength inside the area, fading to nothing `padding` pixels away.
		void RebuildSoft()
		{
			if (zones.empty()) { soft.clear(); return; }
			soft.assign((size_t)width * height, 0.f);
			for (const auto& kv : zones)
			{
				const SoftZone& z = kv.second;
				const float p = (std::max)(z.padding, 0.5f * cellSize); // at least the cell the area sits in
				const CellRect r = RectToCells(z.x0 - p, z.y0 - p, z.x1 + p, z.y1 + p);
				if (!r.Valid()) continue;
				for (int cy = r.cy0; cy <= r.cy1; ++cy)
					for (int cx = r.cx0; cx <= r.cx1; ++cx)
					{
						const float px = CenterX(cx), py = CenterY(cy);
						const float dx = (std::max)((std::max)(z.x0 - px, px - z.x1), 0.f);
						const float dy = (std::max)((std::max)(z.y0 - py, py - z.y1), 0.f);
						const float dd = std::sqrt(dx * dx + dy * dy);
						if (dd >= p) continue;
						float& c = soft[Index(cx, cy)];
						c = (std::min)(c + AvoidCost * (1.f - dd / p), 2.f * AvoidCost);
					}
			}
		}
	};

	class Field
	{
	public:
		int targetCX = 0, targetCY = 0;
		bool hasTarget = false;
		float radius = 0.f;              // mover radius (snapped to its size class) this field is built for
		std::uint64_t lastUsed = 0;
		int startCX = -1, startCY = -1;  // the free cell the field actually grows from (nearest free cell if the target is inside a wall)

		// Published (readable) data
		std::vector<float> dist;  // cost-weighted distance in cells; Inf = unreachable
		std::vector<Vec2> dir;    // unit downhill vector per cell (screen space: +x right, +y down)
		bool ready = false;       // true once a build has completed at least once
		std::uint32_t builtGridVersion = 0;

		// Build-in-progress data
		bool building = false;
		std::uint32_t buildGridVersion = 0;

		void SetTarget(int cx, int cy) { targetCX = cx; targetCY = cy; hasTarget = true; }
		bool NeedsBuild(const Grid& g) const { return hasTarget && (!ready || builtGridVersion != g.version); }

		// Advance the build by up to maxExpansions heap pops (0 = run to completion).
		// Returns true if the field finished building during this call.
		bool Step(const Grid& g, size_t maxExpansions)
		{
			if (!hasTarget) return false;
			if (!building)
			{
				if (!NeedsBuild(g)) return false;
				StartBuild(g);
			}
			// If the obstacles changed mid-build, restart with the new ones.
			if (buildGridVersion != g.version) StartBuild(g);

			size_t n = 0;
			const int W = g.width;
			while (!heap.empty())
			{
				if (maxExpansions && n >= maxExpansions) return false;
				Node top = heap.top(); heap.pop();
				if (state[top.i] == Accepted || top.d > work[top.i]) continue; // stale entry
				state[top.i] = Accepted;
				++n;
				int cx = top.i % W, cy = top.i / W;
				for (int k = 0; k < 4; ++k)
				{
					int nx = cx + DX[k], ny = cy + DY[k];
					if (!g.InBounds(nx, ny)) continue;
					int ni = g.Index(nx, ny);
					if (g.BlockedFor(ni, radius) || state[ni] == Accepted) continue;
					float nd = Solve(g, nx, ny);
					if (nd < work[ni]) { work[ni] = nd; heap.push({ nd, ni }); }
				}
			}
			Finish(g);
			return true;
		}

	private:
		struct Node { float d; int i; };
		struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.d > b.d; } };
		static constexpr int DX[4] = { 1, -1, 0, 0 };
		static constexpr int DY[4] = { 0, 0, 1, -1 };

		enum : std::uint8_t { Far = 0, Accepted = 2 };
		std::vector<std::uint8_t> state;
		std::vector<float> work;
		std::priority_queue<Node, std::vector<Node>, Cmp> heap;

		// First-order upwind solve of |grad T| = s (s = local traversal cost) using accepted orthogonal neighbours.
		float Solve(const Grid& g, int cx, int cy) const
		{
			auto acc = [&](int x, int y) -> float {
				if (!g.InBounds(x, y)) return Inf;
				int i = g.Index(x, y);
				return state[i] == Accepted ? work[i] : Inf;
			};
			const float s = g.SlowFor(g.Index(cx, cy), radius);
			const float ax = (std::min)(acc(cx - 1, cy), acc(cx + 1, cy));
			const float ay = (std::min)(acc(cx, cy - 1), acc(cx, cy + 1));
			if (ax == Inf && ay == Inf) return Inf;
			if (ax == Inf) return ay + s;
			if (ay == Inf) return ax + s;
			const float diff = std::fabs(ax - ay);
			if (diff >= s) return (std::min)(ax, ay) + s;
			return (ax + ay + std::sqrt(2.f * s * s - diff * diff)) * 0.5f;
		}

		void StartBuild(const Grid& g)
		{
			building = true;
			buildGridVersion = g.version;
			work.assign((size_t)g.width * g.height, Inf);
			state.assign((size_t)g.width * g.height, Far);
			heap = decltype(heap)();
			// If the target cell itself is blocked (e.g. the player stands inside a wall), grow from the nearest free cell.
			int sx = targetCX, sy = targetCY;
			if (g.IsBlockedFor(sx, sy, radius)) NearestFree(g, sx, sy);
			startCX = sx; startCY = sy;
			if (g.InBounds(sx, sy) && !g.BlockedFor(g.Index(sx, sy), radius))
			{
				work[g.Index(sx, sy)] = 0.f;
				heap.push({ 0.f, g.Index(sx, sy) });
			}
		}

		void NearestFree(const Grid& g, int& cx, int& cy) const
		{
			int bx = (std::min)((std::max)(cx, 0), g.width - 1), by = (std::min)((std::max)(cy, 0), g.height - 1);
			int maxR = (std::max)(g.width, g.height);
			for (int r = 0; r <= maxR; ++r)
				for (int y = by - r; y <= by + r; ++y)
					for (int x = bx - r; x <= bx + r; ++x)
					{
						if ((std::max)(std::abs(x - bx), std::abs(y - by)) != r) continue;
						if (g.InBounds(x, y) && !g.BlockedFor(g.Index(x, y), radius)) { cx = x; cy = y; return; }
					}
			cx = -1; cy = -1;
		}

		void Finish(const Grid& g)
		{
			dist.swap(work);
			work.clear();
			state.clear();
			building = false;
			builtGridVersion = buildGridVersion;
			ready = true;
			ComputeDirections(g);
		}

		void ComputeDirections(const Grid& g)
		{
			const int W = g.width, H = g.height;
			dir.assign((size_t)W * H, Vec2{});
			auto val = [&](int x, int y, float& out) -> bool {
				if (!g.InBounds(x, y)) return false;
				int i = g.Index(x, y);
				if (g.BlockedFor(i, radius) || dist[i] == Inf) return false;
				out = dist[i]; return true;
			};
			for (int cy = 0; cy < H; ++cy)
				for (int cx = 0; cx < W; ++cx)
				{
					int i = g.Index(cx, cy);
					float d = dist[i];
					if (g.BlockedFor(i, radius) || d == Inf || d == 0.f) continue;
					float l = 0, r = 0, u = 0, dn = 0;
					bool hl = val(cx - 1, cy, l), hr = val(cx + 1, cy, r);
					bool hu = val(cx, cy - 1, u), hd = val(cx, cy + 1, dn);
					// Smooth gradient (central differences where possible).
					float gx = 0.f, gy = 0.f;
					if (hl && hr) gx = (r - l) * 0.5f; else if (hr) gx = r - d; else if (hl) gx = d - l;
					if (hu && hd) gy = (dn - u) * 0.5f; else if (hd) gy = dn - d; else if (hu) gy = d - u;
					// Steepest descent: towards the lower neighbour on each axis. Where the path splits evenly around an
					// obstacle (start, obstacle and target on one line) the central differences cancel and leave no
					// direction at all, so use this instead, with a fixed tie-break so the unit commits to one side.
					float sx = 0.f, sy = 0.f;
					{
						const float dl = hl ? d - l : -Inf, drr = hr ? d - r : -Inf; // positive = that neighbour is lower
						const float du = hu ? d - u : -Inf, dd = hd ? d - dn : -Inf;
						if (drr > 0.f || dl > 0.f) sx = (drr >= dl) ? -drr : dl;  // gradient points away from the lower side
						if (dd > 0.f || du > 0.f) sy = (dd >= du) ? -dd : du;
					}
					const float cl = std::sqrt(gx * gx + gy * gy), sl = std::sqrt(sx * sx + sy * sy);
					// The smooth direction must really lead downhill: the neighbour it points towards has to be lower.
					// (On a ridge it can point back uphill, which would trap the unit between two cells.)
					bool uphill = false;
					if (cl > 1e-6f)
					{
						const float ddx = -gx, ddy = -gy;
						if (std::fabs(ddx) > 0.25f * cl) uphill |= ddx > 0.f ? !(hr && r < d) : !(hl && l < d);
						if (std::fabs(ddy) > 0.25f * cl) uphill |= ddy > 0.f ? !(hd && dn < d) : !(hu && u < d);
					}
					if (cl < 0.3f * sl || uphill) { gx = sx; gy = sy; }
					float len = std::sqrt(gx * gx + gy * gy);
					if (len > 1e-6f) dir[i] = { -gx / len, -gy / len };
				}
		}
	};

	// Result of sampling a field at a world position.
	struct Sample
	{
		Vec2 vec;             // unit vector, screen space (+y down); zero if none or arrived
		float distance = Inf; // approximate path cost in pixels (a little above the true distance near walls); Inf if unreachable
		bool reachable = false;
		bool atTarget = false;
	};

	// Nearest free, reachable cell to (cx,cy) within a search radius that scales with the mover's radius.
	inline bool NearestUsableCell(const Grid& g, const Field& f, int& cx, int& cy)
	{
		const int maxR = 3 + (int)std::ceil(2.f * f.radius / g.cellSize);
		const int bx = (std::min)((std::max)(cx, 0), g.width - 1), by = (std::min)((std::max)(cy, 0), g.height - 1);
		for (int r = 0; r <= maxR; ++r)
			for (int y = by - r; y <= by + r; ++y)
				for (int x = bx - r; x <= bx + r; ++x)
				{
					if ((std::max)(std::abs(x - bx), std::abs(y - by)) != r) continue;
					if (!g.InBounds(x, y)) continue;
					int i = g.Index(x, y);
					if (!g.BlockedFor(i, f.radius) && f.dist[i] != Inf) { cx = x; cy = y; return true; }
				}
		return false;
	}

	// Bilinear sample at world position (px,py). (tx,ty) is the exact target position, used for the final approach.
	// Blocked/unreachable cells get zero weight so vectors never blend toward walls. A position inside a blocked
	// cell is pushed out toward the nearest usable cell, and a position outside the grid is pushed back in.
	inline Sample SampleField(const Grid& g, const Field& f, float px, float py, float tx, float ty)
	{
		Sample s;
		if (!f.ready || !f.hasTarget) return s;

		float wx = px, wy = py;
		{
			const float lo_x = g.originX + 0.5f * g.cellSize, hi_x = g.originX + (g.width - 0.5f) * g.cellSize;
			const float lo_y = g.originY + 0.5f * g.cellSize, hi_y = g.originY + (g.height - 0.5f) * g.cellSize;
			wx = (std::min)((std::max)(px, lo_x), hi_x);
			wy = (std::min)((std::max)(py, lo_y), hi_y);
		}
		const float pushX = wx - px, pushY = wy - py; // non-zero only when the position was outside the grid
		const bool outside = pushX != 0.f || pushY != 0.f;

		// Position in cell-centre space: centre of cell (0,0) is at 0.0
		float fx = (wx - g.originX) / g.cellSize - 0.5f;
		float fy = (wy - g.originY) / g.cellSize - 0.5f;
		int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy);
		float tfx = fx - x0, tfy = fy - y0;

		float vx = 0.f, vy = 0.f, wsum = 0.f, dsum = 0.f, dw = 0.f;
		for (int j = 0; j < 2; ++j)
			for (int i = 0; i < 2; ++i)
			{
				int cx = x0 + i, cy = y0 + j;
				if (!g.InBounds(cx, cy)) continue;
				int idx = g.Index(cx, cy);
				if (g.BlockedFor(idx, f.radius) || f.dist[idx] == Inf) continue;
				float w = (i ? tfx : 1.f - tfx) * (j ? tfy : 1.f - tfy);
				if (w <= 0.f) continue;
				vx += f.dir[idx].x * w; vy += f.dir[idx].y * w;
				dsum += f.dist[idx] * w; dw += w;
				wsum += w;
			}
		if (wsum > 1e-6f) { vx /= wsum; vy /= wsum; dsum /= dw; dw = 1.f; }

		// Is the position inside a blocked cell? Two cases:
		//  - deep inside (all four surrounding cells unusable): head straight for the nearest usable cell;
		//  - only clipping a wall edge (some neighbours usable): keep following the path and add a gentle push out,
		//    so a unit that grazes a corner is nudged away instead of being thrown back and forth.
		const bool deep = wsum <= 1e-6f;
		const bool inside = deep || g.IsBlockedFor(g.CellX(wx), g.CellY(wy), f.radius);
		if (inside)
		{
			int cx = g.CellX(wx), cy = g.CellY(wy);
			if (!NearestUsableCell(g, f, cx, cy)) { if (deep) return s; }
			else
			{
				float ex = g.CenterX(cx) - wx, ey = g.CenterY(cy) - wy;
				float el = std::sqrt(ex * ex + ey * ey);
				if (el > 1e-3f) { ex /= el; ey /= el; } else { ex = ey = 0.f; }
				if (deep)
				{
					vx = ex; vy = ey;
					dsum = f.dist[g.Index(cx, cy)] + el / g.cellSize; dw = 1.f;
				}
				else
				{
					float pl = std::sqrt(vx * vx + vy * vy);
					if (pl > 1e-6f) { vx /= pl; vy /= pl; }
					vx += 0.6f * ex; vy += 0.6f * ey;
				}
			}
		}

		s.reachable = true;
		s.distance = (dw > 0.f ? dsum / dw : 0.f) * g.cellSize;
		if (outside)
		{
			float pl = std::sqrt(pushX * pushX + pushY * pushY), vl = std::sqrt(vx * vx + vy * vy);
			if (vl > 1e-6f) { vx /= vl; vy /= vl; }
			vx += pushX / pl; vy += pushY / pl; // come back into the grid first
		}
		float len = std::sqrt(vx * vx + vy * vy);
		if (len > 1e-6f) s.vec = { vx / len, vy / len };

		// Within half a cell of the goal: report arrival and aim at the goal itself (the exact target position when
		// it is on a free cell, otherwise the free cell the field grows from), never into a wall.
		s.atTarget = s.distance <= g.cellSize * 0.5f;
		if (s.atTarget && f.startCX >= 0 && !inside && !outside)
		{
			const bool exact = f.startCX == f.targetCX && f.startCY == f.targetCY;
			float ax = (exact ? tx : g.CenterX(f.startCX)) - px, ay = (exact ? ty : g.CenterY(f.startCY)) - py;
			float al = std::sqrt(ax * ax + ay * ay);
			if (al > 1.f) s.vec = { ax / al, ay / al }; else s.vec = { 0.f, 0.f };
		}
		return s;
	}

	// Fusion angle convention: 0 = right, counter-clockwise, 0..360. Screen y is down, hence the flip.
	inline float VecToFusionAngle(Vec2 v)
	{
		if (v.x == 0.f && v.y == 0.f) return 0.f;
		float a = std::atan2(-v.y, v.x) * (180.f / 3.14159265358979f);
		if (a < 0.f) a += 360.f;
		if (a >= 360.f) a -= 360.f;
		return a;
	}

	// Direction in the requested format, or -1 when there is no usable direction (no path, or already arrived).
	//  directions = 4, 8, 16 or 32: Fusion's own direction numbers (0-31 scale, so the result can go straight into
	//               "Set direction"): 4 gives 0/8/16/24, 8 gives multiples of 4, 16 multiples of 2, 32 every number.
	//  directions = 360 (or anything else >= 360 / <= 0): smooth angle in degrees, 0 = right, counter-clockwise.
	//  other values of 2..359 snap to that many directions, still on the 0-31 scale (possibly fractional).
	inline float FormatDirection(const Sample& s, int directions)
	{
		if (!s.reachable || (s.vec.x == 0.f && s.vec.y == 0.f)) return -1.f;
		const float angle = VecToFusionAngle(s.vec);
		if (directions <= 1 || directions >= 360) return angle;
		const float step = 360.f / (float)directions;
		const int idx = (int)std::lround(angle / step) % directions;
		return (float)idx * (32.f / (float)directions);
	}

	// ---- Crowd avoidance ---------------------------------------------------------------------------------------
	// A mover that takes part in crowd avoidance. Positions are hotspot positions and r is its personal space: a circle
	// around the hotspot (not around the whole body), so bodies may overlap a little before they push each other.
	struct Agent { int id = 0; float x = 0.f, y = 0.f, r = 0.f; };

	// All agents of this tick in a spatial hash, so "who is near me?" costs a handful of lookups.
	class Crowd
	{
	public:
		std::vector<Agent> agents;

		void Clear() { agents.clear(); index.clear(); buckets.clear(); }
		void Add(const Agent& a) { agents.push_back(a); }
		// Call once after adding this tick's agents.
		void Rebuild()
		{
			index.clear(); buckets.clear();
			maxExtent = 0.f;
			for (const Agent& a : agents) maxExtent = (std::max)(maxExtent, a.r);
			bucketSize = (std::max)(32.f, 2.f * maxExtent);
			for (size_t i = 0; i < agents.size(); ++i)
			{
				index[agents[i].id] = i;
				buckets[Key(BX(agents[i].x), BX(agents[i].y))].push_back(i);
			}
		}
		const Agent* Find(int id) const
		{
			auto it = index.find(id);
			return it == index.end() ? nullptr : &agents[it->second];
		}

		// Sum of pushes away from everybody overlapping the agent's personal space (itself excluded). Each push has a
		// length of 0 (circles just touching) to 3 (on top of each other). Circles may overlap a little: a unit that is
		// pulled towards its target settles at about two thirds of the touching distance.
		Vec2 Separation(const Agent& self) const
		{
			Vec2 push;
			const float reach = self.r + maxExtent;
			const int rings = (int)std::ceil(reach / bucketSize);
			const int bx = BX(self.x), by = BX(self.y);
			for (int j = -rings; j <= rings; ++j)
				for (int i = -rings; i <= rings; ++i)
				{
					auto it = buckets.find(Key(bx + i, by + j));
					if (it == buckets.end()) continue;
					for (size_t k : it->second)
					{
						const Agent& o = agents[k];
						if (o.id == self.id) continue;
						const float range = self.r + o.r; // personal-space circles touch at this distance
						const float dx = self.x - o.x, dy = self.y - o.y;
						const float d = std::sqrt(dx * dx + dy * dy);
						if (d >= range) continue;
						float strength = 1.f - d / range;
						if (d < 0.5f * range) strength += 2.f * (1.f - d / (0.5f * range)); // deep overlap: push much harder
						if (d > 1e-3f) { push.x += dx / d * strength; push.y += dy / d * strength; }
						else
						{
							// Exactly on top of each other: split in a direction that differs per object
							const float ang = (float)(self.id % 1000) * 2.3999632f;
							push.x += std::cos(ang) * strength; push.y += std::sin(ang) * strength;
						}
					}
				}
			return push;
		}

	private:
		float maxExtent = 0.f, bucketSize = 32.f;
		std::unordered_map<int, size_t> index;
		std::unordered_map<std::uint64_t, std::vector<size_t>> buckets;
		int BX(float v) const { return (int)std::floor(v / bucketSize); }
		static std::uint64_t Key(int bx, int by) { return ((std::uint64_t)(std::uint32_t)bx << 32) | (std::uint32_t)by; }
	};

	// Blends a push away from neighbours into a sample's direction (the path still dominates when nobody is close).
	inline void ApplySeparation(Sample& s, Vec2 push, float gain = 3.f)
	{
		if (!s.reachable) return;
		const float pl = std::sqrt(push.x * push.x + push.y * push.y);
		if (pl < 1e-4f) return;
		const float vl = std::sqrt(s.vec.x * s.vec.x + s.vec.y * s.vec.y);
		if (vl < 1e-6f && pl * gain < 0.3f) return; // already at the goal and barely touched: stay put
		float vx = vl > 1e-6f ? s.vec.x / vl : 0.f, vy = vl > 1e-6f ? s.vec.y / vl : 0.f;
		vx += push.x * gain; vy += push.y * gain;
		const float len = std::sqrt(vx * vx + vy * vy);
		if (len > 1e-6f) s.vec = { vx / len, vy / len };
	}


	// Distance from a position to the nearest obstacle edge (bilinear over the cell centres; Inf if there are no obstacles).
	inline float EdgeDistanceAt(const Grid& g, float x, float y)
	{
		const float fx = (std::min)((std::max)((x - g.originX) / g.cellSize - 0.5f, 0.f), (float)(g.width - 1));
		const float fy = (std::min)((std::max)((y - g.originY) / g.cellSize - 0.5f, 0.f), (float)(g.height - 1));
		const int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy);
		const int x1 = (std::min)(x0 + 1, g.width - 1), y1 = (std::min)(y0 + 1, g.height - 1);
		const float tx = fx - x0, ty = fy - y0;
		const float a = g.edge[g.Index(x0, y0)], b = g.edge[g.Index(x1, y0)], c = g.edge[g.Index(x0, y1)], d = g.edge[g.Index(x1, y1)];
		if (a == Inf || b == Inf || c == Inf || d == Inf) return Inf;
		return (a * (1.f - tx) + b * tx) * (1.f - ty) + (c * (1.f - tx) + d * tx) * ty;
	}

	// Keeps a direction from heading into a wall: inside the wall clearance zone the part of the direction that points
	// at the wall is taken away (fully in the hard core, fading out towards the edge of the soft zone).
	inline void ApplyWallGuard(const Grid& g, float radius, float x, float y, Sample& s)
	{
		if (!s.reachable || (s.vec.x == 0.f && s.vec.y == 0.f)) return;
		const float e = EdgeDistanceAt(g, x, y);
		const float zone = 2.f * radius + g.cellSize * 0.5f;
		if (e >= zone) return;
		const float h = g.cellSize * 0.5f;
		float nx = EdgeDistanceAt(g, x + h, y) - EdgeDistanceAt(g, x - h, y);
		float ny = EdgeDistanceAt(g, x, y + h) - EdgeDistanceAt(g, x, y - h);
		const float nl = std::sqrt(nx * nx + ny * ny);
		if (nl < 1e-6f) return;
		nx /= nl; ny /= nl;                       // points away from the wall
		const float into = -(s.vec.x * nx + s.vec.y * ny);
		if (into <= 0.f) return;                  // already heading along or away from the wall
		const float w = e <= radius ? 1.f : (zone - e) / (zone - radius);
		float vx = s.vec.x + nx * into * w, vy = s.vec.y + ny * into * w; // drop the into-the-wall part
		vx += nx * 0.2f * w; vy += ny * 0.2f * w;                          // and lean slightly away from it
		const float len = std::sqrt(vx * vx + vy * vy);
		if (len > 1e-6f) s.vec = { vx / len, vy / len };
	}

	// Crowd avoidance as the extension applies it: blend the push away from neighbours into the path direction,
	// then make sure the result does not head into a wall.
	inline void ApplyCrowd(const Grid& g, float radius, float x, float y, Sample& s, Vec2 push)
	{
		if (push.x * push.x + push.y * push.y < 1e-8f) return;
		ApplySeparation(s, push);
		ApplyWallGuard(g, radius, x, y, s);
	}


	// ---- Debug images -------------------------------------------------------------------------------------------
	// Draws how the grid is interpreted into a transparent RGBA image that lines up 1:1 with the frame (when the grid
	// starts at 0,0), so it can be shown over the game with Fusion's own Active Picture.
	struct Rgba { std::uint8_t r = 0, g = 0, b = 0, a = 0; };
	struct DebugCircle { float x, y, r; Rgba color; };
	struct DebugRect { float x0, y0, x1, y1; Rgba color; };
	struct DebugImage { int width = 0, height = 0; std::vector<std::uint8_t> rgba; };

	inline std::uint32_t Crc32(const std::uint8_t* data, size_t n, std::uint32_t crc = 0)
	{
		static std::uint32_t table[256];
		static bool ready = false;
		if (!ready)
		{
			for (std::uint32_t i = 0; i < 256; ++i)
			{
				std::uint32_t c = i;
				for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : (c >> 1);
				table[i] = c;
			}
			ready = true;
		}
		crc = ~crc;
		for (size_t i = 0; i < n; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
		return ~crc;
	}
	inline std::uint32_t Adler32(const std::uint8_t* data, size_t n)
	{
		std::uint32_t a = 1, b = 0;
		for (size_t i = 0; i < n; ++i) { a = (a + data[i]) % 65521u; b = (b + a) % 65521u; }
		return (b << 16) | a;
	}

	// Minimal PNG writer (RGBA, 8 bit, uncompressed deflate blocks): no libraries needed.
	inline std::vector<std::uint8_t> EncodePng(int w, int h, const std::vector<std::uint8_t>& rgba)
	{
		std::vector<std::uint8_t> out;
		if (w <= 0 || h <= 0 || rgba.size() < (size_t)w * h * 4) return out;
		auto be32 = [](std::vector<std::uint8_t>& v, std::uint32_t x) { v.push_back(x >> 24); v.push_back(x >> 16); v.push_back(x >> 8); v.push_back(x); };
		auto chunk = [&](const char* type, const std::vector<std::uint8_t>& data) {
			be32(out, (std::uint32_t)data.size());
			std::vector<std::uint8_t> body(type, type + 4);
			body.insert(body.end(), data.begin(), data.end());
			out.insert(out.end(), body.begin(), body.end());
			be32(out, Crc32(body.data(), body.size()));
		};
		const std::uint8_t sig[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
		out.insert(out.end(), sig, sig + 8);

		std::vector<std::uint8_t> ihdr;
		be32(ihdr, (std::uint32_t)w); be32(ihdr, (std::uint32_t)h);
		ihdr.push_back(8); ihdr.push_back(6); ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0); // 8 bit RGBA
		chunk("IHDR", ihdr);

		std::vector<std::uint8_t> raw; // scanlines, each with filter byte 0
		raw.reserve((size_t)h * (1 + (size_t)w * 4));
		for (int y = 0; y < h; ++y)
		{
			raw.push_back(0);
			raw.insert(raw.end(), rgba.begin() + (size_t)y * w * 4, rgba.begin() + (size_t)(y + 1) * w * 4);
		}
		std::vector<std::uint8_t> z; z.push_back(0x78); z.push_back(0x01);
		for (size_t pos = 0; pos < raw.size();)
		{
			const size_t len = (std::min)((size_t)65535, raw.size() - pos);
			z.push_back(pos + len == raw.size() ? 1 : 0);
			z.push_back(len & 0xFF); z.push_back(len >> 8); z.push_back(~len & 0xFF); z.push_back((~len >> 8) & 0xFF);
			z.insert(z.end(), raw.begin() + pos, raw.begin() + pos + len);
			pos += len;
		}
		be32(z, Adler32(raw.data(), raw.size()));
		chunk("IDAT", z);
		chunk("IEND", std::vector<std::uint8_t>());
		return out;
	}

	// radius > 0 also shows what a mover of that radius sees: cells it cannot enter and the zone that costs extra.
	inline DebugImage RenderDebugImage(const Grid& g, float radius, bool gridLines,
		const std::vector<DebugRect>& rects, const std::vector<DebugCircle>& circles)
	{
		DebugImage img;
		const float totalW = g.width * g.cellSize, totalH = g.height * g.cellSize;
		float scale = 1.f; // shrink very large frames so the image stays a reasonable size
		while ((totalW / scale) * (totalH / scale) > 4.0e6f) scale += 1.f;
		img.width = (std::max)(1, (int)std::ceil(totalW / scale));
		img.height = (std::max)(1, (int)std::ceil(totalH / scale));
		img.rgba.assign((size_t)img.width * img.height * 4, 0);

		auto blend = [&](int x, int y, const Rgba& c) {
			if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
			std::uint8_t* p = &img.rgba[((size_t)y * img.width + x) * 4];
			const float sa = c.a / 255.f, da = p[3] / 255.f, oa = sa + da * (1.f - sa);
			if (oa <= 0.f) return;
			p[0] = (std::uint8_t)((c.r * sa + p[0] * da * (1.f - sa)) / oa + 0.5f);
			p[1] = (std::uint8_t)((c.g * sa + p[1] * da * (1.f - sa)) / oa + 0.5f);
			p[2] = (std::uint8_t)((c.b * sa + p[2] * da * (1.f - sa)) / oa + 0.5f);
			p[3] = (std::uint8_t)(oa * 255.f + 0.5f);
		};
		auto fill = [&](int x0, int y0, int x1, int y1, const Rgba& c) {
			for (int y = y0; y < y1; ++y) for (int x = x0; x < x1; ++x) blend(x, y, c);
		};

		for (int cy = 0; cy < g.height; ++cy)
			for (int cx = 0; cx < g.width; ++cx)
			{
				const int x0 = (int)std::floor(cx * g.cellSize / scale), y0 = (int)std::floor(cy * g.cellSize / scale);
				const int x1 = (std::max)(x0 + 1, (int)std::floor((cx + 1) * g.cellSize / scale));
				const int y1 = (std::max)(y0 + 1, (int)std::floor((cy + 1) * g.cellSize / scale));
				const int idx = g.Index(cx, cy);
				if (g.raw[idx]) fill(x0, y0, x1, y1, Rgba{ 220, 40, 40, 170 });                       // obstacle
				else if (radius > 0.f && g.BlockedFor(idx, radius)) fill(x0, y0, x1, y1, Rgba{ 255, 140, 0, 150 }); // too close to a wall for this radius
				else
				{
					const float wall = g.WallExtraFor(idx, radius);
					if (wall > 0.01f) fill(x0, y0, x1, y1, Rgba{ 255, 215, 0, (std::uint8_t)(std::min)(130.f, 130.f * wall / Grid::SoftCost) }); // costs extra near walls
				}
				const float area = g.AreaExtraFor(idx);
				if (area > 0.01f) fill(x0, y0, x1, y1, Rgba{ 40, 90, 255, (std::uint8_t)(std::min)(170.f, 170.f * area / Grid::AvoidCost) }); // area to avoid
			}

		if (gridLines && g.cellSize / scale >= 6.f)
		{
			const Rgba line{ 90, 90, 90, 100 };
			for (int cx = 0; cx <= g.width; ++cx) { const int x = (int)std::floor(cx * g.cellSize / scale); fill(x, 0, x + 1, img.height, line); }
			for (int cy = 0; cy <= g.height; ++cy) { const int y = (int)std::floor(cy * g.cellSize / scale); fill(0, y, img.width, y + 1, line); }
		}
		for (const DebugRect& r : rects)
		{
			const int x0 = (int)((r.x0 - g.originX) / scale), x1 = (int)((r.x1 - g.originX) / scale);
			const int y0 = (int)((r.y0 - g.originY) / scale), y1 = (int)((r.y1 - g.originY) / scale);
			fill(x0, y0, x1 + 1, y0 + 1, r.color); fill(x0, y1, x1 + 1, y1 + 1, r.color);
			fill(x0, y0, x0 + 1, y1 + 1, r.color); fill(x1, y0, x1 + 1, y1 + 1, r.color);
		}
		for (const DebugCircle& c : circles)
		{
			const float cx = (c.x - g.originX) / scale, cy = (c.y - g.originY) / scale, rr = c.r / scale;
			const int steps = (std::max)(48, (int)(rr * 8.f));
			for (int i = 0; i < steps; ++i)
			{
				const float a = 6.2831853f * (float)i / (float)steps;
				const int px = (int)std::lround(cx + rr * std::cos(a)), py = (int)std::lround(cy + rr * std::sin(a));
				fill(px, py, px + 2, py + 2, c.color);
			}
			const Rgba cross{ 255, 0, 255, 255 }; // hotspot
			fill((int)cx - 4, (int)cy, (int)cx + 5, (int)cy + 1, cross); fill((int)cx, (int)cy - 4, (int)cx + 1, (int)cy + 5, cross);
		}
		return img;
	}

	// All the fields of one extension object, created on demand and cached.
	class System
	{
	public:
		Grid grid;
		Crowd crowd;                           // objects that keep their distance from each other (refreshed every tick)
		std::unordered_map<std::uint64_t, Field> fields;
		std::uint64_t tick = 0;
		std::size_t maxFields = 16;            // cache size; the least recently used field is dropped first
		std::uint64_t idleTicks = 600;         // fields unused for this many updates are dropped
		std::size_t cellsPerFrame = 0;         // 0 = build fields completely the moment they are asked for

		// Rounds a radius up to a size class so mixed-size movers share fields.
		void SizeClass(float radiusPx, int& cls, float& snapped) const
		{
			const float step = (std::max)(4.f, grid.cellSize * 0.5f);
			cls = radiusPx <= 0.f ? 0 : (std::min)(64, (int)std::ceil(radiusPx / step));
			snapped = cls * step;
		}

		// Finds or creates the field for the cell containing (tx,ty) and the size class of radiusPx.
		// With cellsPerFrame == 0 the field is complete when this returns; otherwise it may still be building.
		Field* Acquire(float tx, float ty, float radiusPx)
		{
			grid.Refresh();
			int cls; float snapped;
			SizeClass(radiusPx, cls, snapped);
			const int cx = (std::min)((std::max)(grid.CellX(tx), 0), grid.width - 1);
			const int cy = (std::min)((std::max)(grid.CellY(ty), 0), grid.height - 1);
			const std::uint64_t key = ((std::uint64_t)cls << 44) | ((std::uint64_t)cy << 22) | (std::uint64_t)cx;
			auto it = fields.find(key);
			if (it == fields.end())
			{
				if (fields.size() >= maxFields) EvictOldest();
				it = fields.emplace(key, Field()).first;
				it->second.SetTarget(cx, cy);
				it->second.radius = snapped;
			}
			Field& f = it->second;
			f.lastUsed = tick;
			if (cellsPerFrame == 0) f.Step(grid, 0);
			return &f;
		}

		// Which way (and how far) from A to B for a mover of the given radius.
		Sample Query(float ax, float ay, float bx, float by, float radiusPx)
		{
			Field* f = Acquire(bx, by, radiusPx);
			if (!f->ready) return Sample();
			return SampleField(grid, *f, ax, ay, bx, by);
		}

		// Call once per tick: ages the cache and continues builds within the per-frame budget.
		void Update()
		{
			++tick;
			grid.Refresh();
			for (auto it = fields.begin(); it != fields.end();)
				it = (tick - it->second.lastUsed > idleTicks) ? fields.erase(it) : std::next(it);

			size_t left = cellsPerFrame;
			for (auto& kv : fields)
			{
				Field& f = kv.second;
				if (!f.building && !f.NeedsBuild(grid)) continue;
				if (cellsPerFrame == 0) { f.Step(grid, 0); continue; }
				if (left == 0) break;
				f.Step(grid, left);
				left = left > cellsPerFrame / 2 ? left - cellsPerFrame / 2 : 0; // simple fair-share cap
			}
		}

		void ResizeGrid(int w, int h, float cell, float ox, float oy)
		{
			grid.Resize(w, h, cell, ox, oy);
			fields.clear();
		}

	private:
		void EvictOldest()
		{
			auto oldest = fields.begin();
			for (auto it = fields.begin(); it != fields.end(); ++it)
				if (it->second.lastUsed < oldest->second.lastUsed) oldest = it;
			if (oldest != fields.end()) fields.erase(oldest);
		}
	};
}
