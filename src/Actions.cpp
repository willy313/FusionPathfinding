#include "Common.hpp"
#include <fstream>

// Every action starts with a breadcrumb (what is running, for crash reports) and is wrapped in FP_TRY/FP_CATCH_VOID,
// so a C++ exception is written to the log instead of taking Fusion down. See Logger.hpp.

///
/// GRID
///

void Extension::CreateGrid(int width, int height, int cellSize, int originX, int originY)
{
	FlowLog::Mark(FlowLog::C_CreateGrid, width, height, (float)cellSize);
	FP_TRY
	if (width < 1 || height < 1 || cellSize < 2 || (double)width * height > 25.0e6)
		FlowLog::Warn("CreateGrid", "CreateGrid: unusual size %d x %d cells of %d px (values below 1 cell / 2 px are raised; more than 25 million cells is far too large)", width, height, cellSize);
	sys.ResizeGrid((std::max)(width, 1), (std::max)(height, 1), (float)(std::max)(cellSize, 2), (float)originX, (float)originY);
	objectObstacles.clear();
	FP_CATCH_VOID("CreateGrid")
}

void Extension::CreateGridFitFrame(int cellSize)
{
	FlowLog::Mark(FlowLog::C_CreateGridFit, cellSize);
	FP_TRY
	if (cellSize < 2)
		FlowLog::Warn("CreateGridFit", "CreateGridFitFrame: cell size %d px is too small, using 2 px", cellSize);
	cellSize = (std::max)(cellSize, 2);
#ifdef _WIN32
	const int w = (rhPtr->rhLevelSx + cellSize - 1) / cellSize;
	const int h = (rhPtr->rhLevelSy + cellSize - 1) / cellSize;
	sys.ResizeGrid(w, h, (float)cellSize, 0.f, 0.f);
#else
	sys.ResizeGrid(64, 64, (float)cellSize, 0.f, 0.f);
#endif
	objectObstacles.clear();
	FP_CATCH_VOID("CreateGridFit")
}

void Extension::ClearObstacles()
{
	FlowLog::Mark(FlowLog::C_ClearObstacles);
	FP_TRY
	sys.grid.ClearAll();
	objectObstacles.clear();
	sys.grid.Refresh();
	FP_CATCH_VOID("ClearObstacles")
}

///
/// OBSTACLES
///

// Shared by Add / Remove. Fusion repeats an action once per selected object; we do the loop ourselves instead.
// Each object remembers which cells it covers (by Fixed value), so adding it again moves it and removing it is exact.
void Extension::ApplyObjectObstacles(bool add)
{
#ifdef _WIN32
	Runtime.CancelRepeatingObjectAction();

	// oil can be -1 if the object type has no instances
	const short oil = Runtime.GetOIListIndexFromObjectParam(0);
	if (oil == -1)
		FlowLog::Warn("ObjectObstaclesNone", "%s obstacles: the object has no instances at the moment, nothing to do", add ? "Add" : "Remove");

	for (auto pObj : DarkEdif::ObjectIterator(rhPtr, oil, DarkEdif::Selection::Implicit))
	{
		HeaderObject* const ho = pObj->get_rHo();
		const int fixedValue = ho->GetFixedValue();

		// Forget wherever this object was before
		auto it = objectObstacles.find(fixedValue);
		if (it != objectObstacles.end())
		{
			sys.grid.AddCells(it->second, -1);
			objectObstacles.erase(it);
		}
		if (!add)
			continue;

		float left, top, right, bottom;
		if (!ObjectRect(rhPtr, ho, left, top, right, bottom))
		{
			FlowLog::Warn("ObjectRectEmpty", "Add obstacles: object %d has an empty size, it is skipped", fixedValue);
			continue;
		}
		const FlowPath::CellRect cells = sys.grid.RectToCells(left, top, right, bottom);
		if (!cells.Valid())
		{
			FlowLog::Warn("ObjectOutsideGrid", "Add obstacles: object %d at (%.0f, %.0f) is outside the grid, it is skipped", fixedValue, left, top);
			continue;
		}
		sys.grid.AddCells(cells, +1);
		objectObstacles[fixedValue] = cells;
	}
	sys.grid.Refresh();
#else
	(void)add;
#endif
}

void Extension::AddObjectsAsObstacles(RunObject*)
{
	FlowLog::Mark(FlowLog::C_AddObjects);
	FP_TRY
	ApplyObjectObstacles(true);
	FP_CATCH_VOID("AddObjectsAsObstacles")
}

void Extension::RemoveObjectsAsObstacles(RunObject*)
{
	FlowLog::Mark(FlowLog::C_RemoveObjects);
	FP_TRY
	ApplyObjectObstacles(false);
	FP_CATCH_VOID("RemoveObjectsAsObstacles")
}

// Copies Fusion's built-in obstacle collision mask (obstacle-type backdrops, quick backdrops, etc.) into the grid for
// the given cells. With clearFirst the imported obstacles in those cells are forgotten first, so walls that have
// been erased disappear and new ones appear (a rescan); without it obstacles are only ever added.
void Extension::ImportCells(int layer, const FlowPath::CellRect& r, bool clearFirst)
{
#ifdef _WIN32
	if (!r.Valid())
		return;
	constexpr int CM_TEST_OBSTACLE_PLAN = 0; // Fusion collision plane for "obstacle" (platform is 1)
	HeaderObject* const ho = rdPtr->get_rHo();
	FlowPath::Grid& g = sys.grid;
	const int cell = (std::max)(1, (int)g.cellSize);
	// Precision 0 (cautious): a cell is blocked if any part of it touches an obstacle. That never misses a wall but
	// narrows corridors by up to a cell on each side. Precision 1 (balanced): only the central half of the cell is
	// tested, so corridors keep closer to their real width.
	const int inset = importPrecision >= 1 ? cell / 4 : 0;
	const int size = (std::max)(1, cell - 2 * inset);

	if (clearFirst)
		g.ClearImported(r);
	for (int cy = r.cy0; cy <= r.cy1; ++cy)
		for (int cx = r.cx0; cx <= r.cx1; ++cx)
		{
			const int px = (int)(g.originX + cx * g.cellSize) + inset;
			const int py = (int)(g.originY + cy * g.cellSize) + inset;
			// Observed in Fusion: the test returns NONZERO when the rectangle is free and 0 when it overlaps
			// an obstacle (the opposite of what the name suggests), so a zero result means blocked.
			if (callColMaskTestRect(ho, px, py, size, size, layer, CM_TEST_OBSTACLE_PLAN) == 0)
				g.MarkImported(cx, cy);
		}
	g.Refresh();
#else
	(void)layer; (void)r; (void)clearFirst;
#endif
}

void Extension::ImportFusionObstacles(int layer)
{
	FlowPath::CellRect all;
	all.cx0 = 0; all.cy0 = 0; all.cx1 = sys.grid.width - 1; all.cy1 = sys.grid.height - 1;
	ImportCells(layer, all, false);
}

void Extension::AddFusionObstacles()
{
	FlowLog::Mark(FlowLog::C_AddFusion);
	FP_TRY
	ImportFusionObstacles(-1);
	FP_CATCH_VOID("AddFusionObstacles")
}

void Extension::AddFusionObstaclesAtLayer(int layer)
{
	FlowLog::Mark(FlowLog::C_AddFusionLayer, layer);
	FP_TRY
	ImportFusionObstacles(layer);
	FP_CATCH_VOID("AddFusionObstaclesAtLayer")
}

// For when walls have been erased or added while the game runs: only this area is looked at again.
void Extension::RescanFusionObstaclesInArea(int x, int y, int width, int height, int layer)
{
	FlowLog::Mark(FlowLog::C_RescanArea, x, y, (float)width, (float)height);
	FP_TRY
	if (width <= 0 || height <= 0)
	{
		FlowLog::Warn("RescanEmpty", "Rescan Fusion obstacles in area: the area is %d x %d px, nothing to rescan", width, height);
		return;
	}
	const FlowPath::CellRect cells = sys.grid.RectToCells((float)x, (float)y, (float)x + (float)width, (float)y + (float)height);
	if (!cells.Valid())
	{
		FlowLog::Warn("RescanOutside", "Rescan Fusion obstacles in area: (%d, %d) %d x %d px is outside the grid", x, y, width, height);
		return;
	}
	ImportCells(layer, cells, true);
	FP_CATCH_VOID("RescanFusionObstaclesInArea")
}

void Extension::RescanAllFusionObstacles()
{
	FlowLog::Mark(FlowLog::C_RescanAll);
	FP_TRY
	FlowPath::CellRect all;
	all.cx0 = 0; all.cy0 = 0; all.cx1 = sys.grid.width - 1; all.cy1 = sys.grid.height - 1;
	ImportCells(-1, all, true);
	FP_CATCH_VOID("RescanAllFusionObstacles")
}

void Extension::SetFusionObstaclePrecision(int precision)
{
	FlowLog::Mark(FlowLog::C_SetPrecision, precision);
	FP_TRY
	importPrecision = precision >= 1 ? 1 : 0;
	FP_CATCH_VOID("SetFusionObstaclePrecision")
}

///
/// AREAS TO AVOID (soft obstacles)
///
/// Unlike obstacles these never block: units prefer to go round them, and cross them when there is no reasonable
/// way round. They are part of the shared fields, so they suit things that do not move much (hazards, a boss, a
/// crowded spot). For objects keeping their distance from each other use MakeObjectsAvoidEachOther instead.

// Shared by Add / Remove. Setting an object again moves its area (areas are remembered by Fixed value).
void Extension::ApplyObjectZones(bool add, int padding)
{
#ifdef _WIN32
	Runtime.CancelRepeatingObjectAction();
	const short oil = Runtime.GetOIListIndexFromObjectParam(0);
	if (oil == -1)
		FlowLog::Warn("ObjectZonesNone", "%s areas to avoid: the object has no instances at the moment, nothing to do", add ? "Add" : "Remove");

	for (auto pObj : DarkEdif::ObjectIterator(rhPtr, oil, DarkEdif::Selection::Implicit))
	{
		HeaderObject* const ho = pObj->get_rHo();
		const int fixedValue = ho->GetFixedValue();
		if (!add)
		{
			sys.grid.RemoveZone(fixedValue);
			continue;
		}
		float left, top, right, bottom;
		if (!ObjectRect(rhPtr, ho, left, top, right, bottom))
		{
			FlowLog::Warn("ObjectRectEmpty", "Add areas to avoid: object %d has an empty size, it is skipped", fixedValue);
			continue;
		}
		FlowPath::SoftZone zone;
		zone.x0 = left; zone.y0 = top; zone.x1 = right; zone.y1 = bottom;
		zone.padding = (float)(std::max)(padding, 0);
		sys.grid.SetZone(fixedValue, zone);
	}
#else
	(void)add; (void)padding;
#endif
}

void Extension::AddObjectsToAvoid(RunObject*, int padding)
{
	FlowLog::Mark(FlowLog::C_AddAvoid, padding);
	FP_TRY
	ApplyObjectZones(true, padding);
	FP_CATCH_VOID("AddObjectsToAvoid")
}

void Extension::RemoveObjectsToAvoid(RunObject*)
{
	FlowLog::Mark(FlowLog::C_RemoveAvoid);
	FP_TRY
	ApplyObjectZones(false, 0);
	FP_CATCH_VOID("RemoveObjectsToAvoid")
}

void Extension::AddPositionToAvoid(int x, int y, int padding)
{
	FlowLog::Mark(FlowLog::C_AddAvoidPos, x, y, (float)padding);
	FP_TRY
	sys.grid.AddPointZone((float)x, (float)y, (float)(std::max)(padding, 0));
	FP_CATCH_VOID("AddPositionToAvoid")
}

void Extension::ClearAreasToAvoid()
{
	FlowLog::Mark(FlowLog::C_ClearAvoid);
	FP_TRY
	sys.grid.ClearZones();
	FP_CATCH_VOID("ClearAreasToAvoid")
}

///
/// CROWD AVOIDANCE
///
/// Objects of the chosen type keep their distance from each other. Each has a personal space: a circle of the given
/// radius around its hotspot (0 = the largest circle that fits inside the object), so bodies may overlap a little.
/// The push is added when asking Direction() for one of them, and the object asking is left out.

void Extension::MakeObjectsAvoidEachOther(RunObject*, int radius)
{
	FlowLog::Mark(FlowLog::C_MakeAvoid, radius);
	FP_TRY
#ifdef _WIN32
	// Deliberately minimal, as this usually runs in Start of Frame while Fusion is still setting up:
	//  - no CancelRepeatingObjectAction(): the action only records the object type, so being called once per
	//    instance is harmless (and cancelling is a fatal error if the runtime is not in the expected state);
	//  - no walking over the instances here: Handle() collects them on the next tick.
	const short oil = Runtime.GetOIListIndexFromObjectParam(0);
	if (oil == -1)
	{
		FlowLog::Warn("MakeAvoidNone", "MakeObjectsAvoidEachOther: no object list for the chosen object (no instances yet?), ignored");
		return;
	}
	const float radiusPx = (float)(std::max)(radius, 0);

	bool found = false;
	for (AvoidGroup& g : avoidGroups)
		if (g.oil == oil) { g.radius = radiusPx; found = true; }
	if (!found)
		avoidGroups.push_back({ oil, radiusPx });
#else
	(void)radius;
#endif
	FP_CATCH_VOID("MakeObjectsAvoidEachOther")
}

void Extension::StopObjectsAvoidingEachOther()
{
	FlowLog::Mark(FlowLog::C_StopAvoid);
	FP_TRY
	avoidGroups.clear();
	sys.crowd.Clear();
	sys.crowd.Rebuild();
	FP_CATCH_VOID("StopObjectsAvoidingEachOther")
}

///
/// PERFORMANCE, SIZE AND LOGGING
///

void Extension::SetCellsPerFrame(int cells)
{
	FlowLog::Mark(FlowLog::C_SetBudget, cells);
	FP_TRY
	sys.cellsPerFrame = (std::size_t)(std::max)(cells, 0);
	FP_CATCH_VOID("SetCellsPerFrame")
}

// Scales the pathfinding size of objects (their own size = 100). Lower it if corridors close to your objects
// although they visibly fit; raise it to keep them further from walls.
void Extension::SetPathfindingSizePercent(int percent)
{
	FlowLog::Mark(FlowLog::C_SetSizePercent, percent);
	FP_TRY
	if (percent < 10 || percent > 400)
		FlowLog::Warn("SizePercent", "SetPathfindingSizePercent: %d%% is outside 10..400%%, it is limited to that range", percent);
	sizePercent = (std::min)((std::max)(percent, 10), 400);
	FP_CATCH_VOID("SetPathfindingSizePercent")
}

void Extension::SetLogFile(const TCHAR* path)
{
	FlowLog::Mark(FlowLog::C_SetLogPath);
	FP_TRY
	const std::string utf8 = path ? DarkEdif::TStringToUTF8(path) : std::string();
	if (utf8.empty())
	{
		FlowLog::Warn("LogPathEmpty", "SetLogFile: the file name is empty, the log stays at %s", FlowLog::Logger::Get().Path().c_str());
		return;
	}
	FlowLog::Logger::Get().SetPath(utf8);
	FlowLog::Logger::Get().BeginSession("log file changed");
	FP_CATCH_VOID("SetLogFile")
}

///
/// DEBUG IMAGES
///
/// Pictures of how the extension sees the frame, written as transparent PNG files that line up with the frame when
/// the grid starts at 0,0. Show one over the game with an Active Picture object (Load picture from file).

#ifdef _WIN32
static bool WriteBinaryFile(const TCHAR* path, const std::vector<std::uint8_t>& bytes)
{
	if (!path || !*path || bytes.empty())
		return false;
	std::ofstream file(path, std::ios::binary);
	if (!file)
		return false;
	file.write((const char*)bytes.data(), (std::streamsize)bytes.size());
	return (bool)file;
}
#endif

// Grid as interpreted: obstacles (red), areas to avoid (blue) and the cell grid.
void Extension::DebugSaveGridImage(const TCHAR* path)
{
	FlowLog::Mark(FlowLog::C_DebugGrid);
	FP_TRY
#ifdef _WIN32
	sys.grid.Refresh();
	const FlowPath::DebugImage img = FlowPath::RenderDebugImage(sys.grid, 0.f, true, {}, {});
	if (!WriteBinaryFile(path, FlowPath::EncodePng(img.width, img.height, img.rgba)))
		FlowLog::Warn("DebugImageWrite", "Debug grid image: could not write the file \"%s\"", path ? DarkEdif::TStringToUTF8(path).c_str() : "");
#else
	(void)path;
#endif
	FP_CATCH_VOID("DebugSaveGridImage")
}

// What objects of this size see: cells they cannot enter (orange), the zone where paths cost extra near walls
// (yellow), plus a circle for each selected object's pathfinding size (cyan), its personal space if it keeps
// its distance from others (green), its bounding rectangle (dark red) and its hotspot (magenta cross).
void Extension::DebugSaveBoundaryImage(RunObject*, const TCHAR* path)
{
	FlowLog::Mark(FlowLog::C_DebugBoundary);
	FP_TRY
#ifdef _WIN32
	Runtime.CancelRepeatingObjectAction();
	sys.grid.Refresh();
	if (!avoidGroups.empty())
		RefreshAgents();

	const short oil = Runtime.GetOIListIndexFromObjectParam(0);
	float radius = 0.f;
	std::vector<FlowPath::DebugRect> rects;
	std::vector<FlowPath::DebugCircle> circles;
	for (auto pObj : DarkEdif::ObjectIterator(rhPtr, oil, DarkEdif::Selection::Implicit))
	{
		HeaderObject* const ho = pObj->get_rHo();
		const float r = PathRadius(ho, sizePercent);
		radius = (std::max)(radius, r);
		circles.push_back({ (float)ho->get_X(), (float)ho->get_Y(), r, FlowPath::Rgba{ 0, 200, 255, 255 } });
		if (const FlowPath::Agent* agent = sys.crowd.Find(ho->GetFixedValue()))
			circles.push_back({ agent->x, agent->y, agent->r, FlowPath::Rgba{ 60, 220, 60, 255 } });
		float left, top, right, bottom;
		if (ObjectRect(rhPtr, ho, left, top, right, bottom))
			rects.push_back({ left, top, right, bottom, FlowPath::Rgba{ 160, 0, 0, 255 } });
	}
	const FlowPath::DebugImage img = FlowPath::RenderDebugImage(sys.grid, radius, false, rects, circles);
	if (!WriteBinaryFile(path, FlowPath::EncodePng(img.width, img.height, img.rgba)))
		FlowLog::Warn("DebugImageWrite", "Debug boundary image: could not write the file \"%s\"", path ? DarkEdif::TStringToUTF8(path).c_str() : "");
#else
	(void)path;
#endif
	FP_CATCH_VOID("DebugSaveBoundaryImage")
}
