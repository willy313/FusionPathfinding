#include "Common.hpp"

///
/// GRID
///

void Extension::CreateGrid(int width, int height, int cellSize, int originX, int originY)
{
	sys.ResizeGrid(width, height, (float)cellSize, (float)originX, (float)originY);
	objectObstacles.clear();
}

void Extension::CreateGridFitFrame(int cellSize)
{
	cellSize = (std::max)(cellSize, 2);
#ifdef _WIN32
	const int w = (rhPtr->rhLevelSx + cellSize - 1) / cellSize;
	const int h = (rhPtr->rhLevelSy + cellSize - 1) / cellSize;
	sys.ResizeGrid(w, h, (float)cellSize, 0.f, 0.f);
#else
	sys.ResizeGrid(64, 64, (float)cellSize, 0.f, 0.f);
#endif
	objectObstacles.clear();
}

void Extension::ClearObstacles()
{
	sys.grid.ClearAll();
	objectObstacles.clear();
	sys.grid.Refresh();
}

///
/// OBSTACLES
///

#ifdef _WIN32
// Bounding rectangle of an object in frame coordinates.
static bool ObjectRect(RunHeader* rhPtr, HeaderObject* ho, float& left, float& top, float& right, float& bottom)
{
	// Rectangle A: position + hotspot + image size. Frame coordinates, always valid, ignores scale/angle.
	left = (float)(ho->get_X() - ho->get_ImgXSpot());
	top = (float)(ho->get_Y() - ho->get_ImgYSpot());
	right = left + (float)ho->get_ImgWidth();
	bottom = top + (float)ho->get_ImgHeight();

	// Rectangle B: hoRect is the on-screen display rectangle in WINDOW coordinates and does account for
	// scale/angle. Convert to frame coordinates and use it only if it agrees with A (same centre, within
	// the object's size); otherwise trust A.
	if (ho->hoRect.right > ho->hoRect.left && ho->hoRect.bottom > ho->hoRect.top)
	{
		const float bl = (float)(ho->hoRect.left + rhPtr->get_WindowX()), bt = (float)(ho->hoRect.top + rhPtr->get_WindowY());
		const float br = (float)(ho->hoRect.right + rhPtr->get_WindowX()), bb = (float)(ho->hoRect.bottom + rhPtr->get_WindowY());
		const float acx = (left + right) * 0.5f, acy = (top + bottom) * 0.5f;
		const float bcx = (bl + br) * 0.5f, bcy = (bt + bb) * 0.5f;
		const float tolX = (std::max)(right - left, br - bl), tolY = (std::max)(bottom - top, bb - bt);
		if (std::fabs(acx - bcx) <= tolX * 0.5f && std::fabs(acy - bcy) <= tolY * 0.5f)
		{
			left = bl; top = bt; right = br; bottom = bb;
		}
	}
	return right > left && bottom > top;
}
#endif

// Shared by Add / Remove. Fusion repeats an action once per selected object; we do the loop ourselves instead.
// Each object remembers which cells it covers (by Fixed value), so adding it again moves it and removing it is exact.
void Extension::ApplyObjectObstacles(bool add)
{
#ifdef _WIN32
	Runtime.CancelRepeatingObjectAction();

	// oil can be -1 if the object type has no instances
	const short oil = Runtime.GetOIListIndexFromObjectParam(0);

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
			continue;
		const FlowPath::CellRect cells = sys.grid.RectToCells(left, top, right, bottom);
		if (!cells.Valid())
			continue;
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
	ApplyObjectObstacles(true);
}

void Extension::RemoveObjectsAsObstacles(RunObject*)
{
	ApplyObjectObstacles(false);
}

// Copies Fusion's built-in obstacle collision mask (obstacle-type backdrops, quick backdrops, etc.)
// into the grid. A cell is blocked if any part of it overlaps an obstacle.
void Extension::ImportFusionObstacles(int layer)
{
#ifdef _WIN32
	constexpr int CM_TEST_OBSTACLE_PLAN = 0; // Fusion collision plane for "obstacle" (platform is 1)
	HeaderObject* const ho = rdPtr->get_rHo();
	FlowPath::Grid& g = sys.grid;
	const int cell = (int)g.cellSize;

	for (int cy = 0; cy < g.height; ++cy)
		for (int cx = 0; cx < g.width; ++cx)
		{
			const int px = (int)(g.originX + cx * g.cellSize);
			const int py = (int)(g.originY + cy * g.cellSize);
			// Observed in Fusion: the test returns NONZERO when the rectangle is free and 0 when it overlaps
			// an obstacle (the opposite of what the name suggests), so a zero result means blocked.
			if (callColMaskTestRect(ho, px, py, cell, cell, layer, CM_TEST_OBSTACLE_PLAN) == 0)
				g.MarkImported(cx, cy);
		}
	g.Refresh();
#else
	(void)layer;
#endif
}

void Extension::AddFusionObstacles()
{
	ImportFusionObstacles(-1);
}

void Extension::AddFusionObstaclesAtLayer(int layer)
{
	ImportFusionObstacles(layer);
}

///
/// PERFORMANCE
///

void Extension::SetCellsPerFrame(int cells)
{
	sys.cellsPerFrame = (std::size_t)(std::max)(cells, 0);
}
