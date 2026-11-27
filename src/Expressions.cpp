#include "Common.hpp"

// Looks up an object by its Fixed value (Fixed("Enemy") in the expression editor).
// Gives its hotspot position, the radius pathfinding should use for it (its own size, scaled by the size percent)
// and, optionally, its current Fusion direction (0-31, -1 if unknown).
bool Extension::Resolve(int fixedValue, float& x, float& y, float& radius, int* currentDirection)
{
#ifdef _WIN32
	if (fixedValue == 0 || fixedValue == -1)
	{
		FlowLog::Warn("ResolveInvalid", "Fixed value %d is not an object: the object may not exist (yet), or Fixed(\"...\") was used on an object type without instances", fixedValue);
		return false;
	}
	auto ro = Runtime.RunObjPtrFromFixed(fixedValue);
	if (!ro)
	{
		FlowLog::Warn("ResolveGone", "Fixed value %d could not be turned into an object: it has been destroyed, or does not exist yet", fixedValue);
		return false;
	}
	HeaderObject* const ho = ro->get_rHo();
	x = (float)ho->get_X();
	y = (float)ho->get_Y();
	radius = PathRadius(ho, sizePercent);

	if (currentDirection)
	{
		// Fusion's own direction number for this object (0-31); the roc structure is null for objects without movement
		rCom* const roc = ro->get_roc();
		*currentDirection = roc ? (((roc->rcDir % 32) + 32) % 32) : -1;
	}
	return true;
#else
	(void)fixedValue; (void)x; (void)y; (void)radius; (void)currentDirection;
	return false;
#endif
}

// Collects every instance of the objects that keep their distance from each other (all instances, whatever is
// currently picked, including ones created since) into the crowd.
void Extension::RefreshAgents()
{
	FlowLog::Mark(FlowLog::C_RefreshAgents, (int)avoidGroups.size());
	sys.crowd.Clear();
#ifdef _WIN32
	for (const AvoidGroup& g : avoidGroups)
		for (auto pObj : DarkEdif::ObjectIterator(rhPtr, g.oil, DarkEdif::Selection::All))
		{
			if (!pObj)
				continue;
			HeaderObject* const ho = pObj->get_rHo();
			if (!ho)
				continue;
			FlowPath::Agent a;
			a.id = ho->GetFixedValue();
			a.x = (float)ho->get_X();
			a.y = (float)ho->get_Y();
			a.r = g.radius > 0.f ? g.radius : InscribedRadius(ho); // personal space around the hotspot
			sys.crowd.Add(a); // Crowd ignores unusable values (not finite, absurd sizes)
		}
#endif
	sys.crowd.Rebuild();
}

// ---- Direction -------------------------------------------------------------------------------------------------
// Every expression asks the field for the target's grid cell (created on first use and shared by everyone heading
// there), so the cost per call is constant however many objects ask.

float Extension::Direction(int fixedA, int fixedB, int directions)
{
	FlowLog::Mark(FlowLog::C_Direction, fixedA, fixedB, (float)directions);
	FP_TRY
	float ax, ay, ar, bx, by, br;
	int currentDir = -1;
	if (!Resolve(fixedA, ax, ay, ar, &currentDir) || !Resolve(fixedB, bx, by, br))
		return -1.f;

	FlowPath::Sample sample = sys.Query(ax, ay, bx, by, ar);
	// Keep distance from the other objects of the crowd (the object asking is excluded), without being pushed into
	// walls. A mover that has no path at all is still pushed apart from its neighbours, so they don't stack up.
	if (const FlowPath::Agent* self = sys.crowd.Find(fixedA))
	{
		const FlowPath::Vec2 push = sys.crowd.Separation(*self);
		if (!FlowPath::SeparateWithoutPath(sample, push))
			FlowPath::ApplyCrowd(sys.grid, ar, ax, ay, sample, push);
	}
	const float result = FlowPath::FormatDirection(sample, directions);
	if (result >= 0.f || currentDir < 0)
		return result;

	// No path, or already arrived: hand back the object's own current direction, so it keeps its heading
	// instead of receiving an invalid value. (Use HasPath / PathDistance if you need to react to these cases.)
	return (directions <= 1 || directions >= 360) ? (float)currentDir * 11.25f : (float)currentDir;
	FP_CATCH("Direction", -1.f)
}

float Extension::DirectionAt(float ax, float ay, float bx, float by, float radius, int directions)
{
	FlowLog::Mark(FlowLog::C_DirectionAt, (int)ax, (int)ay, bx, by);
	FP_TRY
	return FlowPath::FormatDirection(sys.Query(ax, ay, bx, by, radius), directions);
	FP_CATCH("DirectionAt", -1.f)
}

// ---- Distance and reachability -----------------------------------------------------------------------------------

float Extension::PathDistance(int fixedA, int fixedB)
{
	FlowLog::Mark(FlowLog::C_PathDistance, fixedA, fixedB);
	FP_TRY
	float ax, ay, ar, bx, by, br;
	if (!Resolve(fixedA, ax, ay, ar) || !Resolve(fixedB, bx, by, br))
		return -1.f;
	const FlowPath::Sample s = sys.Query(ax, ay, bx, by, ar);
	return s.reachable ? s.distance : -1.f;
	FP_CATCH("PathDistance", -1.f)
}

float Extension::PathDistanceAt(float ax, float ay, float bx, float by, float radius)
{
	FlowLog::Mark(FlowLog::C_PathDistanceAt, (int)ax, (int)ay, bx, by);
	FP_TRY
	const FlowPath::Sample s = sys.Query(ax, ay, bx, by, radius);
	return s.reachable ? s.distance : -1.f;
	FP_CATCH("PathDistanceAt", -1.f)
}

// 1 only for a clear path: one that exists and leads to the target itself (not just to the nearest spot the
// object can reach when the target stands in a wall or somewhere too narrow for it).
int Extension::HasPath(int fixedA, int fixedB)
{
	FlowLog::Mark(FlowLog::C_HasPath, fixedA, fixedB);
	FP_TRY
	float ax, ay, ar, bx, by, br;
	if (!Resolve(fixedA, ax, ay, ar) || !Resolve(fixedB, bx, by, br))
		return 0;
	return sys.Query(ax, ay, bx, by, ar).Clear() ? 1 : 0;
	FP_CATCH("HasPath", 0)
}

int Extension::HasPathAt(float ax, float ay, float bx, float by, float radius)
{
	FlowLog::Mark(FlowLog::C_HasPathAt, (int)ax, (int)ay, bx, by);
	FP_TRY
	return sys.Query(ax, ay, bx, by, radius).Clear() ? 1 : 0;
	FP_CATCH("HasPathAt", 0)
}

// ---- Grid information (handy for checking that the grid looks the way you expect) --------------------------------

int Extension::IsBlocked(float x, float y)
{
	FlowLog::Mark(FlowLog::C_IsBlocked, (int)x, (int)y);
	FP_TRY
	sys.grid.Refresh();
	return sys.grid.IsRawBlocked(sys.grid.CellX(x), sys.grid.CellY(y)) ? 1 : 0;
	FP_CATCH("IsBlocked", 0)
}

int Extension::GridWidth() { return sys.grid.width; }
int Extension::GridHeight() { return sys.grid.height; }
int Extension::CellSize() { return (int)sys.grid.cellSize; }

int Extension::BlockedCells()
{
	FP_TRY
	sys.grid.Refresh();
	int n = 0;
	for (std::uint8_t b : sys.grid.raw) n += b ? 1 : 0;
	return n;
	FP_CATCH("BlockedCells", 0)
}

int Extension::CachedTargets() { return (int)sys.fields.size(); }

// ---- Text ---------------------------------------------------------------------------------------------------------

// The last error the extension wrote to its log file (empty if there was none).
const TCHAR* Extension::LastError()
{
	return Runtime.CopyString(DarkEdif::UTF8ToTString(FlowLog::Logger::Get().LastError()).c_str());
}

// Where the log file is.
const TCHAR* Extension::LogPath()
{
	return Runtime.CopyString(DarkEdif::UTF8ToTString(FlowLog::Logger::Get().Path()).c_str());
}
