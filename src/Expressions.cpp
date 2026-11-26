#include "Common.hpp"


// Looks up an object by its Fixed value (Fixed("Enemy") in the expression editor).
// Gives its hotspot position and the radius pathfinding should use for it: the distance from the hotspot to the
// farthest corner of its image, so the whole body fits wherever the hotspot goes.
bool Extension::Resolve(int fixedValue, float& x, float& y, float& radius, int* currentDirection)
{
#ifdef _WIN32
	if (fixedValue == 0 || fixedValue == -1)
		return false;
	auto ro = Runtime.RunObjPtrFromFixed(fixedValue);
	if (!ro)
		return false;
	HeaderObject* const ho = ro->get_rHo();
	x = (float)ho->get_X();
	y = (float)ho->get_Y();

	radius = RadiusOfHeader(ho);

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
	sys.crowd.Clear();
#ifdef _WIN32
	for (const AvoidGroup& g : avoidGroups)
		for (auto pObj : DarkEdif::ObjectIterator(rhPtr, g.oil, DarkEdif::Selection::All))
		{
			HeaderObject* const ho = pObj->get_rHo();
			FlowPath::Agent a;
			a.id = ho->GetFixedValue();
			a.x = (float)ho->get_X();
			a.y = (float)ho->get_Y();
			a.r = g.radius > 0.f ? g.radius : InscribedRadius(ho); // personal space around the hotspot
			sys.crowd.Add(a);
		}
#endif
	sys.crowd.Rebuild();
}

// ---- Direction -------------------------------------------------------------------------------------------------
// Every expression asks the field for the target's grid cell (created on first use and shared by everyone heading
// there), so the cost per call is constant however many objects ask.

float Extension::Direction(int fixedA, int fixedB, int directions)
{
	float ax, ay, ar, bx, by, br;
	int currentDir = -1;
	if (!Resolve(fixedA, ax, ay, ar, &currentDir) || !Resolve(fixedB, bx, by, br))
		return -1.f;

	FlowPath::Sample sample = sys.Query(ax, ay, bx, by, ar);
	// Keep distance from the other objects of the crowd (the object asking is excluded), without being pushed into walls
	if (const FlowPath::Agent* self = sys.crowd.Find(fixedA))
		FlowPath::ApplyCrowd(sys.grid, ar, ax, ay, sample, sys.crowd.Separation(*self));
	const float result = FlowPath::FormatDirection(sample, directions);
	if (result >= 0.f || currentDir < 0)
		return result;

	// No path, or already arrived: hand back the object's own current direction, so it keeps its heading
	// instead of receiving an invalid value. (Use HasPath / PathDistance if you need to react to these cases.)
	return (directions <= 1 || directions >= 360) ? (float)currentDir * 11.25f : (float)currentDir;
}

float Extension::DirectionAt(float ax, float ay, float bx, float by, float radius, int directions)
{
	return FlowPath::FormatDirection(sys.Query(ax, ay, bx, by, radius), directions);
}

// ---- Distance and reachability -----------------------------------------------------------------------------------

float Extension::PathDistance(int fixedA, int fixedB)
{
	float ax, ay, ar, bx, by, br;
	if (!Resolve(fixedA, ax, ay, ar) || !Resolve(fixedB, bx, by, br))
		return -1.f;
	const FlowPath::Sample s = sys.Query(ax, ay, bx, by, ar);
	return s.reachable ? s.distance : -1.f;
}

float Extension::PathDistanceAt(float ax, float ay, float bx, float by, float radius)
{
	const FlowPath::Sample s = sys.Query(ax, ay, bx, by, radius);
	return s.reachable ? s.distance : -1.f;
}

int Extension::HasPath(int fixedA, int fixedB)
{
	float ax, ay, ar, bx, by, br;
	if (!Resolve(fixedA, ax, ay, ar) || !Resolve(fixedB, bx, by, br))
		return 0;
	return sys.Query(ax, ay, bx, by, ar).reachable ? 1 : 0;
}

int Extension::HasPathAt(float ax, float ay, float bx, float by, float radius)
{
	return sys.Query(ax, ay, bx, by, radius).reachable ? 1 : 0;
}

// ---- Grid information (handy for checking that the grid looks the way you expect) --------------------------------

int Extension::IsBlocked(float x, float y)
{
	sys.grid.Refresh();
	return sys.grid.IsRawBlocked(sys.grid.CellX(x), sys.grid.CellY(y)) ? 1 : 0;
}

int Extension::GridWidth() { return sys.grid.width; }
int Extension::GridHeight() { return sys.grid.height; }
int Extension::CellSize() { return (int)sys.grid.cellSize; }

int Extension::BlockedCells()
{
	sys.grid.Refresh();
	int n = 0;
	for (std::uint8_t b : sys.grid.raw) n += b ? 1 : 0;
	return n;
}

int Extension::CachedTargets() { return (int)sys.fields.size(); }
