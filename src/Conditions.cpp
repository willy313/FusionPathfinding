#include "Common.hpp"

bool Extension::PositionIsBlocked(int x, int y)
{
	FlowLog::Mark(FlowLog::C_CondBlocked, x, y);
	FP_TRY
	sys.grid.Refresh();
	return sys.grid.IsRawBlocked(sys.grid.CellX((float)x), sys.grid.CellY((float)y));
	FP_CATCH("PositionIsBlocked", false)
}

// True if every selected instance of the object has a clear path to the target: a path exists and it leads to the
// target itself, not just to the nearest spot the object can reach. The target is given by its Fixed value, so it
// can be any specific object. Each instance is judged with its own size. The picked objects are left as they are;
// to treat enemies one by one use a "For each object" loop, or an expression such as
// HasPath(Fixed("Enemy"), Fixed("Player")) inside an action, which is evaluated per instance.
// (An Object parameter reaches a condition as an object-list number, not as an object.)
bool Extension::PathBetweenObjectsIsClear(int objectListA, int fixedB)
{
	FlowLog::Mark(FlowLog::C_CondPathObjects, objectListA, fixedB);
	FP_TRY
#ifdef _WIN32
	float bx, by, br;
	if (!Resolve(fixedB, bx, by, br))
		return false;

	bool any = false;
	for (auto pA : DarkEdif::ObjectIterator(rhPtr, (short)objectListA, DarkEdif::Selection::Implicit))
	{
		HeaderObject* const ho = pA ? pA->get_rHo() : nullptr;
		if (!ho)
			continue;
		any = true;
		if (!sys.Query((float)ho->get_X(), (float)ho->get_Y(), bx, by, PathRadius(ho, sizePercent)).Clear())
			return false;
	}
	return any;
#else
	(void)objectListA; (void)fixedB;
	return false;
#endif
	FP_CATCH("PathBetweenObjectsIsClear", false)
}

bool Extension::PathBetweenPositionsIsClear(int ax, int ay, int bx, int by, int radius)
{
	FlowLog::Mark(FlowLog::C_CondPathPositions, ax, ay, (float)bx, (float)by);
	FP_TRY
	return sys.Query((float)ax, (float)ay, (float)bx, (float)by, (float)(std::max)(radius, 0)).Clear();
	FP_CATCH("PathBetweenPositionsIsClear", false)
}
