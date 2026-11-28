#include "Common.hpp"

bool Extension::PositionIsBlocked(int x, int y)
{
	FlowLog::Mark(FlowLog::C_CondBlocked, x, y);
	FP_TRY
	sys.grid.Refresh();
	return sys.grid.IsRawBlocked(sys.grid.CellX((float)x), sys.grid.CellY((float)y));
	FP_CATCH("PositionIsBlocked", false)
}

#ifdef _WIN32
// The first selected instance of an object (an object parameter reaches a condition as an object-list number).
static HeaderObject* FirstSelected(RunHeader* rh, int objectList)
{
	for (auto p : DarkEdif::ObjectIterator(rh, (short)objectList, DarkEdif::Selection::Implicit))
		if (p && p->get_rHo())
			return p->get_rHo();
	return nullptr;
}
#endif

// The four object conditions share this. For every picked instance of the first object it asks either "has a clear
// path to the target" (a path exists and leads to the target itself, not just to the nearest spot the object can
// reach) or, with reachedMode, "has reached its destination" (within `distance` px of the target, or part of a crowd
// clump that has; see Expressions.cpp, ReachedSet). The target is the first picked instance of the second object.
// Each instance is judged with its own size. With `any` one instance is enough, otherwise every instance must pass.
// Stops at the first instance that decides the answer. The picked objects are left as they are.
bool Extension::EvaluateObjects(int objectListA, int objectListB, int distance, bool reachedMode, bool any)
{
#ifdef _WIN32
	HeaderObject* const target = FirstSelected(rhPtr, objectListB);
	if (!target)
		return false;
	const float bx = (float)target->get_X(), by = (float)target->get_Y();
	const int fixedB = target->GetFixedValue();

	bool seen = false;
	for (auto pA : DarkEdif::ObjectIterator(rhPtr, (short)objectListA, DarkEdif::Selection::Implicit))
	{
		HeaderObject* const ho = pA ? pA->get_rHo() : nullptr;
		if (!ho)
			continue;
		seen = true;
		const float ax = (float)ho->get_X(), ay = (float)ho->get_Y(), radius = PathRadius(ho, sizePercent);
		const bool ok = reachedMode ? ReachedDestination(ho->GetFixedValue(), ax, ay, radius, fixedB, bx, by, distance)
		                            : sys.Query(ax, ay, bx, by, radius).Clear();
		if (any && ok) return true;   // one is enough
		if (!any && !ok) return false; // one failure is enough
	}
	return seen && !any;              // all passed (and there was at least one), or nobody passed
#else
	(void)objectListA; (void)objectListB; (void)distance; (void)reachedMode; (void)any;
	return false;
#endif
}

// Every picked instance has a clear path to the target. (To react to enemies one by one see the README: store
// HasPath(...) in an alterable value and use Fusion's own condition on it, which really picks.)
bool Extension::PathBetweenObjectsIsClear(int objectListA, int objectListB)
{
	FlowLog::Mark(FlowLog::C_CondPathObjects, objectListA, objectListB, 0.f);
	FP_TRY
	return EvaluateObjects(objectListA, objectListB, 0, false, false);
	FP_CATCH("PathBetweenObjectsIsClear", false)
}

// At least one picked instance has a clear path to the target.
bool Extension::AnyPathBetweenObjectsIsClear(int objectListA, int objectListB)
{
	FlowLog::Mark(FlowLog::C_CondPathObjects, objectListA, objectListB, 1.f);
	FP_TRY
	return EvaluateObjects(objectListA, objectListB, 0, false, true);
	FP_CATCH("AnyPathBetweenObjectsIsClear", false)
}

// Every picked instance has reached the target.
bool Extension::ObjectReachedDestination(int objectListA, int objectListB, int distance)
{
	FlowLog::Mark(FlowLog::C_CondReached, objectListA, objectListB, 0.f);
	FP_TRY
	return EvaluateObjects(objectListA, objectListB, distance, true, false);
	FP_CATCH("ObjectReachedDestination", false)
}

// At least one picked instance has reached the target.
bool Extension::AnyObjectReachedDestination(int objectListA, int objectListB, int distance)
{
	FlowLog::Mark(FlowLog::C_CondReached, objectListA, objectListB, 1.f);
	FP_TRY
	return EvaluateObjects(objectListA, objectListB, distance, true, true);
	FP_CATCH("AnyObjectReachedDestination", false)
}

bool Extension::PathBetweenPositionsIsClear(int ax, int ay, int bx, int by, int radius)
{
	FlowLog::Mark(FlowLog::C_CondPathPositions, ax, ay, (float)bx, (float)by);
	FP_TRY
	return sys.Query((float)ax, (float)ay, (float)bx, (float)by, (float)(std::max)(radius, 0)).Clear();
	FP_CATCH("PathBetweenPositionsIsClear", false)
}
