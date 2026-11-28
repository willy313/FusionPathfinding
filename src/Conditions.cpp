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

// True if every selected instance of the first object has a clear path to the second object (its first selected
// instance): a path exists and it leads to the target itself, not just to the nearest spot the object can reach.
// Each instance is judged with its own size. The picked objects are left as they are; to treat enemies one by one use
// a "For each object" loop, or an expression such as HasPath(Fixed("Enemy"), Fixed("Player")) inside an action, which
// is evaluated per instance.
bool Extension::PathBetweenObjectsIsClear(int objectListA, int objectListB)
{
	FlowLog::Mark(FlowLog::C_CondPathObjects, objectListA, objectListB);
	FP_TRY
#ifdef _WIN32
	HeaderObject* const target = FirstSelected(rhPtr, objectListB);
	if (!target)
		return false;
	const float bx = (float)target->get_X(), by = (float)target->get_Y();

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
	(void)objectListA; (void)objectListB;
	return false;
#endif
	FP_CATCH("PathBetweenObjectsIsClear", false)
}

// True if every selected instance of the first object has reached the second object: it is within `distance` pixels of
// it by path, or it belongs to a crowd clump that has formed around it (see Expressions.cpp, ReachedSet).
bool Extension::ObjectReachedDestination(int objectListA, int objectListB, int distance)
{
	FlowLog::Mark(FlowLog::C_CondReached, objectListA, objectListB, (float)distance);
	FP_TRY
#ifdef _WIN32
	HeaderObject* const target = FirstSelected(rhPtr, objectListB);
	if (!target)
		return false;
	const float bx = (float)target->get_X(), by = (float)target->get_Y();
	const int fixedB = target->GetFixedValue();

	bool any = false;
	for (auto pA : DarkEdif::ObjectIterator(rhPtr, (short)objectListA, DarkEdif::Selection::Implicit))
	{
		HeaderObject* const ho = pA ? pA->get_rHo() : nullptr;
		if (!ho)
			continue;
		any = true;
		if (!ReachedDestination(ho->GetFixedValue(), (float)ho->get_X(), (float)ho->get_Y(), PathRadius(ho, sizePercent), fixedB, bx, by, distance))
			return false;
	}
	return any;
#else
	(void)objectListA; (void)objectListB; (void)distance;
	return false;
#endif
	FP_CATCH("ObjectReachedDestination", false)
}

bool Extension::PathBetweenPositionsIsClear(int ax, int ay, int bx, int by, int radius)
{
	FlowLog::Mark(FlowLog::C_CondPathPositions, ax, ay, (float)bx, (float)by);
	FP_TRY
	return sys.Query((float)ax, (float)ay, (float)bx, (float)by, (float)(std::max)(radius, 0)).Clear();
	FP_CATCH("PathBetweenPositionsIsClear", false)
}
