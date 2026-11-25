#include "Common.hpp"

bool Extension::PositionIsBlocked(int x, int y)
{
	sys.grid.Refresh();
	return sys.grid.IsRawBlocked(sys.grid.CellX((float)x), sys.grid.CellY((float)y));
}
