#include "Common.hpp"
#include <iomanip>
using namespace std::chrono_literals;

///
/// EXTENSION CONSTRUCTOR/DESTRUCTOR
///

#ifdef _WIN32
Extension::Extension(RunObject* const _rdPtr, const EDITDATA* const edPtr, const CreateObjectInfo* const cobPtr) :
	rdPtr(_rdPtr), rhPtr(_rdPtr->get_rHo()->get_AdRunHeader()), Runtime(this), FusionDebugger(this)
#elif defined(__ANDROID__)
Extension::Extension(const EDITDATA* const edPtr, const jobject javaExtPtr, const CreateObjectInfo* const cobPtr) :
	javaExtPtr(javaExtPtr, "Extension::javaExtPtr from Extension ctor"),
	Runtime(this, this->javaExtPtr), FusionDebugger(this)
#else
Extension::Extension(const EDITDATA* const edPtr, void* const objCExtPtr, const CreateObjectInfo* const cobPtr) :
	objCExtPtr(objCExtPtr), Runtime(this, objCExtPtr), FusionDebugger(this)
#endif
{
	LinkAction(0, CreateGrid);
	LinkAction(1, CreateGridFitFrame);
	LinkAction(2, ClearObstacles);
	LinkAction(3, AddFusionObstacles);
	LinkAction(4, AddFusionObstaclesAtLayer);
	LinkAction(5, AddObjectsAsObstacles);
	LinkAction(6, RemoveObjectsAsObstacles);
	LinkAction(7, SetCellsPerFrame);

	LinkCondition(0, PositionIsBlocked);

	LinkExpression(0, Direction);
	LinkExpression(1, DirectionAt);
	LinkExpression(2, PathDistance);
	LinkExpression(3, PathDistanceAt);
	LinkExpression(4, HasPath);
	LinkExpression(5, HasPathAt);
	LinkExpression(6, IsBlocked);
	LinkExpression(7, GridWidth);
	LinkExpression(8, GridHeight);
	LinkExpression(9, CellSize);
	LinkExpression(10, BlockedCells);
	LinkExpression(11, CachedTargets);

	// Properties set in the Fusion editor
	const int cell = (std::max)(2, (int)edPtr->Props.GetPropertyNum("Cell size (pixels)"sv));
	int gw = (int)edPtr->Props.GetPropertyNum("Grid width (cells, 0 = fit frame)"sv);
	int gh = (int)edPtr->Props.GetPropertyNum("Grid height (cells, 0 = fit frame)"sv);
	sys.cellsPerFrame = (std::size_t)(std::max)(0, (int)edPtr->Props.GetPropertyNum("Cells processed per frame (0 = unlimited)"sv));
	pendingStartImport = edPtr->Props.IsPropChecked("Import Fusion obstacles on start"sv);

#ifdef _WIN32
	// Frame size in pixels, used when width/height are 0 ("fit frame")
	if (gw <= 0) gw = (rhPtr->rhLevelSx + cell - 1) / cell;
	if (gh <= 0) gh = (rhPtr->rhLevelSy + cell - 1) / cell;
#else
	if (gw <= 0) gw = 64;
	if (gh <= 0) gh = 64;
#endif
	sys.ResizeGrid(gw, gh, (float)cell, 0.f, 0.f);
}

Extension::~Extension()
{

}

// Runs every tick of Fusion's runtime
REFLAG Extension::Handle()
{
	if (pendingStartImport)
	{
		pendingStartImport = false;
		ImportFusionObstacles(-1);
	}
	// Ages the field cache and continues any unfinished builds within the per-frame budget
	sys.Update();
	return REFLAG::NONE;
}

#if PAUSABLE_EXTENSION
// Called when Fusion runtime is pausing - not just the F3 pause dialog
void Extension::FusionRuntimePaused()
{

}

// Called when Fusion runtime is resuming after a pause
void Extension::FusionRuntimeContinued()
{

}
#endif // PAUSABLE_EXTENSION

#if TEXT_OEFLAG_EXTENSION
/* Triggered when the runtime changes the font name or size via built-in actions
 * @param colorEdit If true, font color was edited. If false, the font typeface was.
 * @param rect		If not null, contains a hint as to what size the object should be.
 *					This is based on differences between original font size and current one.
 *					You can ignore this hint, but it indicates you may want to call
 *					rdPtr->get_roc()->SetSize(). */
void Extension::OnFontChanged(bool colorEdit, DarkEdif::Rect * rect)
{
	// Prevent warnings about unused variables
	(void)colorEdit;
	(void)rect;

	// We could call Runtime.Redraw() to redraw ext immediately,
	// but we'll be patient and wait for next event tick
	rdPtr->get_roc()->set_changed(true);
	Runtime.Rehandle();
}

#endif

#if DARKEDIF_DISPLAY_TYPE == DARKEDIF_DISPLAY_MANUAL

void Extension::Display()
{
	// You should manually draw the surface(s) here.
	// Surface::BlitToFrameWithExtEffects will let you draw with the ext's sprites/X/Y accounted for,
	// but note you should also manage the collision mask in GetCollisionMask().
	// If you have a single surface, and don't want simple drawing, use the GetDisplaySurface().
	//
	//surf->BlitToFrameWithExtEffects(this, Point { x offset, y offset });
}


void Extension::GetZoneInfos()
{
	LOGI(_T("##### Extension::GetZoneInfos() called.\n"));
	// You can safely do nothing here. Some extensions do:
	// rdPtr->get_rHo()->set_ImgWidth()/set_ImgHeight()
	// but it is done automatically in CF2.5, so shouldn't be necessary.
}

DarkEdif::Surface * Extension::GetDisplaySurface()
{
	// Old style MMF2 fade-in/fade-out transistions requires this function,
	// Auto-rendering of Sprite effects (alpha coeff etc) will be applied by returning a surface here.
	// If you have multiple surfaces, you should can return null here and draw them yourself.
	// If you return null here, Display() will be called instead.
	return nullptr;
}
DarkEdif::CollisionMask * Extension::GetCollisionMask(std::uint32_t flags)
{
	// If your ext supports fine collision, you should return the collision mask corresponding to
	// which pixels have collision and which don't. DarkEdif::Surface can generate collision masks,
	// but it can be computationally expensive.
	// If your ext uses only box collision, DE will detect that and never call this function.
	(void)flags;

	return nullptr;
}

#endif // DARKEDIF_DISPLAY_MANUAL

// These are called if there's no function linked to an ID

void Extension::UnlinkedAction(int ID)
{
	DarkEdif::MsgBox::Error(_T("Extension::UnlinkedAction() called"), _T("Running a fallback for action ID %d. Make sure you ran LinkAction()."), ID);
}

long Extension::UnlinkedCondition(int ID)
{
	DarkEdif::MsgBox::Error(_T("Extension::UnlinkedCondition() called"), _T("Running a fallback for condition ID %d. Make sure you ran LinkCondition()."), ID);
	return 0;
}

long Extension::UnlinkedExpression(int ID)
{
	DarkEdif::MsgBox::Error(_T("Extension::UnlinkedExpression() called"), _T("Running a fallback for expression ID %d. Make sure you ran LinkExpression()."), ID);
	// Unlinked A/C/E is fatal error, but try not to return null string and definitely crash it
	if ((std::size_t)ID < Edif::SDK->ExpressionInfos.size() && Edif::SDK->ExpressionInfos[ID]->Flags.ef == ExpReturnType::String)
		return (long)Runtime.CopyString(_T(""));
	return 0;
}
