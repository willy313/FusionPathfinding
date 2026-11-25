#pragma once
#include "DarkEdif.hpp"
#include "FlowField.hpp" // must be here, not only in Common.hpp: DarkEdif library files include Extension.hpp directly

class Extension final
{
public:
	// ======================================
	// Required variables + functions
	// Variables here must not be moved or swapped around or it can cause future issues
	// ======================================
	RunHeader* rhPtr;
	RunObjectMultiPlatPtr rdPtr;
#ifdef __ANDROID__
	global<jobject> javaExtPtr;
#elif defined(__APPLE__)
	void* const objCExtPtr;
#endif

	Edif::Runtime Runtime;

	static const int MinimumBuild = 254;
	static const int Version = 1;

	// Warning: OEFLAGS/OEPREFS cannot be freely modified when you have used them in MFAs.
	static constexpr OEFLAGS OEFLAGS = OEFLAGS::NEVER_KILL | OEFLAGS::NEVER_SLEEP;
	static constexpr OEPREFS OEPREFS = OEPREFS::NONE;
	// If OEFLAGS::WINDOW_PROC (otherwise you can delete)
	// static constexpr int WindowProcPriority = 100;
	// If OEFLAGS::TEXT (otherwise you can delete)
	// static constexpr TextCapacity TextCapacity = TextCapacity::None;

#ifdef _WIN32
	Extension(RunObject* const rdPtr, const EDITDATA* const edPtr, const CreateObjectInfo* const cobPtr);
#elif defined(__ANDROID__)
	Extension(const EDITDATA* const edPtr, const jobject javaExtPtr, const CreateObjectInfo* const cobPtr);
#else
	Extension(const EDITDATA* const edPtr, void* const objCExtPtr, const CreateObjectInfo* const cobPtr);
#endif
	~Extension();

	// ======================================
	// Extension data
	// ======================================

	DarkEdif::FusionDebugger FusionDebugger;

	// ======================================
	// Pathfinding data
	// ======================================
	FlowPath::System sys;
	bool pendingStartImport = false;   // import Fusion obstacles on the first Handle()
	// Objects added as obstacles: Fixed value -> the cells they cover, so they can be moved or removed exactly.
	std::unordered_map<int, FlowPath::CellRect> objectObstacles;

	// Helpers
	// object -> hotspot position, pathfinding radius and (optionally) its current direction 0-31 (-1 if unknown)
	bool Resolve(int fixedValue, float& x, float& y, float& radius, int* currentDirection = nullptr);
	void ApplyObjectObstacles(bool add);
	void ImportFusionObstacles(int layer);

	// Actions
	void CreateGrid(int width, int height, int cellSize, int originX, int originY);
	void CreateGridFitFrame(int cellSize);
	void ClearObstacles();
	void AddFusionObstacles();
	void AddFusionObstaclesAtLayer(int layer);
	void AddObjectsAsObstacles(RunObject* obj);
	void RemoveObjectsAsObstacles(RunObject* obj);
	void SetCellsPerFrame(int cells);

	// Conditions
	bool PositionIsBlocked(int x, int y);

	// Expressions
	float Direction(int fixedA, int fixedB, int directions);
	float DirectionAt(float ax, float ay, float bx, float by, float radius, int directions);
	float PathDistance(int fixedA, int fixedB);
	float PathDistanceAt(float ax, float ay, float bx, float by, float radius);
	int HasPath(int fixedA, int fixedB);
	int HasPathAt(float ax, float ay, float bx, float by, float radius);
	int IsBlocked(float x, float y);
	int GridWidth();
	int GridHeight();
	int CellSize();
	int BlockedCells();
	int CachedTargets();

	// Runs every tick of Fusion's runtime, can be toggled off and back on
	REFLAG Handle();

#if TEXT_OEFLAG_EXTENSION
	// Extension text struct. Required for text exts.
	DarkEdif::FontInfoMultiPlat font;
	void OnFontChanged(bool colorEdit, DarkEdif::Rect* rc);
#endif
#if DARKEDIF_DISPLAY_TYPE == DARKEDIF_DISPLAY_SIMPLE
	// Extension display surface ptr. Required for simple display exts.
	DarkEdif::Surface * surf = nullptr;
#elif DARKEDIF_DISPLAY_TYPE == DARKEDIF_DISPLAY_MANUAL
	void Display();
	void GetZoneInfos();
	DarkEdif::Surface * GetDisplaySurface();
	DarkEdif::CollisionMask * GetCollisionMask(std::uint32_t flags);
#endif

	// These are called if there's no function linked to an ID
	void UnlinkedAction(int ID);
	long UnlinkedCondition(int ID);
	long UnlinkedExpression(int ID);

#if PAUSABLE_EXTENSION
	// Called when Fusion runtime is pausing - not just the F3 pause dialog
	void FusionRuntimePaused();
	// Called when Fusion runtime is resuming after a pause
	void FusionRuntimeContinued();
#endif // PAUSABLE_EXTENSION
};
