# Flow Field Pathfinding for Clickteam Fusion

Top-down pathfinding for many objects at once. You add your walls, then ask **which direction an object should move to reach another object**. Everything heading for the same spot shares one calculation, so it stays fast with many instances. The answer is a smooth direction, not a jump between a few fixed angles.

## Get the extension
Actions tab -> latest "Build MFX" run -> Artifacts -> `FlowFieldPathfinding`. `INSTALL.txt` inside says where the two `.mfx` files go (close Fusion first).

## Quick start (Ball and other built-in movements)
Add one **Flow Field Pathfinding** object to the frame, then:

```
* Start of Frame
    Flow Field Pathfinding : Add Fusion obstacles to the grid (all layers)
    Flow Field Pathfinding : Add Wall as obstacles            <- only for walls that are active objects

* Always
    Enemy : Set direction to Direction("Flow Field Pathfinding", Fixed("Enemy"), Fixed("Player"), 32)
```

That is all. The extension reads each enemy's position and size by itself, so there is no padding to type in and no target ID. Put each moving object's **hotspot at its centre**.

`Direction(..., 32)` returns Fusion's own direction number (0-31), so it goes straight into **Set direction**. If there is no path, or the object has already arrived, it returns the object's own current direction, so the object keeps its heading.

### Directions argument
| Last argument | Returns |
|---|---|
| `4`, `8`, `16`, `32` | Fusion direction number on the 0-31 scale, snapped to that many directions (4 gives 0/8/16/24) |
| `360` | smooth angle in degrees, 0 = right, 90 = up, counter-clockwise |

For custom movements (Mover with an angle, physics, your own speed code) use `360`, or `DirectionAt` with positions.

## How it works
- **Grid:** the frame is divided into cells (16 px by default). Obstacles mark cells as blocked.
- **Fields on demand:** the first time somebody asks for a path to a spot, the extension builds one field for that spot and keeps it. Every object heading for the same cell with the same size reuses it. A target that moves to a new cell gets a new field (a handful are cached; unused ones are dropped).
- **Object size:** the radius comes from the moving object itself: the distance from its hotspot to the farthest corner of its image. Cells closer to a wall than that radius are blocked, and the area out to twice the radius costs more to cross, which keeps paths off walls and corners. Objects of different sizes get slightly different fields automatically. Gaps too narrow for an object are closed to it.
- **Cost:** a direction lookup is a constant-time read, however many objects ask.

## Obstacles
| Action | Use |
|---|---|
| Add Fusion obstacles to the grid (all layers) | Copies Fusion's own obstacle objects (backdrops set to Obstacle). Also available as an editor property ("Import Fusion obstacles on start") |
| Add Fusion obstacles on layer | Same, for one layer (untested in this version, test it first) |
| Add objects as obstacles | Active objects, by their current position and size. Add them again after they move. Removing is exact, even where objects overlap |
| Remove objects from obstacles | Takes them out again |
| Clear all obstacles | Starts over |

## Areas to avoid (soft obstacles)
Obstacles block. Areas to avoid only **cost extra**: units go round them when there is a reasonable way round, and cross them when there is not. Use them for hazards, a boss, a crowded spot, anything that is bad to walk through but not impossible.

| Action | Use |
|---|---|
| Add objects as areas to avoid (padding) | The object's rectangle plus the padding, in pixels, around it. Add the object again after it moves |
| Add position as an area to avoid (padding) | A circle of the given radius at a position |
| Remove objects from areas to avoid / Clear all areas to avoid | Take them away again |

Areas are part of the shared fields, so they suit things that do not move much. **Do not add the objects you are steering as areas to avoid** (the enemies themselves): each enemy would be pushed away from its own area. Use the next section for that.

## Objects keeping their distance from each other (crowds)
`Make Enemy keep their distance from each other, personal space radius 14 px` is set once. From then on every enemy instance (also ones created later) pushes its neighbours apart whenever `Direction(...)` is asked for it, and never pushes itself.

**Personal space is a circle around the object's centre (its hotspot)**, not an extra margin around its whole body. Two objects start pushing each other when their circles touch, and may overlap a little (about a third of the circle) when something pulls them together, such as chasing the same target. With a radius of `0` the largest circle that fits inside the object's image is used. A circle smaller than the object lets bodies overlap more; a larger one keeps them further apart.

The push never drives a unit into a wall: near walls the part of the direction that points at the wall is removed. `Stop all objects keeping their distance` switches it off. Only `Direction` (the Fixed-value version) includes the push.

Crowded corridors can still jam: avoidance nudges units apart, it does not plan routes for a queue.

## Debug images
Two actions write a transparent PNG that lines up with the frame (when the grid starts at 0,0), so you can see how the extension interprets your scene. Show it over the game with an **Active Picture** object: *Load picture from file*, then place it at 0,0 above your other layers.

| Action | Shows |
|---|---|
| Debug: save an image of the grid | Obstacles in red, areas to avoid in blue (darker = stronger), and the cell grid |
| Debug: save an image of the boundary of (object) | What objects of that size see: **orange** = cells their hotspot cannot enter, **yellow** = the zone where paths cost extra near walls, red and blue as above. For each selected object: a **cyan circle** for its pathfinding size, a **green circle** for its personal space (if it keeps its distance from others), its bounding box in dark red and its hotspot as a magenta cross |

The images are snapshots: call the action again to refresh it. Very large frames are shrunk so the picture stays small. Relative file names are relative to the application's folder.

## Expressions
| Expression | Meaning |
|---|---|
| `Direction(fixedA, fixedB, directions)` | Direction for object A to reach object B. `directions` is 4, 8, 16, 32 or 360 |
| `DirectionAt(ax, ay, bx, by, radius, directions)` | Same with positions and a radius you choose (no crowd avoidance); -1 when there is no path or it has arrived |
| `PathDistance(fixedA, fixedB)` / `PathDistanceAt(...)` | Path length in pixels (slightly above the true distance near walls); -1 if there is no path |
| `HasPath(fixedA, fixedB)` / `HasPathAt(...)` | 1 if a path exists, otherwise 0 |
| `IsBlocked(x, y)` | 1 if the position is inside an obstacle |
| `GridWidth`, `GridHeight`, `CellSize`, `BlockedCells`, `CachedTargets` | Information for checking your setup |

`Fixed("Enemy")` is Fusion's expression for an object's unique id, found under the Enemy object's expressions.

## Tips
- **Cell size:** 16 is a good start. Use 8 for tight spaces; larger cells are faster on huge frames.
- **Many targets:** if lots of different targets are used at once, raise the limit by editing `maxFields` in `FlowField.hpp` (default 16), or use the "Limit pathfinding work" action to spread work over frames.
- **Stutters:** "Limit pathfinding work to N cells per frame" spreads rebuilds over several frames (answers can then lag a little).
- **Walls from backdrops set to Obstacle do not need to be added again** unless the frame changes.

## Troubleshooting
Show these in a String or Counter object for the object you are testing:
- `HasPath("Flow Field Pathfinding", Fixed("Enemy"), Fixed("Player"))`: 0 means the extension sees no route (walls, or a gap too narrow for the object).
- `BlockedCells("Flow Field Pathfinding")`: roughly the number of cells your walls cover. Far too many or too few points at the obstacle import.
- `IsBlocked("Flow Field Pathfinding", X("Enemy"), Y("Enemy"))`: 1 means the enemy's hotspot is inside an obstacle.
- Enemy still touches walls: save a boundary image for it (see Debug images) and check that the cyan circle covers the whole body. If not, the hotspot is probably off-centre or the image size does not match the object. Use `DirectionAt` with a larger radius for a bigger margin.

## For developers
- The core is plain C++ in `src/FlowField.hpp`. `test/test_flowfield.cpp` and `test/test_body.cpp` run in the "core-tests" CI job (or locally: `g++ -std=c++17 test/test_flowfield.cpp`). `test_body.cpp` simulates a round-bodied unit that stops when it touches a wall, like a Fusion sprite, and checks it always reaches its target.
- `python test/check_signatures.py` compares `DarkExt.json` with the C++ declarations; it runs first in the build.
- Do not add, rename or remove editor properties once the object has been used in a project (existing objects break). Add behaviour as actions or expressions instead.
- Keep the identifier (`FLWF`) stable, and append new actions, conditions and expressions at the end so existing projects keep working.
