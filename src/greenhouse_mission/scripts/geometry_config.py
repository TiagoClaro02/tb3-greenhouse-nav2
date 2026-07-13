"""
Single source of truth for the greenhouse field geometry.

Change NUM_CORRIDORS or ROW_WIDTH here and re-run generate_world.py +
generate_map.py — both stay in sync automatically. This is deliberately
built to survive a live requirement change (e.g. "rows are now 0.8m",
or "make it 4 rows") without touching generator code.

Interpretation note (open question, not yet resolved with the
interviewer): NUM_CORRIDORS here counts actual driving corridors.
The outermost corridors are bounded by the field's outer boundary
wall on one side and an interior divider on the other — i.e. the
boundary wall doubles as the edge of the outermost crop row. If the
intended reading is instead "N interior crop-row dividers with the
boundary walls being separate from the rows," set NUM_CORRIDORS to
one more than the number of distinct crop rows you want and treat
the outermost corridors as buffer strips. Ask before assuming.
"""

NUM_CORRIDORS = 3          # number of parallel 1m-wide driving corridors
ROW_WIDTH = 1.0            # meters, width of each corridor
DIVIDER_THICKNESS = 0.2    # meters, thickness of each interior divider wall
ROW_LENGTH = 6.0           # meters, length of each corridor (the "long" direction, x-axis)
HEADLAND = 1.5             # meters, open turning space at each end (x-axis)
BOUNDARY_THICKNESS = 0.2   # meters, thickness of the outer boundary walls
WALL_HEIGHT = 0.6          # meters

# --- Derived geometry (do not hand-edit below this line) ---

NUM_DIVIDERS = NUM_CORRIDORS - 1

# y-position (start, end) of each corridor, y=0 is the near edge of corridor 1
_corridor_bounds = []
_y = 0.0
for i in range(NUM_CORRIDORS):
    _corridor_bounds.append((_y, _y + ROW_WIDTH))
    _y += ROW_WIDTH
    if i < NUM_DIVIDERS:
        _y += DIVIDER_THICKNESS
CORRIDOR_BOUNDS = _corridor_bounds  # list of (y_start, y_end) per corridor

# y-center of each interior divider wall
DIVIDER_CENTERS = [
    CORRIDOR_BOUNDS[i][1] + DIVIDER_THICKNESS / 2.0
    for i in range(NUM_DIVIDERS)
]

FIELD_Y_MIN = 0.0
FIELD_Y_MAX = CORRIDOR_BOUNDS[-1][1]
FIELD_Y_CENTER = (FIELD_Y_MIN + FIELD_Y_MAX) / 2.0
FIELD_Y_SPAN = FIELD_Y_MAX - FIELD_Y_MIN

# Boundary walls sit flush against the outermost corridor edges (no slack —
# this was a bug in an earlier version; see generate_map.py history).
BOUNDARY_SOUTH_Y = FIELD_Y_MIN - BOUNDARY_THICKNESS / 2.0
BOUNDARY_NORTH_Y = FIELD_Y_MAX + BOUNDARY_THICKNESS / 2.0

FIELD_X_MIN = 0.0
FIELD_X_MAX = ROW_LENGTH
FIELD_X_CENTER = (FIELD_X_MIN + FIELD_X_MAX) / 2.0

BOUNDARY_WEST_X = FIELD_X_MIN - HEADLAND
BOUNDARY_EAST_X = FIELD_X_MAX + HEADLAND

# Default blockage: placed in the middle corridor (index NUM_CORRIDORS // 2),
# at the field's x-midpoint. Adjust per the specific test scenario.
#
# Sized so it's a genuine, impassable blockage -- not just an obstacle Nav2
# can navigate around. ROBOT_DIAMETER accounts for TurtleBot3 waffle's
# robot_radius (0.22m in nav2_params.yaml); the gap left on either side of
# the blockage (ROW_WIDTH - BLOCKAGE_WIDTH_ACROSS_ROW) must stay well under
# that diameter, or the planner will correctly find a way through and the
# mission node's skip-on-failure logic will never actually trigger.
ROBOT_DIAMETER = 0.44
BLOCKAGE_CORRIDOR_INDEX = NUM_CORRIDORS // 2
BLOCKAGE_Y_CENTER = sum(CORRIDOR_BOUNDS[BLOCKAGE_CORRIDOR_INDEX]) / 2.0
BLOCKAGE_X_CENTER = FIELD_X_CENTER
BLOCKAGE_WIDTH_ACROSS_ROW = ROW_WIDTH - 0.1  # leaves only a 0.1m gap -- well under ROBOT_DIAMETER
BLOCKAGE_LENGTH_ALONG_ROW = 0.3

# Partial obstacle: placed in row 0 (a different row than the full
# blockage), offset flush against one side rather than centered -- like
# debris resting against a divider wall, not filling the corridor. Leaves
# ROW_WIDTH - PARTIAL_OBSTACLE_WIDTH_ACROSS_ROW of clear width, comfortably
# more than ROBOT_DIAMETER, so the robot should navigate around it within
# the row rather than trigger the max_path_length_m abort.
PARTIAL_OBSTACLE_CORRIDOR_INDEX = 0
PARTIAL_OBSTACLE_WIDTH_ACROSS_ROW = 0.40  # was 0.35 -- tightened one deliberate step, leaves ~0.55m clear (0.11m slack beyond ROBOT_DIAMETER)
PARTIAL_OBSTACLE_LENGTH_ALONG_ROW = 0.3
PARTIAL_OBSTACLE_X_CENTER = FIELD_X_MIN + ROW_LENGTH * 0.4  # off-center, different position than the full blockage
_partial_row_bounds = CORRIDOR_BOUNDS[PARTIAL_OBSTACLE_CORRIDOR_INDEX]
PARTIAL_OBSTACLE_Y_CENTER = _partial_row_bounds[0] + PARTIAL_OBSTACLE_WIDTH_ACROSS_ROW / 2.0 + 0.05

# Entry blockage: placed right at the row entry (near the headland/corridor
# threshold) of the LAST row, rather than mid-row. Tests whether "stuck at
# row entry" is actually a blockage the mission node should detect and
# skip, versus a tuning problem with the entry transition itself (open
# headland -> suddenly-narrow corridor) when nothing is actually in the way.
ENTRY_BLOCKAGE_CORRIDOR_INDEX = NUM_CORRIDORS - 1
_entry_row_bounds = CORRIDOR_BOUNDS[ENTRY_BLOCKAGE_CORRIDOR_INDEX]
ENTRY_BLOCKAGE_Y_CENTER = sum(_entry_row_bounds) / 2.0
ENTRY_BLOCKAGE_WIDTH_ACROSS_ROW = ROW_WIDTH - 0.1  # full block, same margin as the main blockage
ENTRY_BLOCKAGE_LENGTH_ALONG_ROW = 0.3
ENTRY_BLOCKAGE_X_CENTER = FIELD_X_MIN + 0.5  # just inside the row entrance, not mid-row
