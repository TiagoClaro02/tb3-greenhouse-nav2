
NUM_CORRIDORS = 5          # number of parallel 1m-wide driving corridors
ROW_WIDTH = 1.0            # meters, width of each corridor
DIVIDER_THICKNESS = 0.4    # meters, thickness of each interior divider wall
ROW_LENGTH = 6.0           # meters, length of each corridor (the "long" direction, x-axis)
HEADLAND = 1.5             # meters, open turning space at each end (x-axis)
BOUNDARY_THICKNESS = 0.2   # meters, thickness of the outer boundary walls
WALL_HEIGHT = 0.6          # meters

# --- Derived geometry ---

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

# Boundary walls sit flush against the outermost corridor edges
BOUNDARY_SOUTH_Y = FIELD_Y_MIN - BOUNDARY_THICKNESS / 2.0
BOUNDARY_NORTH_Y = FIELD_Y_MAX + BOUNDARY_THICKNESS / 2.0

FIELD_X_MIN = 0.0
FIELD_X_MAX = ROW_LENGTH
FIELD_X_CENTER = (FIELD_X_MIN + FIELD_X_MAX) / 2.0

BOUNDARY_WEST_X = FIELD_X_MIN - HEADLAND
BOUNDARY_EAST_X = FIELD_X_MAX + HEADLAND

# Default blockage: placed in the middle corridor (index NUM_CORRIDORS // 2),
BLOCKAGE_CORRIDOR_INDEX = NUM_CORRIDORS // 2
BLOCKAGE_Y_CENTER = sum(CORRIDOR_BOUNDS[BLOCKAGE_CORRIDOR_INDEX]) / 2.0
BLOCKAGE_X_CENTER = FIELD_X_CENTER
