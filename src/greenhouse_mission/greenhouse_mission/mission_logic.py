import math
 
 
def path_length(points):
    """Total length of a polyline given as a list of (x, y) tuples."""
    total = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        total += math.hypot(dx, dy)
    return total
 
 
def compute_leg(row, current_side):
    """
    Given a row dict ({'y': ..., 'west_x': ..., 'east_x': ...}) and which
    side the robot currently occupies, return the entry/exit x positions,
    the yaw to travel in, and which side the robot ends up on if this leg
    succeeds.
    """
    if current_side not in ('west', 'east'):
        raise ValueError(f"current_side must be 'west' or 'east', got {current_side!r}")
 
    entry_side = current_side
    exit_side = 'east' if entry_side == 'west' else 'west'
    entry_x = row['west_x'] if entry_side == 'west' else row['east_x']
    exit_x = row['east_x'] if entry_side == 'west' else row['west_x']
    yaw = 0.0 if entry_side == 'west' else math.pi
 
    return {
        'entry_x': entry_x,
        'entry_y': row['y'],
        'exit_x': exit_x,
        'exit_y': row['y'],
        'yaw': yaw,
        'entry_side': entry_side,
        'exit_side': exit_side,
    }
 
 
def next_side(entry_side, exit_side, succeeded):
    """
    The robot ends up on exit_side only if it genuinely crossed the row.
    If the leg was aborted (blocked, failed, canceled), it never crossed,
    so it's still on entry_side -- the next row must be entered from
    there too, not from the "expected" alternating side.
    """
    return exit_side if succeeded else entry_side
 
 
def compute_max_path_length(row_width, row_length):
    """
    Same formula used by scripts/generate_waypoints.py -- duplicated
    there intentionally, since that script must run standalone at dev
    time without importing the installed package. Kept here too so the
    formula itself is directly testable (see
    test_max_path_length_safety_property in test_mission_logic.py, which
    demonstrates this formula is NOT safe for all row_width/row_length/
    headland combinations -- only for reasonably narrow, long rows).
    """
    return row_width + math.hypot(row_width, row_length)