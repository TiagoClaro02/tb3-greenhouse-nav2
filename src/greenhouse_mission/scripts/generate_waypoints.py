#!/usr/bin/env python3
import os
import math
import yaml
import geometry_config as g

MARGIN = 0.3  # meters inside the field edge

def build_waypoints():
    rows = []
    for i in range(g.NUM_CORRIDORS):
        y_center = sum(g.CORRIDOR_BOUNDS[i]) / 2.0
        west_x = g.FIELD_X_MIN + MARGIN
        east_x = g.FIELD_X_MAX - MARGIN

        going_east = (i % 2 == 0)
        if going_east:
            entry_x, exit_x, yaw = west_x, east_x, 0.0
        else:
            entry_x, exit_x, yaw = east_x, west_x, math.pi

        rows.append({
            'id': i + 1,
            'waypoints': [
                {'x': round(entry_x, 3), 'y': round(y_center, 3), 'yaw': round(yaw, 5)},
                {'x': round(exit_x, 3), 'y': round(y_center, 3), 'yaw': round(yaw, 5)},
            ],
        })

    # Maximum acceptable planned path length (meters) for a single row
    # traversal. Above this, the planner is assumed to be detouring through
    # a neighboring row rather than actually driving this one (see
    # mission_node.py). Derived, not measured: ROW_WIDTH + hypot(ROW_WIDTH,
    # ROW_LENGTH) -- the width term gives room for a real partial-obstacle
    # swerve within the row; the hypotenuse term is a geometric upper bound
    # unrelated to a full cross-row-and-back detour, which is longer still.
    max_path_length_m = g.ROW_WIDTH + math.hypot(g.ROW_WIDTH, g.ROW_LENGTH)

    out_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'waypoints.yaml')
    with open(out_path, 'w') as f:
        yaml.dump(
            {'max_path_length_m': round(max_path_length_m, 3), 'rows': rows},
            f, default_flow_style=False, sort_keys=False,
        )
    print(f"Wrote {out_path} ({len(rows)} rows, max_path_length_m={max_path_length_m:.3f})")

if __name__ == '__main__':
    build_waypoints()
