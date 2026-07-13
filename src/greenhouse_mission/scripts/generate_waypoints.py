#!/usr/bin/env python3
"""
Generates config/waypoints.yaml from geometry_config.py.

Unlike earlier versions of this file, direction is NOT baked in here.
Each row is stored as its west-side and east-side pose only; mission_node.py
decides at runtime which side to enter from, based on which side the robot
actually ended up on after the previous row (a row that completes normally
puts the robot on the opposite side; a row aborted due to a blockage
leaves the robot on the same side it entered from). This lets the mission
node dynamically re-route: if a row is skipped, the next row is entered
from wherever the robot currently is, instead of blindly continuing a
fixed alternating pattern that would force an unnecessary crossing to the
"expected" side.
"""
import os
import yaml
import geometry_config as g

MARGIN = 0.3  # meters inside the field edge

def build_waypoints():
    rows = []
    for i in range(g.NUM_CORRIDORS):
        y_center = sum(g.CORRIDOR_BOUNDS[i]) / 2.0
        rows.append({
            'id': i + 1,
            'y': round(y_center, 3),
            'west_x': round(g.FIELD_X_MIN + MARGIN, 3),
            'east_x': round(g.FIELD_X_MAX - MARGIN, 3),
        })

    max_path_length_m = g.ROW_WIDTH + (g.ROW_WIDTH ** 2 + g.ROW_LENGTH ** 2) ** 0.5

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
