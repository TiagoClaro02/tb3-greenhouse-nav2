#!/usr/bin/env python3

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

    # Safe spawn point: mid-headland on the west side, row 0's y-center.
    # Guaranteed inside the boundary wall regardless of HEADLAND's value
    # (as long as HEADLAND > 0), unlike a hardcoded literal which silently
    # breaks the moment HEADLAND changes (as it just did).
    spawn_x = g.FIELD_X_MIN - g.HEADLAND / 2.0
    spawn_y = sum(g.CORRIDOR_BOUNDS[0]) / 2.0

    out_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'waypoints.yaml')
    with open(out_path, 'w') as f:
        yaml.dump(
            {
                'max_path_length_m': round(max_path_length_m, 3),
                'spawn_x': round(spawn_x, 3),
                'spawn_y': round(spawn_y, 3),
                'rows': rows,
            },
            f, default_flow_style=False, sort_keys=False,
        )
    print(
        f"Wrote {out_path} ({len(rows)} rows, max_path_length_m={max_path_length_m:.3f}, "
        f"spawn=({spawn_x:.3f}, {spawn_y:.3f}))"
    )

if __name__ == '__main__':
    build_waypoints()
