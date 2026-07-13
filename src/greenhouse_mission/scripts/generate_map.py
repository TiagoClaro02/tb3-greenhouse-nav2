#!/usr/bin/env python3

from PIL import Image
import yaml
import os
import geometry_config as g

RESOLUTION = 0.05  # meters/pixel
MARGIN = 1.0       # extra free space drawn around the field, meters

X_MIN, X_MAX = g.BOUNDARY_WEST_X - MARGIN, g.BOUNDARY_EAST_X + MARGIN
Y_MIN, Y_MAX = g.BOUNDARY_SOUTH_Y - MARGIN, g.BOUNDARY_NORTH_Y + MARGIN

WIDTH_PX = int((X_MAX - X_MIN) / RESOLUTION)
HEIGHT_PX = int((Y_MAX - Y_MIN) / RESOLUTION)

FREE, OCC = 254, 0

def world_to_px(x, y):
    return int((x - X_MIN) / RESOLUTION), int((Y_MAX - y) / RESOLUTION)

def fill_rect(img, cx, cy, sx, sy, value):
    x0, y0, x1, y1 = cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2
    c0, r0 = world_to_px(x0, y1)
    c1, r1 = world_to_px(x1, y0)
    for row in range(max(r0, 0), min(r1, HEIGHT_PX)):
        for col in range(max(c0, 0), min(c1, WIDTH_PX)):
            img.putpixel((col, row), value)

def main():
    img = Image.new('L', (WIDTH_PX, HEIGHT_PX), FREE)

    for y_center in g.DIVIDER_CENTERS:
        fill_rect(img, g.FIELD_X_CENTER, y_center, g.ROW_LENGTH, g.DIVIDER_THICKNESS, OCC)

    total_x_span = g.BOUNDARY_EAST_X - g.BOUNDARY_WEST_X
    fill_rect(img, g.FIELD_X_CENTER, g.BOUNDARY_SOUTH_Y, total_x_span, g.BOUNDARY_THICKNESS, OCC)
    fill_rect(img, g.FIELD_X_CENTER, g.BOUNDARY_NORTH_Y, total_x_span, g.BOUNDARY_THICKNESS, OCC)
    fill_rect(img, g.BOUNDARY_WEST_X, g.FIELD_Y_CENTER, g.BOUNDARY_THICKNESS, g.FIELD_Y_SPAN, OCC)
    fill_rect(img, g.BOUNDARY_EAST_X, g.FIELD_Y_CENTER, g.BOUNDARY_THICKNESS, g.FIELD_Y_SPAN, OCC)
    # NOTE: row_blockage is intentionally NOT baked in - it must be an
    # unexpected obstacle discovered at runtime, not known to AMCL/planner.

    out_dir = os.path.join(os.path.dirname(__file__), '..', 'maps')
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, 'greenhouse_map.pgm')
    yaml_path = os.path.join(out_dir, 'greenhouse_map.yaml')
    img.save(pgm_path)
    with open(yaml_path, 'w') as f:
        yaml.dump({
            'image': 'greenhouse_map.pgm',
            'resolution': RESOLUTION,
            'origin': [X_MIN, Y_MIN, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }, f, default_flow_style=False)
    print(f"Wrote {pgm_path} ({WIDTH_PX}x{HEIGHT_PX}px)")
    print(f"Wrote {yaml_path}")

if __name__ == '__main__':
    main()
