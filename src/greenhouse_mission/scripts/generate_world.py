#!/usr/bin/env python3

import os
import geometry_config as g

WALL_TEMPLATE = """
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{r} {gr} {b} 1</ambient><diffuse>{r2} {g2} {b2} 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

def wall(name, x, y, sx, sy, color=(0.5, 0.5, 0.5)):
    r, gr, b = color
    return WALL_TEMPLATE.format(
        name=name, x=x, y=y, z=g.WALL_HEIGHT / 2.0,
        sx=sx, sy=sy, sz=g.WALL_HEIGHT,
        r=r, gr=gr, b=b, r2=min(r + 0.1, 1), g2=min(gr + 0.1, 1), b2=min(b + 0.1, 1),
    )

def build_world():
    models = []
    for i, y_center in enumerate(g.DIVIDER_CENTERS):
        models.append(wall(
            f"divider_wall_{i+1}", g.FIELD_X_CENTER, y_center,
            g.ROW_LENGTH, g.DIVIDER_THICKNESS, color=(0.1, 0.5, 0.1),
        ))

    total_x_span = g.BOUNDARY_EAST_X - g.BOUNDARY_WEST_X
    models.append(wall("boundary_south", g.FIELD_X_CENTER, g.BOUNDARY_SOUTH_Y, total_x_span, g.BOUNDARY_THICKNESS))
    models.append(wall("boundary_north", g.FIELD_X_CENTER, g.BOUNDARY_NORTH_Y, total_x_span, g.BOUNDARY_THICKNESS))
    models.append(wall("boundary_west", g.BOUNDARY_WEST_X, g.FIELD_Y_CENTER, g.BOUNDARY_THICKNESS, g.FIELD_Y_SPAN))
    models.append(wall("boundary_east", g.BOUNDARY_EAST_X, g.FIELD_Y_CENTER, g.BOUNDARY_THICKNESS, g.FIELD_Y_SPAN))

    models.append(f"""
    <model name="row_blockage">
      <static>true</static>
      <pose>{g.BLOCKAGE_X_CENTER} {g.BLOCKAGE_Y_CENTER} 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{g.BLOCKAGE_LENGTH_ALONG_ROW} {g.BLOCKAGE_WIDTH_ACROSS_ROW} 0.5</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{g.BLOCKAGE_LENGTH_ALONG_ROW} {g.BLOCKAGE_WIDTH_ACROSS_ROW} 0.5</size></box></geometry>
          <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.9 0.15 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>
""")

#     models.append(f"""
#     <model name="partial_obstacle">
#       <static>true</static>
#       <pose>{g.PARTIAL_OBSTACLE_X_CENTER} {g.PARTIAL_OBSTACLE_Y_CENTER} 0.2 0 0 0</pose>
#       <link name="link">
#         <collision name="collision"><geometry><box><size>{g.PARTIAL_OBSTACLE_LENGTH_ALONG_ROW} {g.PARTIAL_OBSTACLE_WIDTH_ACROSS_ROW} 0.4</size></box></geometry></collision>
#         <visual name="visual"><geometry><box><size>{g.PARTIAL_OBSTACLE_LENGTH_ALONG_ROW} {g.PARTIAL_OBSTACLE_WIDTH_ACROSS_ROW} 0.4</size></box></geometry>
#           <material><ambient>0.85 0.55 0.1 1</ambient><diffuse>0.95 0.65 0.15 1</diffuse></material>
#         </visual>
#       </link>
#     </model>
# """)

    ground_size_x = total_x_span + 6
    ground_size_y = g.FIELD_Y_SPAN + 6

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="greenhouse">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>{g.FIELD_X_CENTER} {g.FIELD_Y_CENTER} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{ground_size_x} {ground_size_y}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{ground_size_x} {ground_size_y}</size></plane></geometry>
          <material><ambient>0.4 0.35 0.2 1</ambient><diffuse>0.5 0.45 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!--
      Generated from geometry_config.py - do not hand-edit corridor
      positions here. NUM_CORRIDORS={g.NUM_CORRIDORS}, ROW_WIDTH={g.ROW_WIDTH}m.
    -->
{''.join(models)}
  </world>
</sdf>
"""
    out_path = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'greenhouse.sdf')
    with open(out_path, 'w') as f:
        f.write(sdf)
    print(f"Wrote {out_path} ({g.NUM_CORRIDORS} corridors, {g.ROW_WIDTH}m wide)")

if __name__ == '__main__':
    build_world()
