
import math
import os
import time
 
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
import yaml
 
from greenhouse_mission.mission_logic import compute_leg, next_side, path_length
 
 
def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose
 
 
def load_mission_data(navigator):
    pkg_share = get_package_share_directory('greenhouse_mission')
    waypoints_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    with open(waypoints_path) as f:
        data = yaml.safe_load(f)
    return data['rows'], data['max_path_length_m'], data['spawn_x'], data['spawn_y']
 
 
class PlanMonitor:
    """Tracks the length of the most recently published global plan."""
 
    def __init__(self, navigator):
        self.latest_length = None
        self._sub = navigator.create_subscription(Path, '/plan', self._on_plan, 10)
 
    def _on_plan(self, msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.latest_length = path_length(points)
 
    def reset(self):
        self.latest_length = None
 
 
def drive_to(navigator, status_pub, row_id, total_rows, label, pose,
             monitor=None, max_path_length_m=None):
    status_pub.publish(String(data=f'row {row_id}/{total_rows}: driving to {label}'))
    if monitor is not None:
        monitor.reset()
    navigator.goToPose(pose)
 
    aborted_for_blockage = False
    while not navigator.isTaskComplete():
        if monitor is not None and monitor.latest_length is not None:
            if monitor.latest_length > max_path_length_m:
                navigator.get_logger().warn(
                    f'Row {row_id}/{total_rows}: live replan ({monitor.latest_length:.2f}m) '
                    f'exceeds max ({max_path_length_m:.2f}m) -- obstacle detected, aborting leg'
                )
                status_pub.publish(String(
                    data=f'row {row_id}/{total_rows}: blockage detected mid-drive, aborting'
                ))
                navigator.cancelTask()
                aborted_for_blockage = True
                break
        time.sleep(0.1)
 
    if aborted_for_blockage:
        while not navigator.isTaskComplete():
            time.sleep(0.05)
        return TaskResult.FAILED
 
    return navigator.getResult()
 
 
def run_row(navigator, status_pub, row, total_rows, max_path_length_m, monitor, current_side):
    row_id = row['id']
    leg = compute_leg(row, current_side)
 
    entry_pose = make_pose(navigator, leg['entry_x'], leg['entry_y'], leg['yaw'])
    exit_pose = make_pose(navigator, leg['exit_x'], leg['exit_y'], leg['yaw'])
 
    navigator.get_logger().info(f"Row {row_id}/{total_rows}: entering from {leg['entry_side']}")
 
    result = drive_to(
        navigator, status_pub, row_id, total_rows, 'entry', entry_pose,
        monitor=monitor, max_path_length_m=max_path_length_m,
    )
    if result != TaskResult.SUCCEEDED:
        navigator.get_logger().warn(f'Row {row_id}/{total_rows}: could not reach entry ({result.name})')
        return result, next_side(leg['entry_side'], leg['exit_side'], succeeded=False)
 
    result = drive_to(
        navigator, status_pub, row_id, total_rows, 'exit', exit_pose,
        monitor=monitor, max_path_length_m=max_path_length_m,
    )
 
    succeeded = (result == TaskResult.SUCCEEDED)
    if not succeeded:
        navigator.get_logger().warn(
            f"Row {row_id}/{total_rows}: did not complete crossing, "
            f"staying on {leg['entry_side']} side for the next row"
        )
 
    return result, next_side(leg['entry_side'], leg['exit_side'], succeeded)
 
 
def main():
    rclpy.init()
    navigator = BasicNavigator()
    status_pub = navigator.create_publisher(String, 'mission_status', 10)
    monitor = PlanMonitor(navigator)
 
    rows, max_path_length_m, spawn_x, spawn_y = load_mission_data(navigator)
    total_rows = len(rows)
 
    initial_pose = make_pose(navigator, spawn_x, spawn_y, 0.0)
    navigator.setInitialPose(initial_pose)
 
    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()
 
    navigator.get_logger().info(f'Max allowed path length per row: {max_path_length_m:.2f}m')
    completed, skipped = [], []
 
    # Must match the launch file's spawn x_pose/y_pose side (currently the
    # west headland, x_pose=-1.0) -- another manually-synced coupling, same
    # as the AMCL initial_pose <-> launch spawn pose pair.
    current_side = 'west'
 
    for row in rows:
        row_id = row['id']
        try:
            result, current_side = run_row(
                navigator, status_pub, row, total_rows, max_path_length_m, monitor, current_side
            )
        except Exception as exc:  # noqa: BLE001 -- mission must survive any single row's failure
            navigator.get_logger().error(f'Row {row_id} raised an exception: {exc}')
            status_pub.publish(String(data=f'row {row_id}/{total_rows}: EXCEPTION, skipping'))
            skipped.append(row_id)
            continue
 
        if result == TaskResult.SUCCEEDED:
            completed.append(row_id)
            navigator.get_logger().info(f'Row {row_id}/{total_rows}: complete')
            status_pub.publish(String(data=f'row {row_id}/{total_rows}: complete'))
        else:
            skipped.append(row_id)
            navigator.get_logger().warn(f'Row {row_id}/{total_rows}: {result.name}, skipping')
            status_pub.publish(String(data=f'row {row_id}/{total_rows}: {result.name}, skipping'))
 
    summary = f'Mission finished. Completed rows: {completed}. Skipped rows: {skipped}.'
    navigator.get_logger().info(summary)
    status_pub.publish(String(data=summary))
 
    navigator.lifecycleShutdown()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()