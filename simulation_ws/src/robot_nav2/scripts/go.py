#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import csv
import os 


class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.get_logger().info('CSV 航点发布器已启动。')
        self.get_logger().info('等待 Nav2 的 "navigate_through_poses" 动作服务器...')

    def send_goal(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        # 等待服务器
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('"navigate_through_poses" 动作服务器未连接。即将退出。')
            rclpy.shutdown() 
            return

        self.get_logger().info('Action server is ready. Sending goal with %d waypoints.' % len(poses))
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation result: {0}'.format(result))
        self.get_logger().info('所有航点已完成！即将退出。')
        rclpy.shutdown() 

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance to next waypoint: {feedback.distance_remaining:.2f} meters')

def read_waypoints_from_csv(filename):

    expanded_path = os.path.expanduser(filename)
    
    waypoints = []
    if not os.path.exists(expanded_path):
        rclpy.logging.get_logger('read_waypoints_from_csv').error(f'CSV file not found at: {expanded_path}')
        return []
        
    with open(expanded_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) == 4:  # Ensure each row has x, y, z, w
                waypoints.append({
                    "x": float(row[0]),
                    "y": float(row[1]),
                    "z": float(row[2]),
                    "w": float(row[3])
                })
    return waypoints

def main(args=None):
    rclpy.init(args=args)


    client = NavThroughPosesClient()

    # 从CSV文件读取导航点
    csv_file_path = '/home/yahboom/simulation_ws/src/robot_nav2/config/output.csv'
    waypoints = read_waypoints_from_csv(csv_file_path)
    
    if not waypoints:
        client.get_logger().error('No waypoints loaded, shutting down.')
        client.destroy_node()
        rclpy.shutdown()
        return

    # 将导航点转换为 PoseStamped 消息
    poses = []
    for point in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = client.get_clock().now().to_msg() # 使用当前时间
        pose.pose.position.x = point["x"]
        pose.pose.position.y = point["y"]
        pose.pose.position.z = point["z"] # z 通常为 0.0
        pose.pose.orientation.w = point["w"] # 四元数
        poses.append(pose)

    client.send_goal(poses)
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()