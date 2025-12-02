#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys, select, termios, tty
import csv
import codecs
import os

class GoalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('get_point')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_stamped_cb,
            10)
        self.subscription  # prevent unused variable warning
        self.points = []

        # 尝试获取终端设置，如果失败则设为 None
        try:
            self.settings = termios.tcgetattr(sys.stdin)
            self.interactive = True
        except:
            self.settings = None
            self.interactive = False
            self.get_logger().warn("非交互式终端，键盘输入功能将不可用")

        self.get_logger().info("--- 航点采集器已启动 ---")
        self.get_logger().info("1. 切换至多点导航，在 RViz 中使用 'Nav2 Goal' 按钮点击地图。")
        self.get_logger().info("2. 每点击一次，坐标就会被记录。")
        if self.interactive:
            self.get_logger().info("3. 按 'S' 键来保存所有已记录的点到 CSV 文件。")
            self.get_logger().info("4. 按 'Ctrl+C' 退出程序。")
        else:
            self.get_logger().info("3. 发送 SIGTERM 或 Ctrl+C 退出程序。")

    def pose_stamped_cb(self, msg):
        
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gz = msg.pose.orientation.z  
        gw = msg.pose.orientation.w
        
        self.points.append([gx, gy, gz, gw])
        
        # 打印日志，确认点已添加
        self.get_logger().info(f'点 {len(self.points)} 已添加: x={gx:.2f}, y={gy:.2f}, z={gz:.2f}, w={gw:.2f}')

    def get_key(self):
        if not self.interactive or self.settings is None:
            return ''
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except:
            return ''

    def data_write_csv(self, file_name, datas):
        # 确保路径被正确解析
        expanded_path = os.path.expanduser(file_name)
        try:
            with codecs.open(expanded_path, 'w+', 'utf-8') as file_csv:
                writer = csv.writer(file_csv, delimiter=',', quoting=csv.QUOTE_MINIMAL)
                for data in datas:
                    writer.writerow(data)
            self.get_logger().info(f'成功将 {len(datas)} 个点写入到: {expanded_path}')
        except Exception as e:
            self.get_logger().error(f'写入 CSV 文件失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()


    csv_file_path = '/home/micu/catkin_ws/src/robot_nav2/config/output.csv'
    node.get_logger().info(f"航点将保存到: {csv_file_path}")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            key = node.get_key()
            if key == 's' or key == 'S':
                node.data_write_csv(csv_file_path, node.points)
                node.get_logger().info("点已保存！你可以继续添加新点，或按 Ctrl+C 退出。")
            if key == '\x03':  # Ctrl+C to exit
                break
    except KeyboardInterrupt:
        node.get_logger().info("检测到 Ctrl+C，正在关闭...")
    finally:
        if node.settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
                node.get_logger().info("终端设置已恢复。")
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()