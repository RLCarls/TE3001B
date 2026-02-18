import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pandas as pd
import sys

class GazeboReplayer(Node):
    def __init__(self, csv_path):
        super().__init__('gazebo_replayer')
        self.pub = self.create_publisher(JointTrajectory, '/model/so101/joint_trajectory', 10)
        self.df = pd.read_csv(csv_path)
        self.joints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
        self.send_trajectory()

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        
        for i, row in self.df.iterrows():
            point = JointTrajectoryPoint()
            point.positions = [float(row[j]) for j in self.joints if j in row]
            # Sincronización básica de tiempo
            dur = rclpy.duration.Duration(seconds=row['t'])
            point.time_from_start = dur.to_msg()
            msg.points.append(point)
        
        print("Enviando trayectoria a Gazebo...")
        self.pub.publish(msg)

def main():
    if len(sys.argv) < 2: return
    rclpy.init()
    node = GazeboReplayer(sys.argv[1])
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
