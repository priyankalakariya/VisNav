import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import mujoco
import numpy as np
import math


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path('models/world.xml')
        self.data  = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=480, width=640)

        # Publishers
        self.cam_pub  = self.create_publisher(Image,     '/camera/image_raw', 10)
        self.odom_pub = self.create_publisher(Odometry,  '/odom',             10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan',             10)

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Obstacle velocities for Brownian motion
        self.obs_vel = np.zeros((3, 2))

        # Timers
        self.create_timer(0.01,      self.step_physics)
        self.create_timer(1.0/30.0,  self.publish_camera)
        self.create_timer(1.0/50.0,  self.publish_odom)
        self.create_timer(1.0/10.0,  self.publish_scan)

    def step_physics(self):
        sigma = 0.02
        eta = np.random.normal(0, sigma, (3, 2))
        self.obs_vel += eta
        self.obs_vel = np.clip(self.obs_vel, -0.5, 0.5)

        for i in range(3):
            obs_x_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, f'obs{i+1}_x')
            self.data.qvel[obs_x_id]     = self.obs_vel[i, 0]
            self.data.qvel[obs_x_id + 1] = self.obs_vel[i, 1]

        mujoco.mj_step(self.model, self.data)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        d = 0.4
        r = 0.05

        left_vel  = (v - w * d/2) / r
        right_vel = (v + w * d/2) / r

        self.data.ctrl[0] = left_vel
        self.data.ctrl[1] = right_vel

    def publish_camera(self):
        self.renderer.update_scene(self.data, camera='robot_cam')
        pixels = self.renderer.render()

        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.height   = 480
        msg.width    = 640
        msg.encoding = 'rgb8'
        msg.step     = 640 * 3
        msg.data     = pixels.flatten().tobytes()

        self.cam_pub.publish(msg)

    def publish_odom(self):
        x     = self.data.qpos[0]
        y     = self.data.qpos[1]
        theta = self.data.qpos[3]
        vx    = self.data.qvel[0]
        vy    = self.data.qvel[1]
        vth   = self.data.qvel[3]

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.orientation.z = math.sin(theta/2)
        msg.pose.pose.orientation.w = math.cos(theta/2)
        msg.twist.twist.linear.x    = vx
        msg.twist.twist.angular.z   = vth
        self.odom_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z    = math.sin(theta/2)
        t.transform.rotation.w    = math.cos(theta/2)
        self.tf_broadcaster.sendTransform(t)

    def publish_scan(self):
        distance = float(self.data.sensordata[0])

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min       = -math.pi/4
        msg.angle_max       =  math.pi/4
        msg.angle_increment =  math.pi/2
        msg.range_min       = 0.1
        msg.range_max       = 5.0
        msg.ranges          = [distance]

        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
