#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from math import cos, sin, sqrt
import tf_transformations

def calculate_trailer_yaw(tractor_yaw, trailer_yaw, velocity, dt):
    rtr = 1.5

    yaw = trailer_yaw + (velocity / rtr * sin(tractor_yaw - trailer_yaw)) * dt

    return yaw

class TrailerJointStatePublisher(Node):
    def __init__(self):
        super().__init__('trailer_joint_state_publisher')

        self.time_now = self.get_clock().now().to_msg()
        # Create a publisher for JointState messages
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Create a TF broadcaster to publish static transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to periodically publish the joint state and TF
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_state_and_tf)

        self.get_logger().info('Trailer Joint State Publisher started.')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tractor_yaw = 0.0
        self.tractor_pos = (0.0, 0.0)

        self.trailer_yaw = 0.0

        self.cur_vel = 0.0

    def cmd_vel_callback(self, msg):
        self.cur_vel = sqrt( msg.linear.x**2 + msg.linear.y**2)

 

    def publish_joint_state_and_tf(self):
        now = self.get_clock().now().to_msg()
        dt = (now.sec + now.nanosec * 1e-9) - (self.time_now.sec + self.time_now.nanosec * 1e-9) 
        self.time_now = now

        self.trailer_yaw = calculate_trailer_yaw(self.tractor_yaw, self.trailer_yaw,self.cur_vel, dt)

        # Publish the joint state
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['trailer_connector_joint']
        joint_state_msg.position = [self.trailer_yaw]  # Joint position (in radians)
        joint_state_msg.velocity = [0.0]  # Joint velocity
        joint_state_msg.effort = [0.0]    # Joint effort
        self.joint_state_pub.publish(joint_state_msg)
        

        try:
            m_to_bl_tf: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            m_to_bl_tf.header.stamp = self.get_clock().now().to_msg()
            m_to_bl_tf.header.frame_id = 'map'  # Parent frame
            m_to_bl_tf.child_frame_id = 'trailer_connector_link'  # Child frame

            yaw = tf_transformations.euler_from_quaternion([m_to_bl_tf.transform.rotation.x, m_to_bl_tf.transform.rotation.y, m_to_bl_tf.transform.rotation.z, m_to_bl_tf.transform.rotation.w])[2]
            rot = tf_transformations.quaternion_from_euler(0, 0, self.trailer_yaw)
            self.tractor_yaw = yaw
            self.tractor_pos = (m_to_bl_tf.transform.translation.x, m_to_bl_tf.transform.translation.y)

            added_x = -0.5 * cos(yaw)
            added_y = -0.5 * sin(yaw)

            m_to_bl_tf.transform.translation.x += added_x  # Update with the actual XYZ position
            m_to_bl_tf.transform.translation.y += added_y

            m_to_bl_tf.transform.translation.z += -0.065

            m_to_bl_tf.transform.rotation.x = rot[0]
            m_to_bl_tf.transform.rotation.y = rot[1]
            m_to_bl_tf.transform.rotation.z = rot[2]
            m_to_bl_tf.transform.rotation.w = rot[3]



            self.tf_broadcaster.sendTransform(m_to_bl_tf)
        except Exception as e:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'  # Parent frame
            transform.child_frame_id = 'trailer_connector_link'  # Child frame
            transform.transform.translation.x = -0.5  # Update with the actual XYZ position
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = -0.065
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            self.get_logger().error(f"Failed to get transform: {str(e)}")

            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TrailerJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
