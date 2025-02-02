#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from math import cos, sin, sqrt, pi
import tf_transformations

def calculate_trailer_yaw(tractor_yaw, trailer_yaw, velocity, dt):
    rtr = 0.5625 # Distance between the hitch and the trailer's axle center

    yaw = trailer_yaw + ((velocity / rtr) * sin(tractor_yaw - trailer_yaw)) * dt # Forward Euler integration

    if tractor_yaw - trailer_yaw > 3.14159/4:
        yaw = tractor_yaw - 3.14159/4
    elif tractor_yaw - trailer_yaw < -3.14159/4:
        yaw = tractor_yaw + 3.14159/4

    return yaw

class TrailerJointStatePublisher(Node):
    def __init__(self):
        super().__init__('trailer_joint_state_publisher')

        self.time_now = self.get_clock().now().to_msg()
        # Create a publisher for JointState messages
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Create a TF broadcaster to publish static transform
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_state_and_tf)

        self.get_logger().info('Trailer Joint State Publisher started.')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tractor_yaw = 0.0
        self.trailer_yaw = 0.0
        self.tractor_pos = (0.0,0.0)
        self.cur_vel = 0.0

 

    def publish_joint_state_and_tf(self):
        now = self.get_clock().now().to_msg()
        dt = (now.sec + now.nanosec * 1e-9) - (self.time_now.sec + self.time_now.nanosec * 1e-9) 
        self.time_now = now

        self.trailer_yaw = calculate_trailer_yaw(self.tractor_yaw, self.trailer_yaw,self.cur_vel, dt)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['trailer_connector_joint']
        joint_state_msg.position = [self.trailer_yaw]  # Joint position (in radians)
        joint_state_msg.velocity = [0.0]  # Joint velocity
        joint_state_msg.effort = [0.0]    # Joint effort
        self.joint_state_pub.publish(joint_state_msg)

        left_wheel = JointState()
        left_wheel.header.stamp = self.get_clock().now().to_msg()
        left_wheel.name = ['trailer_wheel_lr_joint']
        left_wheel.position = [0.0]  # Joint position (in radians)
        left_wheel.velocity = [0.0]  # Joint velocity
        left_wheel.effort = [0.0]    # Joint effort
        self.joint_state_pub.publish(left_wheel)

        left_wheel = JointState()
        left_wheel.header.stamp = self.get_clock().now().to_msg()
        left_wheel.name = ['trailer_wheel_rr_joint']
        left_wheel.position = [0.0]  # Joint position (in radians)
        left_wheel.velocity = [0.0]  # Joint velocity
        left_wheel.effort = [0.0]    # Joint effort
        self.joint_state_pub.publish(left_wheel)
        

        try:
            m_to_bl_tf: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            trailer_tf = TransformStamped()
            trailer_left_wheel_tf = TransformStamped()
            trailer_right_wheel_tf = TransformStamped()

            new_pos = (m_to_bl_tf.transform.translation.x,m_to_bl_tf.transform.translation.y)
            
            dist = sqrt((new_pos[0] - self.tractor_pos[0])**2 + (new_pos[1] - self.tractor_pos[1])**2)
            self.tractor_pos = new_pos
            self.cur_vel = dist / dt
            m_to_bl_tf.header.stamp = self.get_clock().now().to_msg()
            m_to_bl_tf.header.frame_id = 'map'  # Parent frame
            m_to_bl_tf.child_frame_id = 'trailer_connector_link'  # Child frame

            yaw = tf_transformations.euler_from_quaternion([m_to_bl_tf.transform.rotation.x, m_to_bl_tf.transform.rotation.y, m_to_bl_tf.transform.rotation.z, m_to_bl_tf.transform.rotation.w])[2]
            rot = tf_transformations.quaternion_from_euler(0, 0, self.trailer_yaw)
            self.tractor_yaw = yaw


            trailer_tf.header.stamp = self.get_clock().now().to_msg()
            trailer_tf.header.frame_id = 'base_link'  # Parent frame
            trailer_tf.child_frame_id = 'trailer_connector_link'  # Child frame
            trailer_tf.transform.translation.x += -0.925/2  # Update with the actual XYZ position
            trailer_tf.transform.translation.y = 0
            trailer_tf.transform.translation.z += -0.065
            trailer_tf.transform.rotation.x = m_to_bl_tf.transform.rotation.x + rot[0]
            trailer_tf.transform.rotation.y = m_to_bl_tf.transform.rotation.y + rot[1]
            trailer_tf.transform.rotation.z = m_to_bl_tf.transform.rotation.z + rot[2]
            trailer_tf.transform.rotation.w = m_to_bl_tf.transform.rotation.w + rot[3]

            trailer_left_wheel_tf.header.stamp = self.get_clock().now().to_msg()
            trailer_left_wheel_tf.header.frame_id = 'trailer_connector_link'  # Parent frame
            trailer_left_wheel_tf.child_frame_id = 'trailer_wheel_lr_link'  # Child frame
            trailer_left_wheel_tf.transform.translation.x = trailer_tf.transform.translation.x - 4*0.1375
            trailer_left_wheel_tf.transform.translation.y = trailer_tf.transform.translation.y + 0.2 + 0.45/2
            trailer_left_wheel_tf.transform.translation.z = trailer_tf.transform.translation.z - 0.105 
            trailer_left_wheel_tf.transform.rotation.x = 0.0
            trailer_left_wheel_tf.transform.rotation.y = 0.0
            trailer_left_wheel_tf.transform.rotation.z = 0.0
            trailer_left_wheel_tf.transform.rotation.w = 0.5   


            trailer_right_wheel_tf.header.stamp = self.get_clock().now().to_msg()
            trailer_right_wheel_tf.header.frame_id = 'trailer_connector_link'  # Parent frame
            trailer_right_wheel_tf.child_frame_id = 'trailer_wheel_rr_link'  # Child frame
            trailer_right_wheel_tf.transform.translation.x = trailer_tf.transform.translation.x - 4*0.1375 
            trailer_right_wheel_tf.transform.translation.y = trailer_tf.transform.translation.y - 0.2 - 0.45/2
            trailer_right_wheel_tf.transform.translation.z = trailer_tf.transform.translation.z - 0.105  
            trailer_right_wheel_tf.transform.rotation.x = 0.0
            trailer_right_wheel_tf.transform.rotation.y = 0.0
            trailer_right_wheel_tf.transform.rotation.z = 0.0
            trailer_right_wheel_tf.transform.rotation.w = 0.5  


            self.tf_broadcaster.sendTransform(trailer_tf)
            self.tf_broadcaster.sendTransform(trailer_left_wheel_tf)
            self.tf_broadcaster.sendTransform(trailer_right_wheel_tf)

            # avg = 0.001 seconds
            aux = self.get_clock().now().to_msg()
            self.get_logger().info(f"dt: {aux.sec + aux.nanosec * 1e-9 - now.sec - now.nanosec * 1e-9}")

        except Exception as e:
            self.get_logger().error(f"Failed to post transform: {e}")

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
