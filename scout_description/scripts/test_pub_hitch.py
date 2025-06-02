#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster, TransformListener, Buffer
from math import cos, sin, sqrt, pi, atan2
import tf_transformations

def calculate_trailer_yaw(self,tractor_yaw, trailer_yaw, velocity, dt):
    rtr = 0.5625 # Distance between the hitch and the trailer's axle center

    # Calculate new yaw based on velocity and current angles
    yaw = trailer_yaw + ((velocity / rtr) * sin(tractor_yaw - trailer_yaw)) * dt

    while yaw > 2*pi:
        yaw -= 2 * pi
    while yaw < 0:
        yaw += 2 * pi

    angle_diff = -yaw + tractor_yaw
    while angle_diff > pi:
        angle_diff -= 2 * pi
    while angle_diff < -pi:
        angle_diff += 2 * pi

    # If angle difference exceeds ±45 degrees, lock the trailer at the maximum allowed angle
    if abs(angle_diff) > pi/4:
        yaw = tractor_yaw - (pi/4 if angle_diff > 0 else -pi/4)
        self.get_logger().info(f"Trailer yaw locked at {yaw}")

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
        self.trailer_yaw = -90.0
        self.tractor_pos = (0.0,0.0)
        self.cur_vel = 0.0
        self.prev_pos = None

        self.sent_exception = False

    def publish_joint_state_and_tf(self):
        now = self.get_clock().now().to_msg()
        dt = (now.sec + now.nanosec * 1e-9) - (self.time_now.sec + self.time_now.nanosec * 1e-9) 
        self.time_now = now

        try:
            m_to_bl_tf: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            if self.trailer_yaw != -90.0:   
                self.trailer_yaw = calculate_trailer_yaw(self,self.tractor_yaw, self.trailer_yaw, self.cur_vel, dt)
            else:
                self.trailer_yaw = tf_transformations.euler_from_quaternion([m_to_bl_tf.transform.rotation.x, m_to_bl_tf.transform.rotation.y, m_to_bl_tf.transform.rotation.z, m_to_bl_tf.transform.rotation.w])[2]
                self.get_logger().info("WHAT DI HEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEL")     
                self.get_logger().info(f"{self.trailer_yaw}")                

            
            new_pos = (m_to_bl_tf.transform.translation.x, m_to_bl_tf.transform.translation.y)
            
            if self.prev_pos is not None:
                # Calculate distance moved
                dist = sqrt((new_pos[0] - self.prev_pos[0])**2 + (new_pos[1] - self.prev_pos[1])**2)
                
                # Calculate movement direction relative to tractor yaw
                movement_angle = atan2(new_pos[1] - self.prev_pos[1], new_pos[0] - self.prev_pos[0])
                angle_diff = movement_angle - self.tractor_yaw
                
                # Normalize angle difference to [-pi, pi]
                while angle_diff > pi:
                    angle_diff -= 2 * pi
                while angle_diff < -pi:
                    angle_diff += 2 * pi
                
                # If angle difference is > pi/2 or < -pi/2, we're moving backwards
                self.cur_vel = dist / dt * (-1 if abs(angle_diff) > pi/2 else 1)
            
            self.prev_pos = new_pos
            self.tractor_pos = new_pos

            yaw = tf_transformations.euler_from_quaternion([m_to_bl_tf.transform.rotation.x, m_to_bl_tf.transform.rotation.y, m_to_bl_tf.transform.rotation.z, m_to_bl_tf.transform.rotation.w])[2]
            rot = tf_transformations.quaternion_from_euler(0, 0, self.trailer_yaw)
            self.tractor_yaw = yaw

            trailer_tf = TransformStamped()
            trailer_link_tf = TransformStamped()
            trailer_left_wheel_tf = TransformStamped()
            trailer_right_wheel_tf = TransformStamped()

            trailer_tf.header.stamp = self.get_clock().now().to_msg()
            trailer_tf.header.frame_id = 'map'  # Parent frame
            trailer_tf.child_frame_id = 'trailer_connector_link'  # Child frame

            x_mov = -(0.9250000/2) * cos(self.tractor_yaw)
            y_mov = -(0.9250000/2) * sin(self.tractor_yaw)

            trailer_tf.transform.translation.x = m_to_bl_tf.transform.translation.x + x_mov
            trailer_tf.transform.translation.y = m_to_bl_tf.transform.translation.y + y_mov
            trailer_tf.transform.translation.z = m_to_bl_tf.transform.translation.z + -0.065
            trailer_tf.transform.rotation.x = rot[0] 
            trailer_tf.transform.rotation.y = rot[1]
            trailer_tf.transform.rotation.z = rot[2]
            trailer_tf.transform.rotation.w = rot[3]


            trailer_link_tf.header.stamp = self.get_clock().now().to_msg()
            trailer_link_tf.header.frame_id = 'trailer_connector_link'  # Parent frame
            trailer_link_tf.child_frame_id = 'trailer_link'  # Child frame
            trailer_link_tf.transform.translation.x = -0.15
            trailer_link_tf.transform.translation.y = 0.0
            trailer_link_tf.transform.translation.z = 0.0
            trailer_link_tf.transform.rotation.x = 0.0
            trailer_link_tf.transform.rotation.y = 0.0
            trailer_link_tf.transform.rotation.z = 0.0
            trailer_link_tf.transform.rotation.w = 1.0



            trailer_left_wheel_tf.header.stamp = self.get_clock().now().to_msg()
            trailer_left_wheel_tf.header.frame_id = 'trailer_link'  # Parent frame
            trailer_left_wheel_tf.child_frame_id = 'trailer_wheel_lr_link'  # Child frame
            trailer_left_wheel_tf.transform.translation.x = -0.55/4 - 0.55/2
            trailer_left_wheel_tf.transform.translation.y = -(0.4/2 + 0.045/2) 
            trailer_left_wheel_tf.transform.translation.z = -0.105 
            trailer_left_wheel_tf.transform.rotation.x = 0.0
            trailer_left_wheel_tf.transform.rotation.y = 0.0
            trailer_left_wheel_tf.transform.rotation.z = 0.0
            trailer_left_wheel_tf.transform.rotation.w = 1.0   



            trailer_right_wheel_tf.header.stamp = self.get_clock().now().to_msg()
            trailer_right_wheel_tf.header.frame_id = 'trailer_link'  # Parent frame
            trailer_right_wheel_tf.child_frame_id = 'trailer_wheel_rr_link'  # Child frame
            trailer_right_wheel_tf.transform.translation.x = -0.55/4 - 0.55/2
            trailer_right_wheel_tf.transform.translation.y = (0.4/2 + 0.045/2) 
            trailer_right_wheel_tf.transform.translation.z =  -0.105  
            trailer_right_wheel_tf.transform.rotation.x = 0.0
            trailer_right_wheel_tf.transform.rotation.y = 0.0
            trailer_right_wheel_tf.transform.rotation.z = 0.0
            trailer_right_wheel_tf.transform.rotation.w = 1.0 

            

            transforms = [
                trailer_tf, 
                trailer_link_tf, 
                trailer_left_wheel_tf, 
                trailer_right_wheel_tf
            ]

            self.tf_broadcaster.sendTransform(transforms)
            # self.joint_state_pub.publish(joint_state_msg)
            # self.joint_state_pub.publish(right_wheel)
            # self.joint_state_pub.publish(left_wheel)
            # self.get_logger().info(f"Trailer yaw: {self.trailer_yaw}, Tractor yaw: {self.tractor_yaw}, Trailer pos: {self.tractor_pos}")


    
            
        except Exception as e:
            if not self.sent_exception:
                self.get_logger().error(f"Failed to get transform: {str(e)}")
                self.sent_exception = True


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
