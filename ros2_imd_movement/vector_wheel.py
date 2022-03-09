import sys
import math, numpy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped, Twist
from ros2_imd_interfaces.msg import MotorCmd, MotorFeed, WheelFeed

class VectorWheel(Node):
    def __init__(self):
        super().__init__('vector_wheel')
        self.get_logger().info('Starting ns={0},exec={1}'.format(self.get_namespace(), self.get_name()))

        self.declare_parameter('origin_frame_name', 'base_link')
        self.declare_parameter('motor_feed_topic',  'feedback')
        self.declare_parameter('motor_cmd_topic',   'command')
        self.declare_parameter('wheel_feed_topic',  'wheel/feedback')
        self.declare_parameter('wheel_cmd_topic',   'cmd_vel')

        self.declare_parameter('wheel.radius',              0.025) #[m]
        self.declare_parameter('wheel.footprint',           [0.05, -0.05]) #(x, y)[m]
        self.declare_parameter('wheel.fixed_angle',         0) #(Fixed angle of rotation axis (Z-axis rotation, X-axis origin))[rad]
        self.declare_parameter('wheel.sliding_direction',   0) #(Angle of sliding direction (Z-axis rotation, Y-axis origin))[rad]

        self.origin_frame_id = self.get_parameter('origin_frame_name').get_parameter_value().string_value
        self.motor_feed_topic = self.get_parameter('motor_feed_topic').get_parameter_value().string_value
        self.motor_cmd_topic = self.get_parameter('motor_cmd_topic').get_parameter_value().string_value
        self.wheel_feed_topic =  self.get_parameter('wheel_feed_topic').get_parameter_value().string_value
        self.wheel_cmd_topic =  self.get_parameter('wheel_cmd_topic').get_parameter_value().string_value

        self.wheel_radius = self.get_parameter('wheel.radius').get_parameter_value().double_value
        self.wheel_footprint = self.get_parameter('wheel.footprint').get_parameter_value().double_array_value
        self.wheel_fixed_angle = self.get_parameter('wheel.fixed_angle').get_parameter_value().double_value
        self.wheel_sliding_direction = self.get_parameter('wheel.sliding_direction').get_parameter_value().double_value

        self.get_logger().info('My origin frame id : {0}'.format(self.origin_frame_id))
        self.get_logger().info('Topics are transformed by kinematics.the correspondence is as follows : ')
        self.get_logger().info('        |Wheel topic\t|  |Frame topic\t|')
        self.get_logger().info('Command |{0}\t|<-|{1}\t|'.format(self.motor_cmd_topic, self.wheel_cmd_topic))
        self.get_logger().info('Odometry|{0}\t|->|{1}\t|\n'.format(self.motor_feed_topic, self.wheel_feed_topic))
        self.get_logger().info('Wheel params : ')
        self.get_logger().info(' -radius : {:f}[m]'.format(self.wheel_radius))
        flen = math.sqrt(self.wheel_footprint[0]*self.wheel_footprint[0]+self.wheel_footprint[1]*self.wheel_footprint[1])
        self.get_logger().info(' -footprint : [{0:f}, {1:f}][m](L:{2:f}[m])'.format(self.wheel_footprint[0], self.wheel_footprint[1], flen))
        self.get_logger().info(' -fixed_angle : {:f}[rad]'.format(self.wheel_fixed_angle))
        self.get_logger().info(' -sliding_direction : {:f}[rad]\n'.format(self.wheel_sliding_direction))

        self.create_kinematic_matrix_()

        self.initialized_origin = False
        self.broadcaster = StaticTransformBroadcaster(self)

        self.sub_motor_feed = self.create_subscription(
            MotorFeed,
            self.motor_feed_topic,
            self.subscribeFeedbackCallback,
            10)

        self.pub_motor_cmd = self.create_publisher(
            MotorCmd,
            self.motor_cmd_topic,
            10)

        self.pub_wheel_feed = self.create_publisher(
            WheelFeed,
            self.wheel_feed_topic,
            10)

        self.sub_wheel_cmd = self.create_subscription(
            Twist,
            self.wheel_cmd_topic,
            self.subscribeCommandCallback,
            10)

        self.check_sub_timer = self.create_timer(
            5,
            self.waitMsgCallback_)
    
    def subscribeFeedbackCallback(self, msg):
        wheel_feed = WheelFeed()
        wheel_feed.header.stamp = self.get_clock().now().to_msg()
        wheel_feed.header.frame_id = msg.header.frame_id

        wheel_feed.mat = (self.transform_mat[0,0],self.transform_mat[0,1],self.transform_mat[0,2])
        wheel_feed.pose = self.rad_to_vel_(msg.pose)
        wheel_feed.velocity = self.rad_to_vel_(msg.velocity)

        self.pub_wheel_feed.publish(wheel_feed)

        if self.initialized_origin == False:
            self.motor_frame_id = wheel_feed.header.frame_id
            self.broadcastOriginFrame_()
            self.get_logger().info('Now,broadcasted static frame from {0}.'.format(self.origin_frame_id))
            self.check_sub_timer.cancel()
            self.initialized_origin = True

    def subscribeCommandCallback(self, msg):
        cmd = MotorCmd()
        cmd.velocity = self.vel_to_rad_(self.frame_to_wheel_vec_(msg.linear.x, msg.linear.y, msg.angular.z))
        self.pub_motor_cmd.publish(cmd)

    def broadcastOriginFrame_(self):
        origin_to_motor = TransformStamped()

        origin_to_motor.header.stamp = self.get_clock().now().to_msg()
        origin_to_motor.header.frame_id = self.origin_frame_id
        origin_to_motor.child_frame_id = self.motor_frame_id
        origin_to_motor.transform.translation.x = self.wheel_footprint[0]
        origin_to_motor.transform.translation.y = self.wheel_footprint[1]
        origin_to_motor.transform.translation.z = 0.0

        origin_to_motor.transform.rotation.x = 0.0
        origin_to_motor.transform.rotation.y = 0.0
        origin_to_motor.transform.rotation.z = math.sin(self.wheel_fixed_angle/2.0)
        origin_to_motor.transform.rotation.w = math.cos(self.wheel_fixed_angle/2.0)

        self.broadcaster.sendTransform(origin_to_motor)

    def waitMsgCallback_(self):
        self.get_logger().info('waiting topic : {0}'.format(self.motor_feed_topic))

    def rad_to_vel_(self, radian):
        return self.wheel_radius * radian

    def vel_to_rad_(self, vel):
        return  vel / self.wheel_radius

    def create_kinematic_matrix_(self):
        lx = self.wheel_footprint[0]
        ly = self.wheel_footprint[1]

        sin_f = math.sin(self.wheel_fixed_angle)
        cos_f = math.cos(self.wheel_fixed_angle)
        tan_s = math.tan(self.wheel_sliding_direction)

        if tan_s > 10:
            self.get_logger().error('It is not possible to define a sliding angle that is the same as or approximates the direction of rotation of the wheel (=|pi/2|).')

        Ts = numpy.matrix([[1, tan_s]])
        Tf = numpy.matrix([[cos_f, sin_f], [-sin_f, cos_f]])
        Tl = numpy.matrix([[1, 0, -ly], [0, 1, lx]])
        self.transform_mat = numpy.dot(numpy.dot(Ts, Tf), Tl)

    def frame_to_wheel_vec_(self, x, y, theta):
        V = numpy.matrix([[x],[y],[theta]])
        return numpy.dot(self.transform_mat, V)[0, 0]

def main(args=None):
    rclpy.init(args=args)
    try:
        vector_wheel = VectorWheel()
        rclpy.spin(vector_wheel)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(-1)
    finally:
        rclpy.try_shutdown()
        vector_wheel.destroy_node()


if __name__ == '__main__':
    main()
