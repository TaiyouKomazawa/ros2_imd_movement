import sys
import math, numpy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros.transform_broadcaster import TransformBroadcaster

from message_filters import Subscriber, ApproximateTimeSynchronizer

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from ros2_imd_interfaces.msg import WheelFeed

class TopicSynchronizer(Node):
    def __init__(self):
        super().__init__('topic_synchronizer')
        self.get_logger().info('Starting ns={0},exec={1}'.format(self.get_namespace(), self.get_name()))

        self.declare_parameter('odom_frame',            'odom_frame')
        self.declare_parameter('wheels_origin_frame',   'base_link')

        self.declare_parameter('wheel_odom_topics',     ['']) #['imd1/wheel/feedback', 'imd2/wheel/feedback']
        self.declare_parameter('merged_odom_topic',     '/odometry')

        self.declare_parameter('calc_matrix_once',      True)
        self.declare_parameter('sync_hz',               60.0)

        wheel_odom_topics = self.get_parameter('wheel_odom_topics').get_parameter_value().string_array_value
        self.merged_odom_topic = self.get_parameter('merged_odom_topic').get_parameter_value().string_value

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.wheels_origin_frame = self.get_parameter('wheels_origin_frame').get_parameter_value().string_value

        self.calc_mat_once = self.get_parameter('calc_matrix_once').get_parameter_value().bool_value
        
        self.sync_hz = self.get_parameter('sync_hz').get_parameter_value().double_value
        if self.sync_hz <= 0.0:
            self.sync_hz = 60

        if len(wheel_odom_topics) > 0:
            self.get_logger().info("Target topic names : {0}".format(wheel_odom_topics))

            self.subscriptions_list = []
            for name in wheel_odom_topics:
                self.subscriptions_list.append(Subscriber(self, WheelFeed, name))
        
            self.mat_initialized = False
            self.last_pose = [0.0,0.0]
            self.roted_pose = [0.0,0.0]
            self.last_vel = [0.0,0.0]
            self.roted_vel = [0.0,0.0]
            self.ats = ApproximateTimeSynchronizer(self.subscriptions_list, 20, 1.0/self.sync_hz)
            self.ats.registerCallback(self.subscriptionCallback)

            self.tf_initialized = False
            self.broadcaster = TransformBroadcaster(self)

            self.odom_pub = self.create_publisher(
                Odometry,
                self.merged_odom_topic,
                10)
        else:
            self.get_logger().error("Odometry topic name not specified.")


    def subscriptionCallback(self, *msgs):

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.wheels_origin_frame
        rot_angle = 0.0

        if len(msgs) > 1:
            if self.mat_initialized == False:
                T = numpy.vstack(msg.mat for msg in msgs)
                self.inv_T = numpy.linalg.pinv(T)
                if self.calc_mat_once:
                    self.mat_initialized = True

            w_pose = []
            w_vel = []
            for msg in msgs:
                w_pose.append(msg.pose)
                w_vel.append(msg.velocity)

            pose = numpy.dot(self.inv_T, numpy.matrix(w_pose).T)
            vel = numpy.dot(self.inv_T, numpy.matrix(w_vel).T)

            rot = pose[2, 0]
            diff = [0.0, 0.0]
            diff[0] = pose[0, 0] - self.last_pose[0]
            diff[1] = pose[1, 0] - self.last_pose[1]
            dst = self.rotate_(diff, rot)
            self.roted_pose[0] += dst[0]
            self.roted_pose[1] += dst[1]

            odom_msg.pose.pose.position.x = self.roted_pose[0]
            self.last_pose[0] = pose[0, 0]
            odom_msg.pose.pose.position.y = self.roted_pose[1]
            self.last_pose[1] = pose[1, 0]
            odom_msg.pose.pose.orientation.z = math.sin(rot/2.0)
            odom_msg.pose.pose.orientation.w = math.cos(rot/2.0)

            diff[0] = vel[0, 0] - self.last_vel[0]
            diff[1] = vel[1, 0] - self.last_vel[1]
            dst = self.rotate_(diff, rot)
            self.roted_vel[0] += dst[0]
            self.roted_vel[1] += dst[1]

            odom_msg.twist.twist.linear.x = self.roted_vel[0]
            self.last_vel[0] = vel[0, 0]
            odom_msg.twist.twist.linear.y = self.roted_vel[1]
            self.last_vel[1] = vel[1, 0]
            odom_msg.twist.twist.angular.z = vel[2, 0]


        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = odom_msg.header.stamp
        odom_to_base.header.frame_id = odom_msg.header.frame_id
        odom_to_base.child_frame_id = odom_msg.child_frame_id
        odom_to_base.transform.translation.x = odom_msg.pose.pose.position.x
        odom_to_base.transform.translation.y = odom_msg.pose.pose.position.y
        odom_to_base.transform.translation.z = 0.0

        odom_to_base.transform.rotation.x = 0.0
        odom_to_base.transform.rotation.y = 0.0
        odom_to_base.transform.rotation.z = odom_msg.pose.pose.orientation.z
        odom_to_base.transform.rotation.w = odom_msg.pose.pose.orientation.w

        self.odom_pub.publish(odom_msg)
        self.broadcaster.sendTransform(odom_to_base)

        if self.tf_initialized == False:
            self.get_logger().info("The first tf2 frames are now published.(from '{0}' frame to '{1}' frame)".format(
                    odom_msg.header.frame_id, odom_msg.child_frame_id))
            self.tf_initialized = True
    
    def rotate_(self, src, rot):
        dst = [0.0, 0.0]
        dst[0] =  src[0]*math.cos(rot) + src[1]*math.sin(rot)
        dst[1] = -src[0]*math.sin(rot) + src[1]*math.cos(rot)
        return dst

def main(args=None):
    rclpy.init(args=args)
    try:
        topic_synchronizer = TopicSynchronizer()
        rclpy.spin(topic_synchronizer)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(-1)
    finally:
        rclpy.try_shutdown()
        topic_synchronizer.destroy_node()


if __name__ == '__main__':
    main()