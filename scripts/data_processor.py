#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates
import numpy as np
from advctrl_balancing_robot.msg import Observation


class DataProcessor():
    def __init__(self):
        rospy.loginfo("DataProcessor loading")
        # for publishing transforms between map and robot
        self.map_broadcaster = tf.TransformBroadcaster()
        self.base_pose = None
        self.link_states_sub = rospy.Subscriber(
            "/gazebo/model_states", LinkStates, self.link_states_callback

        )
        self.observation_pub = rospy.Publisher(
            '/observation/', Observation, queue_size=1)
        self.tf_listener = tf.TransformListener()
        # observation of states; x, x_dot, theta, theta_dot, psi, psi_dot
        self.observation = Observation()
        rospy.loginfo("DataProcessor successfuly loaded")
        pass

    def link_states_callback(self, msg):
        self.base_pose = msg.pose[1]
        base_twist = msg.twist[1]
        psi = tf.transformations.euler_from_quaternion((self.base_pose.orientation.x, self.base_pose.orientation.y,
                                                        self.base_pose.orientation.z, self.base_pose.orientation.w))[2]
        x_dot = base_twist.linear.x * \
            np.cos(psi) + base_twist.linear.y * np.sin(psi)
        psi_dot = base_twist.angular.z
        self.observation.x_dot = -x_dot
        self.observation.psi = psi
        self.observation.psi_dot = psi_dot
        angular_velocity = np.array(
            [base_twist.angular.x, base_twist.angular.y, base_twist.angular.z])
        rot_mat = np.array([[np.cos(psi), - np.sin(psi), 0],
                            [np.sin(psi), np.cos(psi), 0],
                            [0, 0, 1]])
        angular_velocity = angular_velocity.dot(rot_mat)
        self.observation.theta_dot = -angular_velocity[1]
        pass

    def spin(self):

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    '/base_footprint', '/body_com', rospy.Time(0))
                rot = tf.transformations.euler_from_quaternion(rot)
                self.observation.theta = -rot[1]
                # print(rot[1])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            if (self.base_pose is not None):
                base_footprint_orientation = tf.transformations.euler_from_quaternion((self.base_pose.orientation.x, self.base_pose.orientation.y,
                                                                                      self.base_pose.orientation.z, self.base_pose.orientation.w))
                base_footprint_orientation = tf.transformations.quaternion_from_euler(
                    0, 0, base_footprint_orientation[2])
                self.map_broadcaster.sendTransform((self.base_pose.position.x, self.base_pose.position.y, 0),
                                                   base_footprint_orientation,
                                                   rospy.Time.now(),
                                                   "base_footprint",
                                                   "map")
            self.observation_pub.publish(self.observation)

            # print(self.observation)
            rate.sleep()


def main(args=None):
    rospy.init_node("data_processor")
    data_processor = DataProcessor()
    data_processor.spin()


if __name__ == "__main__":
    main()
