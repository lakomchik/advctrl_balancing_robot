#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates
import numpy as np
import scipy
import scipy.linalg
from std_msgs.msg import Float64
from math import pow, fabs
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
        self.tf_listener = tf.TransformListener()
        # observation of states; x, x_dot, theta, theta_dot, psi, psi_dot
        self.observation = np.zeros(6)

        self.T1_pub = rospy.Publisher(
            '/teeterbot/right_torque_cmd', Float64, queue_size=1)
        self.T2_pub = rospy.Publisher(
            '/teeterbot/left_torque_cmd', Float64, queue_size=1)
        self.T_max = 5.2
        self.K = self.lqr_coeffs()
        self.control = Float64()
        pass

    def lqr_coeffs(self):
        A = np.zeros([4, 4], dtype=float)
        B = np.zeros([4, 1], dtype=float)
        # System parameters
        mb = 10.
        Jb = 1.40833
        r = 0.2
        l = 0.4
        mw = 1.
        Jw = 0.02
        g = 9.81
        # Constants for linearized A and B
        den1 = mb + 2*(1/pow(r, 2)*Jw + mw) - \
            pow(mb * l, 2)/(Jb + mb * pow(l, 2))
        C1 = (pow((mb * l), 2) * g / (Jb + mb * pow(l, 2)))/den1
        C2 = (2/r + 2*mb*l/(Jb + mb * pow(l, 2)))/den1
        den2 = Jb + mb*pow(l, 2) - pow(mb*l, 2) / \
            (mb + 2 * (1/pow(r, 2) * Jw + mw))
        C3 = mb * g * l / den2
        C4 = (2 * mb * l/(mb*r + 2 * (Jw/r + mw * r))+2)/den2
        A[0, 1] = 1
        A[1, 2] = - C1
        A[2, 3] = 1
        A[3, 2] = C3
        B[1, 0] = C2
        B[3, 0] = -C4
        print(A)
        print(B)
        R = np.diag([0.01])
        Q = np.diag([0., 0.5, 5.5, .4])
        # file.write(str(Q[0, 0])+"\t" + str(Q[1, 1])+"\t" +
        #            str(Q[2, 2])+str(Q[3, 3])+"\t"+str(R[0, 0])+"\n")
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        print(P)
        R_inv = scipy.linalg.inv(R)
        K = np.dot(R_inv, np.dot(B.transpose(), P))
        # print(K)
        print(scipy.linalg.eigvals(A - np.dot(B, K)))
        return K

    def link_states_callback(self, msg):
        self.base_pose = msg.pose[1]
        base_twist = msg.twist[1]
        # calculating x_dot
        psi = tf.transformations.euler_from_quaternion((self.base_pose.orientation.x, self.base_pose.orientation.y,
                                                        self.base_pose.orientation.z, self.base_pose.orientation.w))[2]
        x_dot = base_twist.linear.x * \
            np.cos(psi) + base_twist.linear.y * np.sin(psi)
        psi_dot = base_twist.angular.z
        self.observation[1] = -x_dot
        self.observation[4] = psi
        self.observation[5] = psi_dot
        angular_velocity = np.array(
            [base_twist.angular.x, base_twist.angular.y, base_twist.angular.z])
        rot_mat = np.array([[np.cos(psi), - np.sin(psi), 0],
                            [np.sin(psi), np.cos(psi), 0],
                            [0, 0, 1]])
        angular_velocity = angular_velocity.dot(rot_mat)
        self.observation[3] = -angular_velocity[1]
        pass

    def calculate_action(self):
        ref_state = np.zeros([4], dtype=float)
        torgue = np.dot(
            self.K, (ref_state-self.observation[:4]).reshape(-1, 1))
        print(torgue)
        self.control.data = -torgue[0, 0]
        if (fabs(self.control.data) > self.T_max):
            self.control.data = self.T_max * \
                (self.control.data)/fabs(self.control.data)
        # print(self.control.data)
        delta = 0.1
        left = Float64()
        right = Float64()
        left.data = self.control.data - delta
        right.data = self.control.data + delta
        self.T1_pub.publish(right)
        self.T2_pub.publish(left)

    def spin(self):

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    '/base_footprint', '/body_com', rospy.Time(0))
                rot = tf.transformations.euler_from_quaternion(rot)
                self.observation[2] = -rot[1]
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
            # pass
            # print("OK")
            print(self.observation.T)
            self.calculate_action()
            rate.sleep()


def main(args=None):
    rospy.init_node("data_processor")
    data_processor = DataProcessor()
    data_processor.spin()


if __name__ == "__main__":
    main()
