#!/usr/bin/env python
import scipy as sp
import rospy
import numpy as np
import tf
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from math import pow, fabs
import scipy.linalg
from advctrl_balancing_robot.msg import Observation
from geometry_msgs.msg import Twist


class LQRController():
    def __init__(self):

        # topics for publishing torgues
        self.T1_pub = rospy.Publisher(
            '/teeterbot/right_torque_cmd', Float64, queue_size=1)
        self.T2_pub = rospy.Publisher(
            '/teeterbot/left_torque_cmd', Float64, queue_size=1)
        self.observation_sub = rospy.Subscriber(
            '/observation/', Observation, self.observation_callback, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber(
            '/cmd_vel/', Twist, self.cmd_vel_callback, queue_size=1)
        # torgue limit
        self.T_max = 5.2
        self.K = self.lqr_coeffs()
        self.T1_control = Float64()
        self.T2_control = Float64()
        self.ref_state = np.zeros(6)  # holds reference system states
        self.current_state = np.zeros(6)
        self.K = self.lqr_coeffs()
        pass

    def lqr_coeffs(self):
        """Calculates lqr cofficents by solving ARE 

        Returns:
            K gains
        """
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
        R = np.diag([0.01])
        Q = np.diag([0., 0.5, 5.5, .4])
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        R_inv = scipy.linalg.inv(R)
        K = np.dot(R_inv, np.dot(B.transpose(), P))
        print(scipy.linalg.eigvals(A - np.dot(B, K)))
        return K

    def observation_callback(self, msg):
        self.current_state[0] = msg.x
        self.current_state[1] = msg.x_dot
        self.current_state[2] = msg.theta
        self.current_state[3] = msg.theta_dot
        self.current_state[4] = msg.psi
        self.current_state[5] = msg.psi_dot

    def calculate_action(self):
        torgue = np.dot(
            self.K, (self.ref_state[:4]-self.current_state[:4]).reshape(-1, 1))
        torgue = -torgue[0, 0]
        if (fabs(torgue) > self.T_max):
            torgue = self.T_max * np.sign(torgue)

        # very dummy solution, but it works, IDC!!!
        delta = self.ref_state[5] / 1
        self.T1_control.data = torgue + delta
        self.T2_control.data = torgue - delta
        self.T1_pub.publish(self.T1_control)
        self.T2_pub.publish(self.T2_control)

    def cmd_vel_callback(self, msg):
        self.ref_state[1] = msg.linear.x/3
        self.ref_state[5] = msg.angular.z/2

        pass

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.calculate_action()

            rate.sleep()


def main(args=None):
    rospy.init_node("lqr_controller")
    data_processor = LQRController()
    data_processor.spin()


if __name__ == "__main__":
    main()
