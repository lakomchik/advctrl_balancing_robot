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
        self.ref_x_dot_pub = rospy.Publisher(
            '/ref_x_dot', Float64, queue_size=1)
        self.ref_psi_dot_pub = rospy.Publisher(
            '/ref_psi_dot', Float64, queue_size=1)
        self.cur_x_dot_pub = rospy.Publisher(
            '/cur_x_dot', Float64, queue_size=1)
        self.cur_psi_dot_pub = rospy.Publisher(
            '/cur_psi_dot', Float64, queue_size=1)
        self.cur_theta_pub = rospy.Publisher(
            '/cur_theta', Float64, queue_size=1)
        self.cur_theta_dot_pub = rospy.Publisher(
            '/cur_theta_dot', Float64, queue_size=1)
        # torgue limit
        self.T_max = 10.2
        self.K = self.lqr_coeffs()
        self.T1_control = Float64()
        self.T2_control = Float64()
        self.ref_state = np.zeros(6)  # holds reference system states
        self.current_state = np.zeros(6)
        self.K_1, self.K_2 = self.lqr_coeffs()
        pass

    def lqr_coeffs(self):

        d = 0.5
        l = 0.4
        r = 0.2
        m_b = 10
        m_w = 1
        g = 9.81
        I_2 = 0.15
        I_1 = 0.60833
        I_3 = 0.60833
        Dipl_1 = (m_b + 3 * m_w) * (m_b * d ** 2 + I_3) - (m_b * d) ** 2
        Dipl_2 = I_2 + (5 * l**2 + r**2 / 2) * m_w
        A23 = (-m_b * d)**2 * g/(Dipl_1)
        A43 = (m_b + 3*m_w) * m_b * d * g/Dipl_1
        B21 = (m_b * d ** 2 + I_3)/(Dipl_1 * r)
        B41 = -m_b * d/(Dipl_1 * r)
        B61 = l/(Dipl_2 * r)
        A = np.array([
            [0, 1, 0, 0],
            [0, 0, A23, 0],
            [0, 0, 0, 1],
            [0, 0, A43, 0],
        ])
        B = np.array([
            [0],
            [B21],
            [0],
            [B41]
        ])
        # solving first system
        R = np.diag([0.01])
        Q = np.diag([0., 0.5, 5.5, .4])
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        R_inv = scipy.linalg.inv(R)
        K = np.dot(R_inv, np.dot(B.transpose(), P))
        # print(scipy.linalg.eigvals(A - np.dot(B, K)))
        A_2 = np.array([[0, 1],
                        [0, 0]])
        B_2 = np.array([[0],
                        [B61]])
        R_2 = np.diag([0.01])
        Q_2 = np.diag([0., 1.5])
        P_2 = scipy.linalg.solve_continuous_are(A_2, B_2, Q_2, R_2)
        R_inv_2 = scipy.linalg.inv(R_2)
        K_2 = np.dot(R_inv_2, np.dot(B_2.transpose(), P_2))
        # print(K, K_2)
        return K, K_2
        # pass

    def observation_callback(self, msg):
        self.current_state[0] = msg.x
        self.current_state[1] = msg.x_dot
        self.current_state[2] = msg.theta
        self.current_state[3] = msg.theta_dot
        self.current_state[4] = msg.psi
        self.current_state[5] = msg.psi_dot

    def calculate_action(self):
        torgue_theta = np.dot(
            self.K_1, (self.ref_state[:4]-self.current_state[:4]).reshape(-1, 1))
        torgue_theta = -torgue_theta[0, 0]
        torgue_psi = np.dot(
            self.K_2, (self.ref_state[4:] -
                       self.current_state[4:]).reshape(-1, 1)
        )
        torgue_psi = torgue_psi[0, 0]
        left_torgue = 0.5 * torgue_theta + 0.5 * torgue_psi
        right_torgue = 0.5 * torgue_theta - 0.5 * torgue_psi

        if (fabs(right_torgue) > self.T_max):
            right_torgue = self.T_max * np.sign(right_torgue)
        if (fabs(left_torgue) > self.T_max):
            left_torgue = self.T_max * np.sign(left_torgue)
        # very dummy solution, but it works, IDC!!!
        self.T1_control.data = left_torgue
        self.T2_control.data = right_torgue
        self.T1_pub.publish(self.T1_control)
        self.T2_pub.publish(self.T2_control)

    def cmd_vel_callback(self, msg):
        self.ref_state[1] = msg.linear.x
        self.ref_state[5] = msg.angular.z

        pass

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.calculate_action()
            self.ref_x_dot_pub.publish(Float64(self.ref_state[1]))
            self.cur_x_dot_pub.publish(Float64(self.current_state[1]))
            self.ref_psi_dot_pub.publish(Float64(self.ref_state[5]))
            self.cur_psi_dot_pub.publish(Float64(self.current_state[5]))
            self.cur_theta_pub.publish(Float64(self.current_state[2]))
            self.cur_theta_dot_pub.publish(Float64(self.current_state[3]))
            rate.sleep()


def main(args=None):
    rospy.init_node("lqr_controller")
    data_processor = LQRController()
    data_processor.spin()


if __name__ == "__main__":
    main()
