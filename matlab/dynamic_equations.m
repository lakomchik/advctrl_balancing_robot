%dynamic equations from https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/Control-Lab/SigiStudent.pdf
%syms d l r m_b m_w J g K I_2 I_1 I_3 x x_dot x_dot_dot theta theta_dot theta_dot_dot psi psi_dot psi_dot_dot T_l T_r

% %dynamics equations
% eq1 = (m_b + 2 * m_w + 2 * J / r^2 ) * x_dot_dot - m_b * l * (psi_dot ^ 2 + theta_dot ^ 2) * sin(theta) + m_b * l * cos(theta) * theta_dot_dot - (T_l + T_r)/r ==0;
% eq2 = (I_2 + m_b * l ^ 2) * theta_dot_dot + (m_b * l * cos(theta)) * x_dot_dot + (I_3 - I_1 - m_b * l^2) * psi_dot^2 * sin(theta) * cos(theta) - m_b * l * g * sin(theta) + (T_l + T_r) == 0;
% eq3 = (I_3 + 2 * K + m_w * d^2/2  + J * d^2 /(2 * r^2) - (I_3 -I_1 - m_b * l^2) * sin(theta)^2) * psi_dot_dot + (m_b * l * x_dot - 2 * ( I_3 - I_1 - m_b * l^2)*theta_dot * cos(theta))* psi_dot * sin(theta) - (T_r - T_l)*d/(2*r) ==0;
% 
% 
% %obtaining second order derivatives
% solution = solve([eq1, eq2, eq3], [x_dot_dot, theta_dot_dot, psi_dot_dot]);
% 
% x_ddot = solution.x_dot_dot;
% theta_ddot = solution.theta_dot_dot;
% psi_ddot = solution.psi_dot_dot;
% 
% %system_linearization theta = 0, x = 0
% x_ddot_l = subs(x_ddot,[x, sin(theta), cos(theta)], [0, theta, 1]);
% theta_ddot_l = subs(theta_ddot,[x, sin(theta), cos(theta)], [0, theta, 1]);
% psi_ddot_l = subs(psi_ddot,[x, sin(theta), cos(theta)], [0, theta, 1]);
% 
% 
% names = [d l r m_b m_w J g K I_2 I_1 I_3];
% values = [0.5, 0.4, 0.2, 10, 1., 1.40833, 9.81, 0.051, 0.60833, 0.60833, 0.15];
% 
% x_ddot_l = vpa(subs(x_ddot_l,names, values))
% theta_ddot_l = vpa(subs(theta_ddot_l,names, values))
% psi_ddot_l = subs(vpa(subs(psi_ddot_l,names, values)),[theta^2],0)

% d l r m_b m_w J g K I_2 I_1 I_3 = [0.5, 0.4, 0.2, 10, 1., 1.40833, 9.81, 0.051, 0.60833, 0.60833, 0.15]
d = 0.5;
l = 0.4;
r = 0.2;
m_b = 10;
m_w = 1;
J = 1.40833;
g = 9.81;
K = 0.051;
I_2 = 0.60833;
I_1 = 0.60833;
I_3 = 0.15;
% syms A24 A26 A44 A46 A66 B21 B22 B41 B42 B61 B62
Dipl_1 = (m_b + 3 * m_w) * (m_b * d ^ 2 + I_3) - (m_b * d) ^ 2;
Dipl_2 = I_2 + (5 * l^2 + r^2 /2) * m_w;
A23 = (-m_b * d)^2 * g/(Dipl_1);
A43 = (m_b + 3*m_w) * m_b * d * g/Dipl_1;
B21 = (m_b * d^2 + I_3)/(Dipl_1 * r);
B41 = -m_b * d/(Dipl_1 * r);
B61 = l/(Dipl_2 * r);


A = [0 1 0 0 0 0;
    0 0 A23 0 0 0;
    0 0 0 1 0 0;
    0 0 A43 0 0 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0];
B = [0 ;
    B21;
    0;
    B41;
    0;
    B61];

A = A(1:4,1:4)
B = B(1:4)

Q = diag([0.0001,1,1,1])
R = eye(1) * 1

lqr(A,B,Q,R)


