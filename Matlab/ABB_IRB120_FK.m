clc; clear vars; close all;

disp('--- ABB IRB 120 Forward Kinematics Home Position Test ---');
theta_home = [0; 0; 0; 0; 0; 0];
[T_home, A_all_home, T_all_home] = FK(theta_home);
disp('Joint angles (deg):');
disp(theta_home');
disp('End-effector pose at home position:');
disp(T_home);
disp('Individual joint transformation matrices:');
for i = 1:6
    fprintf('A_%d = \n', i);
    disp(A_all_home(:,:,i));
end

disp('Cumulative base-to-joint transforms T_0i:');
for i = 1:6
    fprintf('T_0%d = \n', i);
    disp(T_all_home(:,:,i));
end


function [T, A_all, T_all] = FK(theta)
% ABB_IRB120_FK  Forward Kinematics for ABB IRB 120
%   theta: 6x1 vector of joint angles [deg]
%   T: 4x4 homogeneous transformation matrix (end-effector pose)
%   A_all: 4x4x6 array of individual joint transformation matrices
%   T_all: 4x4x6 array of cumulative transforms T_0i


% DH Parameters (corrected, in mm and degrees)
% Note: a2 and a3 are negative to match ABB IRB 120 convention
DH = [
        0    -90   290   theta(1);
    270     0     0    (theta(2) - 90);
     70   -90     0    theta(3);
        0    90   302    theta(4);
        0   -90     0    theta(5);
        0     0    72    theta(6)
];

T = eye(4);
A_all = zeros(4,4,6);
T_all = zeros(4,4,6);
for i = 1:6
    a = DH(i,1);
    alpha = deg2rad(DH(i,2));
    d = DH(i,3);
    theta_i = deg2rad(DH(i,4));
    % DH Transformation for each joint
    A = [cos(theta_i), -sin(theta_i)*cos(alpha),  sin(theta_i)*sin(alpha), a*cos(theta_i);
         sin(theta_i),  cos(theta_i)*cos(alpha), -cos(theta_i)*sin(alpha), a*sin(theta_i);
                  0  ,             sin(alpha),             cos(alpha),            d;
                  0  ,                     0,                     0,            1];
    A_all(:,:,i) = A;
    T = T * A;
    T_all(:,:,i) = T;
end
end

