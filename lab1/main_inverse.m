% This is only to test the inverse.m code, it's also in Lab1.m file

clear; clc; close all;

DH = [
        0, 76, 0, pi/2;
        0, -23.65, 43.23, 0;
        0, 0, 0, pi/2;
        0, 43.18, 0, -pi/2;
        0, 0, 0, pi/2;
        0, 20, 0, 0;
    ];

% create my robot
myrobot = mypuma560(DH);

% Define the object pick up from table
x = linspace(20, 30, 100);
y = linspace(23, 30, 100);
z = linspace(15, 100, 100);

% define the Rz(pi/4) rotation matrix
R = [cos(pi/4), -sin(pi/4), 0; sin(pi/4), cos(pi/4) 0; 0, 0, 1];

% define the q matrix
q = zeros(100,6);

% solve the Inverse Kinematics Problem
for i = 1:100
    H = [R [x(i); y(i); z(i)]; 0 0 0 1];
    q(i,:) = inverse(H, myrobot);
end

% plot the solution
plot3(x, y, z,'r')
hold on
plot(myrobot, q)
