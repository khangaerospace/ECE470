% This is only to test the forward.m code, it's also in Lab1.m file

clear; clc; close all;

% join angle
theta1 = linspace(0,pi,200);
theta2 = linspace(0,pi/2,200);
theta3 = linspace(0,pi,200);
theta4 = linspace(pi/4, (3*pi)/4, 200);
theta5 = linspace((-pi/3), (pi/3), 200);
theta6 = linspace(0,2*pi, 200);

% DH table
DH = [
        0, 76, 0, pi/2;
        0, -23.65, 43.23, 0;
        0, 0, 0, pi/2;
        0, 43.18, 0, -pi/2;
        0, 0, 0, pi/2;
        0, 20, 0, 0;
    ];

H = eye(4);

myrobot = mypuma560(DH);

q = [theta1; theta2; theta3; theta4; theta5; theta6]';

o = zeros(200,3);

% solve the Forward Kinematics problem
for i = 1:200
    H = forward(q(i,:), myrobot);
    o(i, :) = H(1:3, 4); 
end

% plot the solutiuon
plot3(o(:,1),o(:,2),o(:,3), 'r')
hold on
plot(myrobot,q)

