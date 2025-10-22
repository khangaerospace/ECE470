%% Part 1 Forward Kinematics
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

%% Part 2 Inverse Kinematics

H_check = [cos(pi/4), -sin(pi/4), 0,20;
    sin(pi/4), cos(pi/4), 0, 23;
    0,0,1,15;
    0,0,0,1]
q_check = inverse(H_check, myrobot);

disp(q_check) % to see if it match the ground truth in the handout

% define the DH table
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


