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


H1 = eul2tr([0 pi pi/2]);
H1(1:3,4)=100*[-1; 3; 3;]/4;
q1 = inverse(H1,myrobot);

H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);

tau = att(q1,q2,myrobot)

