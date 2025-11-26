clear; clc; close all;
thetas = zeros(6);

%% Part 1
DH = [
        0, 76, 0, pi/2;
        0, -23.65, 43.23, 0;
        0, 0, 0, pi/2;
        0, 43.18, 0, -pi/2;
        0, 0, 0, pi/2;
        0, 20, 0, 0;
    ]; % DH table for Puma 560

H = eye(4);

myrobot = mypuma560(DH);


H1 = [0 1 0 0;
    1 0 0 0;
    0 0 -1 0;
    0 0 0 1];

H1(1:3,4)=100*[-1; 3; 3;]/4;
q1 = inverse(H1,myrobot);

%H2 = eul2tr([0 pi -pi/2]);
H2 = [ 0 -1 0 0;
    -1 0 0 0;
    0 0 -1 0;
    0 0 0 1];
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);


tau = att(q1,q2,myrobot);


%% Part2 (since code for part 2 is being result for part 3, we commented this out)

%qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
%qref = motionplan(q1,q2,0,10,myrobot,[],0.01);

%t = linspace(0,10,300);
%q = ppval(qref,t);
%disp(q)
%plot(myrobot,q')

%% part 3 a

setupobstacle
q3 = 0.9*q1 + 0.1*q2;
tau = rep(q3,myrobot,obs{1});

q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot, obs{6});

% part 3b
setupobstacle
hold on
axis([-100 100 -100 100 0 200]);
view(-32,50);
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs{1},0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q);
hold off