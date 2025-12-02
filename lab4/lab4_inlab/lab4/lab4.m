%% in Lab 4 component 

clearvars -except udpObj
close all
theta = zeros(6);
DH = [theta(1), 400, 25, pi/2;
      theta(2), 0, 315, 0;
      theta(3), 0, 35, pi/2;
      theta(4), 365, 0, -pi/2;
      theta(5), 0, 0, pi/2;
      theta(6), 161.44, -156, 0];

DH_forces = [theta(1), 400, 25, pi/2;
      theta(2), 0, 315, 0;
      theta(3), 0, 35, pi/2;
      theta(4), 365, 0, -pi/2;
      theta(5), 0, 0, pi/2;
      theta(6), 161.44, 0, 0];
myrobot_forces = mykuka(DH_forces);

myrobot = mykuka(DH);

z_grid = 45;

p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [650 350 225];

kuka = mykuka(DH);
R=[0 0 1;0 -1 0;1 0 0];
H0=[R p0';zeros(1,3) 1];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
H3=[R p3';zeros(1,3) 1];

home = [0, pi/2, 0, 0, pi/2, 0];
q0 = inverse_kuka( H0, kuka);
q1 = inverse_kuka( H1, kuka);
q2 = inverse_kuka( H2, kuka);
q3 = inverse_kuka( H3, kuka);

tol = 0.035;
hold on
setupobstacle
plotobstacle(prepobs);
qrefhome = motionplan(home,q0,0,10,myrobot_forces,prepobs,tol);
disp('Finish Home')
t = linspace(0,10,100);
q0_path = ppval(qrefhome,t)';
q0_path(:,6) = linspace(home(6),q0(6),100);

% p0 -> p1
qref = motionplan(q0,q1,0,10,myrobot_forces,prepobs,tol);
disp('Finish P0 -> P1')
q1_path = ppval(qref,t)';
q1_path(:,6) = linspace(q0(6),q1(6),100);
%q1_path = linespace(q0,q1,100);
% p1 ->p2
qref1 = motionplan(q1,q2,0,10,myrobot_forces,prepobs,tol);
disp('Finish P1 -> P2')
q2_path = ppval(qref1,t)';
q2_path(:,6) = linspace(q1(6),q2(6),100);

% p2->p3
qref2 = motionplan(q2,q3,0,10,myrobot_forces,prepobs,tol);
disp('Finish P2 -> P3')
q3_path = ppval(qref2,t)';
q3_path(:,6) = linspace(q2(6),q3(6),100);

%%
plot(myrobot,q0_path);
plot(myrobot,q1_path);
plot(myrobot,q2_path);
plot(myrobot,q3_path);
hold off
%%
% for k = 1:size(q0_path,1)
%     setAngles(q0_path(k,:),0.03)
% end
% setGripper(0)
% for k = 1:size(q1_path,1)
%     setAngles(q1_path(k,:),0.03)
% end
% 
% setGripper(1)
% for k = 1:size(q2_path,1)
%     setAngles(q2_path(k,:),0.03)
% end
% for k = 1:size(q3_path,1)
%     setAngles(q3_path(k,:),0.03)
% end
% setGripper(0)

%% Creative Design

% The idea is to stack all three plot on top of each other
% unfortunately, we can not get this to run properly in lab so the TA
% told us to show the simulation. I hope this is ood enough!
% Thank you for a wonderful semester, I learned a lot! 

block_hight = 10;
home = [0, pi/2, 0, 0, pi/2, 0];

disp("Creative Design Begin")

% block 1
p1_0 = [370 -440 150];
p1_1 = [370 -440 z_grid]; % Location of blog 1
p1_2 = [650 350 225]; % Stack Location
p1_3 = [650 350 z_grid+block_hight]; % Stack Location move down

kuka = mykuka(DH);
R=[0 0 1;0 -1 0;1 0 0];
H1_0 =[R p1_0';zeros(1,3) 1];
H1_1=[R p1_1';zeros(1,3) 1];
H1_2=[R p1_2';zeros(1,3) 1];
H1_3=[R p1_3';zeros(1,3) 1];


q1_0 = inverse_kuka( H1_0, kuka);
q1_1 = inverse_kuka( H1_1, kuka);
q1_2 = inverse_kuka( H1_2, kuka);
q1_3 = inverse_kuka( H1_3, kuka);
% perform pick up for part 1 first

tol = 0.1;

qrefhome = motionplan(home,q1_0,0,10,myrobot_forces,prepobs,tol);
disp('Above Block 1 Location')
t = linspace(0,10,100);
q1_0_path = ppval(qrefhome,t)';
q1_0_path(:,6) = linspace(home(6),q1_0(6),100);

% p0 -> p1
qref1_0 = motionplan(q1_0,q1_1,0,10,myrobot_forces,prepobs,tol);
disp('Taking Block 1')
q1_1_path = ppval(qref1_0,t)';
q1_1_path(:,6) = linspace(q1_0(6),q1_1(6),100);
%q1_path = linespace(q0,q1,100);
% p1 ->p2
qref1_1 = motionplan(q1_1,q1_2,0,10,myrobot_forces,prepobs,tol);
disp('Above Stacking Location')
q1_2_path = ppval(qref1_1,t)';
q1_2_path(:,6) = linspace(q1_1(6),q1_2(6),100);

% p2->p3
qref1_2 = motionplan(q1_2,q1_3,0,10,myrobot_forces,prepobs,tol);
disp('Stacking block 1')
q1_3_path = ppval(qref1_2,t)';
q1_3_path(:,6) = linspace(q1_2(6),q1_3(6),100);

% block 2 pick up and stack it
p2_0 = [750 -220 225];
p2_1 = [750 -220 z_grid];
p2_2 = [650 350 225]; % Stack Location
p2_3 = [650 350 z_grid+2*block_hight]; % Stack Location move down

H2_0 =[R p2_0';zeros(1,3) 1];
H2_1=[R p2_1';zeros(1,3) 1];
H2_2=[R p2_2';zeros(1,3) 1];
H2_3=[R p2_3';zeros(1,3) 1];

q2_0 = inverse_kuka( H2_0, kuka);
q2_1 = inverse_kuka( H2_1, kuka);
q2_2 = inverse_kuka( H2_2, kuka);
q2_3 = inverse_kuka( H2_3, kuka);
% perform pick up for part 1 first
qref2_00 = motionplan(q1_3,home,0,10,myrobot_forces,prepobs,tol);
disp('Going Home')
q2_00_path = ppval(qref2_00,t)';
q2_00_path(:,6) = linspace(q1_3(6),home(6),100);

qref2_home = motionplan(home,q2_0,0,10,myrobot_forces,prepobs,tol);
disp('Above Block 2 Location')
q2_0_path = ppval(qref2_home,t)';
q2_0_path(:,6) = linspace(home(6),q2_0(6),100);

% p0 -> p1
qref2_0 = motionplan(q2_0,q2_1,0,10,myrobot_forces,prepobs,tol);
disp('Taking Block 2')
q2_1_path = ppval(qref2_0,t)';
q2_1_path(:,6) = linspace(q2_0(6),q2_1(6),100);
%q1_path = linespace(q0,q1,100);
% p1 ->p2
qref2_1 = motionplan(q2_1,q2_2,0,10,myrobot_forces,prepobs,tol);
disp('Above Stacking Location')
q2_2_path = ppval(qref2_1,t)';
q2_2_path(:,6) = linspace(q2_1(6),q2_2(6),100);

% p2->p3
qref2_2 = motionplan(q2_2,q2_3,0,10,myrobot_forces,prepobs,tol);
disp('Stacking block 2')
q2_3_path = ppval(qref2_2,t)';
q2_3_path(:,6) = linspace(q2_2(6),q2_3(6),100);

% Plot
figure
hold on
setupobstacle
plotobstacle(prepobs);
hold on
plot(myrobot,q1_0_path);
plot(myrobot,q1_1_path);
plot(myrobot,q1_2_path);
plot(myrobot,q1_3_path);
plot(myrobot,q2_00_path);
plot(myrobot,q2_0_path);
plot(myrobot,q2_1_path);
plot(myrobot,q2_2_path);
plot(myrobot,q2_3_path);
hold off

