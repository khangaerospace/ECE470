clear vars -except udpObj

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

H = eye(4);

myrobot = mykuka(DH);
myrobot_forces = mykuka(DH_forces);

% test 3.4
setupobstacle_lab4
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6], myrobot_forces, obs{1});
disp(tau)

% test 3.5
kuka = mykuka(DH);
kuka_forces = mykuka(DH_forces);
p1 = [620 375 50];
p2 = [620 -375 50];
R=[0 0 1;0 -1 0;1 0 0];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
q1 = inverse_kuka( H1, kuka);
q2 = inverse_kuka( H2, kuka);


hold on
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,kuka_forces,obs,0.05);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q);
hold off



