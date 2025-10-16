DH = [
        0, 76, 0, pi/2;
        0, -23.65, 43.23, 0;
        0, 0, 0, pi/2;
        0, 43.18, 0, -pi/2;
        0, 0, 0, pi/2;
        0, 20, 0, 0;
    ];

myrobot = mypuma560(DH);

x = linspace(20, 30, 100);
y = linspace(23, 30, 100);
z = linspace(15, 100, 100);
R = [cos(pi/4), -sin(pi/4), 0; sin(pi/4), cos(pi/4) 0; 0, 0, 1];

q = zeros(100,6);

for i = 1:100
    H = [R [x(i); y(i); z(i)]; 0 0 0 1];
    q(i,:) = inverse(H, myrobot);
end
plot3(x, y, z,'r')
hold on
plot(myrobot, q)
