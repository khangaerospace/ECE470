X_workspace = zeros(3, 100);

th = linspace(0,2*pi,100);
X_workspace(1,:) = 13*(sin(th).^3)+620;
X_workspace(2,:) = 16*cos(th)-5*cos(2*th)-2*cos(3*th)-cos(4*th);
X_workspace(3,:) = -1;


R =[0 0 1;0 -1 0; 1 0 0 ];

for i = 1:100

    X_baseframe = FrameTransformation(X_workspace(:,i));
    H = [R X_baseframe; zeros(1,3) 1];

    q = inverse_kuka(H,myrobot);
    setAngles(q,0.02)
end