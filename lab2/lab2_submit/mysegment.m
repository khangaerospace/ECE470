X_workspace = zeros(3, 100);

X_workspace(1,:) = 620;
X_workspace(2,:) = linspace(-100, 100, 100);
X_workspace(3,:) = -1;


R =[0 0 1;0 -1 0; 1 0 0 ];

for i = 1:100
    X_baseframe = FrameTransformation(X_workspace(:,i));
    H = [R X_baseframe; zeros(1,3) 1];
    
    q = inverse_kuka(H,myrobot);
    setAngles(q,0.04)
end

