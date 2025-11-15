Jug_workspace = zeros(3, 187);

data = readmatrix('jug.xlsx');
Jug_workspace(1,:) = 550+10*data(:,1);
Jug_workspace(2,:) = 10*data(:,2);
Jug_workspace(3,:) =-ones(length(data),1);

R =[0 0 1;0 -1 0; 1 0 0 ];

for i = 1:100
    Jug_baseframe = FrameTransformation(Jug_workspace(:,i));
    H = [R Jug_baseframe; zeros(1,3) 1];

    q = inverse_kuka(H,myrobot);
    setAngles(q,0.02)
end