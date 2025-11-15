%theta = zeros(6);

%DH = [theta(1), 400, 25, pi/2;
      % theta(2), 0, 315, 0;
      % theta(3), 0, 35, pi/2;
      % theta(4), 365, 0, -pi/2;
      % theta(5), 0, 0, pi/2;
      % theta(6), 161.44, -296.23, 0];




delta = fminunc(@deltajoint,[0 0]);

myrobot = mykuka_search(delta);

%% --------- Go to calibration point -------------
R =[0 0 1 673.76;
    0 -1 0  133.78; 
    1 0 0 19.66;
    0 0 0 1 ];

q = inverse_kuka(R,myrobot);
setAngles(q,0.04)

%% -------- Go to the work frame ------------

p_workspace = [600; 100; 30];
p_baseframe = FrameTransformation(p_workspace);
R =[0 0 1;0 -1 0; 1 0 0 ];


H= [R p_baseframe; zeros(1,3) 1];

q = inverse_kuka(H,myrobot);
setAngles(q,0.04)
