function pBaseFrame = FrameTransformation(pWorkspace)
    % TODO 1/2: Add proper documentation for this function.
    
    %-------------------------- Calibration ----------------------------%
    % TODO 2/2: Fill in values Xi for i = {1, 2, 3}, which are 3 by 1
    % vectors. Note that if you recalibrate the robot, YOU MUST UPDATE
    % deltajoint.m too!
    
    % X1 = [673.76, 133.78, 19.66]';
    % X2 = [604.29, 353.77, 17.36]';
    % X3 = [668.87, -329.64, 19.45]';

    X1 = [690.92, -13.13, 26.61]';
    X2 = [592.3, -221.36, 24.55]';
    X3 = [463.07, 174.28, 24.1]';

    % X1 = [673.76, 133.78, 26.61]';
    % X2 = [604.29, 353.77, 24.55]';
    % X3 = [668.87, -329.64, 24.1]';
	%-------------------------------------------------------------------%

    % Finding plane coordinates: a*x + b*y + c*z = 1
    M = [X1'; X2'; X3']
    M_inverse = pinv(M)
    params = pinv(M)*[1 1 1]'
    a = params(1);
    b = params(2);
    c = params(3);
    v = [a b c]'  % plane normal vector; should be close to [0;0; 1] !
    
    % Rotation matrix; mapping from Base-frame to workspace-frame: R*v1 = v2
    v2 = v/norm(v);
    v1 = [0 0 1]';    
    rotV = vrrotvec(v1,v2);    
    R = vrrotvec2mat(rotV);
    
    % Calculating transformation matrix:  H*pWorkspace = pBaseFrame
    xOrig = 0;
    yOrig = 0;
    zOrig = (1 -a*xOrig -b*yOrig)/c;    
    Orig = [xOrig  yOrig  zOrig]';    
    H = [R, Orig; zeros(1,3), 1];    
    
    tmp = H*[pWorkspace;1];
    pBaseFrame = tmp(1:3);   % any point on the table is converted back
                             % to robot base-frame. 
                             % "pBaseFrame" can be used by robot
                             % inverse kinematics to find proper joint
                             % angles!
end
    
    
    
    
    
    


