function H = forward(joint, myrobot)
    % Compute the transformation matrix for each joint using DH parameters
   DH = myrobot;

   % finding the DH parameter
   d0 = DH.d;
   a0 = DH.a;
   alpha0 = DH.alpha;

   H = eye(4); % Initialize the transformation matrix as the identity matrix
   

    for i = 1:size(6, 1)
        theta = joint(i); % Joint angle + offset
        d = d0(i, 2);
        a = a0(i, 3);
        alpha = alpha0(i, 1);
        
        % Create the transformation matrix for the current joint
        A_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
               sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
               0, sin(alpha), cos(alpha), d;
               0, 0, 0, 1];
        
        % Update the overall transformation matrix
        H = H * A_i;
    end
     
end

