function H = forward_kuka(joint, myrobot)
    
    H = eye(4);

    
    for i = 1:length(joint)

        q = joint(i); % join variable
        
        alpha = myrobot.alpha(1,i);
        d = myrobot.d(1,i);
        a = myrobot.a(1,i);
        

        % Create H matrix 
        H_i = [cos(q) -sin(q)*cos(alpha) sin(q)*sin(alpha)    a*cos(q);
               sin(q) cos(q)*cos(alpha)  -cos(q)*sin(alpha)   a*sin(q);
               0      sin(alpha)         cos(alpha)           d;
               0      0                 0                   1];
        H = H* H_i;
    end
     
end