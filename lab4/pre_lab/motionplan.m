function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    q = q0;
    alpha_att = 0.013;
    alpha_rep = 0.01;
    % use lecture 21
    k = 1;
    while norm(q(end,1:5)-q2(1:5))>tol
        
        atractive = att(q(k,:), q2(:), myrobot)';
        repulsive = rep(q(k,:), myrobot, obs)';

        dq = alpha_att*atractive+alpha_rep*repulsive; % change in distance
        
        q(k+1, 1:6) = q(k,1:6) + dq;
        k = k + 1;
    end

    % compute the spline
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');
end 