function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    q = q0;
    alpha_att = 0.01; % set this to 0.01 later
    alpha_rep = 0.01;
    % use lecture 21
    k = 1;
    while norm(q(end,1:5)-q2(1:5))>tol
        
        atractive = att(q(k,:), q2(:), myrobot)';
        tau_rep = [0 0 0 0 0 0];
        for j = 1:length(obs)
            repulsive = rep(q(k,:), myrobot, obs{j})';
            tau_rep = tau_rep + alpha_rep*repulsive';
        end

        dq = alpha_att*atractive + tau_rep; % change in distance
        
        q(end+1, 1:6) = q(end,1:6) + dq;
        k = k + 1;
        if k > 4000
            break
            disp(k)
        end
    end

    % compute the spline
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');
end 