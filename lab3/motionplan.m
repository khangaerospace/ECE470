function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    q = q0;
    alpha = 0.01;

    % use lecture 21
    k = 1;
    while norm(q(end,1:5)-q2(1:5))>tol

        tau = att(q(k+1,:), q2(:), myrobot);

        dq = alpha*tau; % change in distance
        q(k+1, 1:6) = q(k,1:6) + dq;
        k = k + 1;
    end
    N = size(q,1);
    q(:,6) = linspace(q0(6), q2(6), N).';
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');
end