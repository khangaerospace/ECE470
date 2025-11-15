function X_error = deltajoint(delta)
    % TODO 1/2: Add proper documentation for this function.

    kuka = mykuka_search(delta);

    %-------------------------- Calibration ----------------------------%
    % TODO 2/2: Fill in values Xi and Qi for i = {1, 2, 3}. Xi are 3 by 1
    % column vectors, while Qi are 1 by 6 row vectors.
    
    X1 = [673.76, 133.78, 26.61]';
    X2 = [604.29, 353.77, 24.55]';
    X3 = [668.87, -329.64, 24.1]';
    Q1 = [0.1916, 0.6447, 0, 0.1328, 0.9327,0];
    Q2 = [0.5348, 0.6224, 0, 0, 1.0067,  0];
    Q3 = [-0.4531, 0.5424, 0, 0, 1.2881, 0];
    %-------------------------------------------------------------------%

    H1=forward_kuka(Q1, kuka);
    H2=forward_kuka(Q2, kuka);
    H3=forward_kuka(Q3, kuka);
    
    X_error=norm(H1(1:3,4)-X1)+norm(H2(1:3,4)-X2)+norm(H3(1:3,4)-X3);
end