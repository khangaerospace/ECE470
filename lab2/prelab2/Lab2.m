theta = zeros(6);

DH = [theta(1), 400, 25, pi/2;
      theta(2), 0, 315, 0;
      theta(3), 0, 35, pi/2;
      theta(4), 365, 0, -pi/2;
      theta(5), 0, 0, pi/2;
      theta(6), 161.44, -296.23, 0];


kuka = mykuka(DH);

H = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]',kuka)
inverse_kuka(H, kuka)