%% Squawk 4: Phugoid at cruise
%  737-class transport, flight test aircraft T-1. M 0.85, 35,000 ft, airplane as designed.
%  Linear model, stability axes. Run this script to load A, B, C, D, u0.
%
% States  x = [du (ft/s); w (ft/s); q (rad/s); dtheta (rad)]   Etkin (4.9,18)
% Inputs  c = [delta_e (rad); delta_p (lb of thrust)]          Etkin (7.6,4)
% Signs: +delta_e is trailing edge down, nose DOWN (C_m_de = -1.20).
% Angle of attack: dalpha = w/u0.
%
% Flight test input: controls fixed, 10 ft/s fast: x0 = [10; 0; 0; 0], 600 s.
% Altitude is not a state: integrate hdot = u0*dtheta - w.

u0 = 826.935;          % ft/s
h  = 35000;            % ft

A = [ ...
      -0.00503057     0.0215514             0       -32.174;
       -0.0867525     -0.482064        822.23            -0;
      1.47454e-05   -0.00352204     -0.655124             0;
                0             0             1             0 ];

B = [ ...
                0    0.00022189;
         -26.1878             0;
         -3.57185   3.39443e-06;
                0             0 ];

C = eye(4);
D = zeros(4, 2);
states = {'du (ft/s)', 'w (ft/s)', 'q (rad/s)', 'dtheta (rad)'};
inputs = {'delta_e (rad)', 'delta_p (lb)'};
fprintf('Squawk 4 loaded: A (4x4), B (4x2), C, D, u0 = %.1f ft/s\n', u0);
