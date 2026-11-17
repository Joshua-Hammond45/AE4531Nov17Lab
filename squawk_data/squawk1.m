%% Squawk 1: Dutch roll at cruise, yaw damper OFF
%  737-class transport, flight test aircraft T-1. M 0.85, 35,000 ft, airplane as designed.
%  Linear model, stability axes. Run this script to load A, B, C, D, u0.
%
% States  x = [v (ft/s); p (rad/s); r (rad/s); phi (rad)]      Etkin (4.9,19)
% Inputs  c = [delta_a (rad); delta_r (rad)]                    Etkin (7.9,3)
% Signs: +delta_a rolls RIGHT (C_l_da = +0.06; opposite to Etkin's B747).
%        +delta_r is trailing edge left, nose LEFT (C_n_dr = -0.09).
% Sideslip: beta = v/u0.
%
% Flight test input: hands off, initial sideslip of 1 deg: x0 = [u0*deg2rad(1); 0; 0; 0], no control input, 40 s.
% Yaw damper: OFF for this test point.

u0 = 826.935;          % ft/s
h  = 35000;            % ft

A = [ ...
       -0.0679578     -0.255068       -825.15        32.174;
       -0.0104323      -2.07628      0.708746             0;
       0.00270469     -0.110227     -0.395995             0;
                0             1             0             0 ];

B = [ ...
                0       8.99148;
          4.04892      0.553527;
       -0.0389187       -1.7967;
                0             0 ];

C = eye(4);
D = zeros(4, 2);
states = {'v (ft/s)', 'p (rad/s)', 'r (rad/s)', 'phi (rad)'};
inputs = {'delta_a (rad)', 'delta_r (rad)'};
fprintf('Squawk 1 loaded: A (4x4), B (4x2), C, D, u0 = %.1f ft/s\n', u0);
