%% Squawk 6: Dutch roll at low speed, yaw damper ON at cruise settings
%  737-class transport, flight test aircraft T-1. 560 ft/s (M 0.576), 35,000 ft, C_L = 0.94, airplane as designed.
%  Linear model, stability axes. Run this script to load A, B, C, D, u0.
%
% States  x = [v (ft/s); p (rad/s); r (rad/s); phi (rad)]      Etkin (4.9,19)
% Inputs  c = [delta_a (rad); delta_r (rad)]                    Etkin (7.9,3)
% Signs: +delta_a rolls RIGHT (C_l_da = +0.06; opposite to Etkin's B747).
%        +delta_r is trailing edge left, nose LEFT (C_n_dr = -0.09).
% Sideslip: beta = v/u0.
%
% Yaw damper (cruise settings): delta_r = delta_r_pilot + Kr*[tau_w*s/(tau_w*s + 1)]*r, Kr = 1.0, tau_w = 2.0 s.
% Flight test input: hands off, initial sideslip of 1 deg: x0 = [u0*deg2rad(1); 0; 0; 0] (washout state 0), 40 s, damper ON.
Kr = 1.0;  tau_w = 2.0;

u0 = 560;          % ft/s
h  = 35000;            % ft

A = [ ...
        -0.046021     -0.172732      -558.791        32.174;
       -0.0065497      -1.40204      0.412737             0;
       0.00142154     -0.169418     -0.244615             0;
                0             1             0             0 ];

B = [ ...
                0       4.12348;
          1.82542     0.0653116;
          0.10382     -0.831947;
                0             0 ];

C = eye(4);
D = zeros(4, 2);
states = {'v (ft/s)', 'p (rad/s)', 'r (rad/s)', 'phi (rad)'};
inputs = {'delta_a (rad)', 'delta_r (rad)'};
fprintf('Squawk 6 loaded: A (4x4), B (4x2), C, D, u0 = %.1f ft/s\n', u0);
