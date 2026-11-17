%% AE4531 Lab, Squawk 4: Phugoid at cruise
%  1) Load squawk4.m (A, B, C, D, u0)
%  2) Build squawk4_openloop.slx programmatically:
%       From Workspace (delta_e), From Workspace (delta_p) -> Mux -> State-Space (x0)
%       -> Gain (engineering units) -> Scope + To Workspace
%  3) Reproduce the flight test (controls fixed, du0 = +10 ft/s, 600 s) and
%     overlay it on the packet plot (if the packet image is present)
%  4) Build squawk4_pitchhold.slx: delta_e = delta_e_pilot + K_theta*dtheta,
%     altitude logged by integrating hdot = u0*dtheta - w
%  5) Sweep K_theta and plot phugoid and short-period damping ratio vs K_theta
%
%  Your original model (nov12SimulinkAE4531.slx) is not modified.

clear; close all; clc;
root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end
cd(root);
outdir = fullfile(root, 'squawk4_results');
if ~exist(outdir, 'dir'), mkdir(outdir); end

%% 1) Load the squawk
run(fullfile(root, 'squawk_data', 'squawk4.m'));   % A, B, C, D, u0, h, states, inputs
x0    = [10; 0; 0; 0];      % flight test IC: 10 ft/s fast
Tend  = 600;                % s
r2d   = 180/pi;

% Engineering-unit conversion: [du ft/s; w ft/s; q rad/s; dtheta rad]
%                           -> [du ft/s; alpha deg; q deg/s; theta deg]
Keng   = diag([1, r2d/u0, r2d, r2d]);
engNames = {'\Deltau (ft/s)', '\Delta\alpha (deg)', 'q (deg/s)', '\Delta\theta (deg)'};

% Pilot inputs (controls fixed for the flight test): [time, value]
tin           = [0; Tend];
delta_e_input = [tin, zeros(2,1)];    % rad
delta_p_input = [tin, zeros(2,1)];    % lb
K_theta       = 0;                    % pitch-hold gain (rad elevator / rad pitch)

%% 2) Build the open-loop model
mdl = 'squawk4_openloop';
build_openloop(mdl, root);

%% 3) Flight test reproduction
so  = sim(mdl, 'StopTime', num2str(Tend), 'ReturnWorkspaceOutputs', 'on');
t   = so.yeng.Time;
yE  = squeeze(so.yeng.Data);  if size(yE,1) == 4 && size(yE,2) ~= 4, yE = yE.'; end
xS  = squeeze(so.xout.Data);  if size(xS,1) == 4 && size(xS,2) ~= 4, xS = xS.'; end

% Independent check against lsim
% (variable-step output, so compare on a uniform grid)
tu  = (0:0.05:Tend).';
xl  = lsim(ss(A,B,C,D), zeros(numel(tu),2), tu, x0);
fprintf('Open loop: max |Simulink - lsim| state error = %.3g\n', ...
    max(abs(interp1(t, xS, tu) - xl), [], 'all'));

% Phugoid read off the trace (peaks of du)
[pk, ipk] = findpeaks(yE(:,1), t, 'MinPeakDistance', 50);
if numel(pk) >= 2
    Tp_meas  = mean(diff(ipk));
    dlog     = log(pk(1)/pk(2));
    zeta_ft  = dlog / sqrt(4*pi^2 + dlog^2);
    fprintf('From trace: phugoid period = %.1f s, zeta (log dec) = %.4f\n', Tp_meas, zeta_ft);
end
[wnOL, zOL] = damp(A);
[wnOL, iw] = sort(wnOL); zOL = zOL(iw);   % 1-2 phugoid, 3-4 short period
fprintf('Open-loop eig: phugoid wn = %.4f rad/s (T = %.1f s), zeta = %.4f | SP wn = %.3f, zeta = %.3f\n', ...
    wnOL(1), 2*pi/(wnOL(1)*sqrt(1 - zOL(1)^2)), zOL(1), wnOL(3), zOL(3));

f1 = figure('Name', 'Squawk 4 flight test reproduction', 'Position', [80 80 900 750]);
tl = tiledlayout(f1, 4, 1, 'TileSpacing', 'compact');
for k = 1:4
    nexttile; plot(t, yE(:,k), 'LineWidth', 1.4); grid on; ylabel(engNames{k});
end
xlabel(tl, 'Time (s)');
title(tl, 'Squawk 4: controls fixed, \Deltau_0 = +10 ft/s, M 0.85 / 35,000 ft (Simulink)');
exportgraphics(f1, fullfile(outdir, 'squawk4_flighttest.png'), 'Resolution', 150);

% ---- Overlay on the packet plot ------------------------------------------
% Save the packet's flight-test plot as squawk_data/squawk4_packet.png and set
% the calibration below: the pixel corners of the packet's plot box and the
% data limits printed on its axes. The script draws the Simulink trace on top.
packetImg = fullfile(root, 'squawk_data', 'squawk4_packet.png');
if exist(packetImg, 'file')
    pk_cal.xlim  = [0 600];        % packet x-axis limits (s)        <-- edit
    pk_cal.ylim  = [-10 10];       % packet y-axis limits (du, ft/s) <-- edit
    pk_cal.state = 1;              % which engineering output the packet shows
    overlay_packet(packetImg, pk_cal, t, yE(:, pk_cal.state), ...
        fullfile(outdir, 'squawk4_packet_overlay.png'));
else
    fprintf(['No packet image found (%s).\n' ...
             '  Save the packet plot there and re-run to get the overlay.\n'], packetImg);
end

%% 4) Pitch attitude hold + altitude
mdl2 = 'squawk4_pitchhold';
build_pitchhold(mdl2, root);

Klist = [0 0.03 0.2 0.5];   % 0.03 = design gain (see section 6)
f2 = figure('Name', 'Pitch attitude hold', 'Position', [100 60 900 850]);
tl2 = tiledlayout(f2, 5, 1, 'TileSpacing', 'compact');
ax = gobjects(5,1); for k = 1:5, ax(k) = nexttile(tl2); hold on; grid on; end
for K_theta = Klist
    so2 = sim(mdl2, 'StopTime', num2str(Tend), 'ReturnWorkspaceOutputs', 'on');
    t2  = so2.yeng.Time;
    y2  = squeeze(so2.yeng.Data); if size(y2,1) == 4 && size(y2,2) ~= 4, y2 = y2.'; end
    dh  = squeeze(so2.dh.Data);
    for k = 1:4, plot(ax(k), t2, y2(:,k), 'LineWidth', 1.2); end
    plot(ax(5), t2, h + dh, 'LineWidth', 1.2);

    % check against the analytic closed loop
    Acl = A + B(:,1)*K_theta*[0 0 0 1];
    xs2 = squeeze(so2.xout.Data); if size(xs2,1) == 4 && size(xs2,2) ~= 4, xs2 = xs2.'; end
    xl2 = lsim(ss(Acl, zeros(4,1), eye(4), 0), zeros(numel(tu),1), tu, x0);
    fprintf('K_theta = %5.2f: max |Simulink - closed-loop lsim| = %.3g, final dh = %+.1f ft\n', ...
        K_theta, max(abs(interp1(t2, xs2, tu) - xl2), [], 'all'), dh(end));
end
for k = 1:4, ylabel(ax(k), engNames{k}); end
ylabel(ax(5), 'h (ft)');
legend(ax(1), compose('K_\\theta = %.2f', Klist), 'Location', 'northeast');
xlabel(tl2, 'Time (s)');
title(tl2, 'Pitch attitude hold: \delta_e = \delta_{e,pilot} + K_\theta \Delta\theta, \Deltau_0 = +10 ft/s');
exportgraphics(f2, fullfile(outdir, 'squawk4_pitchhold_response.png'), 'Resolution', 150);

%% 5) K_theta sweep: damping ratios
Ksweep = linspace(0, 0.5, 2501);   % K_theta = 0 to 0.5, step 2e-4
zph = nan(size(Ksweep)); zsp = zph; wph = zph; wsp = zph;
P = nan(4, numel(Ksweep));
for i = 1:numel(Ksweep)
    p = eig(A + B(:,1)*Ksweep(i)*[0 0 0 1]);
    [~, idx] = sort(abs(p));  p = p(idx);   % two slowest = phugoid, two fastest = SP
    P(:,i) = p;
    [zph(i), wph(i)] = pair_damping(p(1:2));
    [zsp(i), wsp(i)] = pair_damping(p(3:4));
end
% where the phugoid stops oscillating (zeta = 1)
iCrit = find(zph >= 1, 1);
Kcrit = Ksweep(iCrit);

% Verify the sweep against a Simulink linearization at one gain
K_theta = 0.5;
[Al, ~, ~, ~] = linmod_states(mdl2);
fprintf('linmod check at K_theta = 0.5: max |eig(Simulink) - eig(analytic)| = %.3g\n', ...
    max(abs(sort(eig(Al)) - sort(eig(A + B(:,1)*0.5*[0 0 0 1])))));

%% 6) MIL-F-8785C, Class III, Category B (cruise) — see milspec.png
% CAP = wn_sp^2 / (n/alpha), n/alpha = (u0/g)/T_theta2. 1/T_theta2 is the
% short-period zero of q/delta_e; theta feedback to delta_e does not move it.
g     = 32.174;
zq    = zero(ss(A, B(:,1), [0 0 1 0], 0));
zq    = zq(abs(zq) > 1e-6);                    % drop the free s = 0 zero
invT2 = -real(zq(abs(zq) == max(abs(zq))));     % 1/T_theta2 (1/s)
invT2 = invT2(1);
nalpha = u0/g * invT2;                          % g/rad
CAP   = wsp.^2 / nalpha;
fprintf('\nMIL-F-8785C Class III Cat B: 1/T_theta2 = %.3f 1/s, n/alpha = %.2f g/rad\n', invT2, nalpha);

lvl_ph  = 1*(zph >= 0.04) + 2*(zph >= 0 & zph < 0.04) + 3*(zph < 0);
lvl_spz = 1*(zsp >= 0.30 & zsp <= 2) + 2*(zsp >= 0.20 & zsp < 0.30) + ...
          3*(zsp >= 0.15 & zsp < 0.20) + 4*(zsp < 0.15 | zsp > 2);
lvl_cap = 1*(CAP >= 0.085 & CAP <= 3.6) + 2*((CAP >= 0.038 & CAP < 0.085) | (CAP > 3.6 & CAP <= 10)) + ...
          3*(CAP > 10) + 4*(CAP < 0.038);
lvl     = max([lvl_ph; lvl_spz; lvl_cap], [], 1);   % airplane level = worst requirement
L1      = Ksweep(lvl == 1);
K_design = round(mean([min(L1) max(L1)]) / 0.005) * 0.005;   % middle of the Level 1 window
[~, jD]  = min(abs(Ksweep - K_design));
fprintf('Level 1 window (all longitudinal reqs): %.4f < K_theta <= %.4f\n', min(L1), max(L1));
for L = 2:3
    iL = find(lvl_spz >= L, 1);
    if isempty(iL)
        fprintf('Short period stays better than Level %d over K_theta = 0 to %.2f\n', L, Ksweep(end));
    else
        fprintf('Short period drops to Level %d at K_theta = %.4f\n', L, Ksweep(iL));
    end
end
lvlName = {'Level 1', 'Level 2', 'Level 3', 'worse than Level 3'};
for jj = [1 jD]
    fprintf(['  K_theta = %.3f: phugoid zeta = %.4f (%s) | SP zeta = %.3f (%s) | ' ...
             'CAP = %.3f (%s)  =>  %s\n'], Ksweep(jj), zph(jj), lvlName{lvl_ph(jj)}, ...
             zsp(jj), lvlName{lvl_spz(jj)}, CAP(jj), lvlName{lvl_cap(jj)}, lvlName{lvl(jj)});
end

f3 = figure('Name', 'Damping vs K_theta', 'Position', [120 120 1200 520]);
tl3 = tiledlayout(f3, 1, 2, 'TileSpacing', 'compact');
for tile = 1:2
    ax3 = nexttile(tl3); hold(ax3, 'on');
    patch(ax3, [min(L1) max(L1) max(L1) min(L1)], [0 0 3 3], [0.85 0.95 0.85], ...
        'EdgeColor', 'none', 'DisplayName', 'Level 1 window (MIL-F-8785C Cl III Cat B)');
    plot(ax3, Ksweep, zph, 'LineWidth', 1.8, 'DisplayName', 'Phugoid');
    plot(ax3, Ksweep, zsp, 'LineWidth', 1.8, 'DisplayName', 'Short period');
    yline(ax3, 0.04, '-.', 'phugoid L1: \zeta \geq 0.04', 'HandleVisibility', 'off');
    yline(ax3, 0.30, '-.', 'SP L1: \zeta \geq 0.30', 'HandleVisibility', 'off');
    yline(ax3, 0.20, '-.', 'SP L2: \zeta \geq 0.20', 'HandleVisibility', 'off');
    yline(ax3, 1, ':', '\zeta = 1', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'right');
    xline(ax3, K_design, '-', ['design K_\theta = ' num2str(K_design, '%.3f')], ...
        'HandleVisibility', 'off', 'LabelVerticalAlignment', 'top');
    plot(ax3, 0, zph(1), 'ko', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
    plot(ax3, 0, zsp(1), 'ko', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
    grid(ax3, 'on'); xlabel(ax3, 'K_\theta  (rad \delta_e per rad \Delta\theta)'); ylabel(ax3, '\zeta');
    if tile == 1
        xline(ax3, Kcrit, '--', ['phugoid critically damped, K_\theta = ' num2str(Kcrit, '%.3f')], ...
            'HandleVisibility', 'off', 'LabelVerticalAlignment', 'middle');
        xlim(ax3, [0 0.5]); ylim(ax3, [0 1.6]); title(ax3, 'Full sweep (\zeta > 1: two real roots)');
        legend(ax3, 'Location', 'southoutside', 'Orientation', 'horizontal');
    else
        xlim(ax3, [0 0.15]); ylim(ax3, [0 0.8]); title(ax3, 'Zoom: Level 1 window');
    end
end
title(tl3, 'Squawk 4: damping ratio vs pitch-attitude-hold gain K_\theta');
exportgraphics(f3, fullfile(outdir, 'squawk4_zeta_vs_Ktheta.png'), 'Resolution', 150);

f4 = figure('Name', 'Root locus vs K_theta', 'Position', [140 140 900 520]);
plot(real(P.'), imag(P.'), '.', 'MarkerSize', 4); hold on; grid on;
plot(real(P(:,1)), imag(P(:,1)), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);
xlabel('Real (1/s)'); ylabel('Imag (rad/s)');
title('Closed-loop poles, K_\theta = 0 \rightarrow 0.5 (x = open loop)');
exportgraphics(f4, fullfile(outdir, 'squawk4_rootlocus_Ktheta.png'), 'Resolution', 150);

T = table(Ksweep(:), zph(:), wph(:), zsp(:), wsp(:), ...
    'VariableNames', {'K_theta', 'zeta_phugoid', 'wn_phugoid', 'zeta_sp', 'wn_sp'});
writetable(T, fullfile(outdir, 'squawk4_Ktheta_sweep.csv'));
fprintf('Phugoid becomes critically damped at K_theta = %.4f\n', Kcrit);
for Kp = [0 0.05 0.1 0.2 0.3 0.4 0.5]
    [~, j] = min(abs(Ksweep - Kp));
    fprintf('  K_theta = %4.2f: zeta_ph = %6.3f (wn %.4f) | zeta_sp = %.3f (wn %.3f)\n', ...
        Ksweep(j), zph(j), wph(j), zsp(j), wsp(j));
end
K_theta = 0;   % leave the workspace in the flight-test configuration
fprintf('Figures and CSV written to %s\n', outdir);

%% ===================== local functions ==================================
function [z, wn] = pair_damping(p)
% Damping of the quadratic factor (s - p1)(s - p2) = s^2 + 2 z wn s + wn^2.
% For a complex pair this equals damp(); for two real roots it gives z > 1.
    wn = sqrt(real(p(1)*p(2)));
    z  = -real(p(1) + p(2)) / (2*wn);
end

function [Al, Bl, Cl, Dl] = linmod_states(mdl)
% Linearize the closed-loop state dynamics (inputs are From Workspace, so
% only A is meaningful; the altitude integrator state is removed).
    [Al, Bl, Cl, Dl] = linmod(mdl);
    % keep the 4 aircraft states (drop the altitude integrator: zero row/col)
    keep = any(Al, 1).';   % nothing feeds back from h, so its column is zero
    Al = Al(keep, keep);
end

function build_openloop(mdl, root)
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    f = fullfile(root, [mdl '.slx']);
    if exist(f, 'file'), delete(f); end
    new_system(mdl); open_system(mdl);
    add_block('simulink/Sources/From Workspace', [mdl '/delta_e (rad)'], ...
        'VariableName', 'delta_e_input', 'Position', [40 60 140 90]);
    add_block('simulink/Sources/From Workspace', [mdl '/delta_p (lb)'], ...
        'VariableName', 'delta_p_input', 'Position', [40 140 140 170]);
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], 'Inputs', '2', ...
        'Position', [200 50 205 180]);
    add_block('simulink/Continuous/State-Space', [mdl '/Aircraft'], ...
        'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D', 'InitialCondition', 'x0', ...
        'Position', [260 90 380 140]);
    add_block('simulink/Math Operations/Gain', [mdl '/To eng units'], ...
        'Gain', 'Keng', 'Multiplication', 'Matrix(K*u)', 'Position', [450 95 530 135]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [620 40 660 80]);
    add_block('simulink/Sinks/To Workspace', [mdl '/yeng'], 'VariableName', 'yeng', ...
        'SaveFormat', 'Timeseries', 'Position', [620 100 700 130]);
    add_block('simulink/Sinks/To Workspace', [mdl '/xout'], 'VariableName', 'xout', ...
        'SaveFormat', 'Timeseries', 'Position', [450 180 530 210]);
    add_line(mdl, 'delta_e (rad)/1', 'Mux/1', 'autorouting', 'on');
    add_line(mdl, 'delta_p (lb)/1', 'Mux/2', 'autorouting', 'on');
    add_line(mdl, 'Mux/1', 'Aircraft/1', 'autorouting', 'on');
    add_line(mdl, 'Aircraft/1', 'To eng units/1', 'autorouting', 'on');
    add_line(mdl, 'Aircraft/1', 'xout/1', 'autorouting', 'on');
    add_line(mdl, 'To eng units/1', 'Scope/1', 'autorouting', 'on');
    add_line(mdl, 'To eng units/1', 'yeng/1', 'autorouting', 'on');
    set_param(mdl, 'StopTime', '600', 'Solver', 'ode45', 'MaxStep', '0.05', ...
        'RelTol', '1e-6', 'AbsTol', '1e-8');
    save_system(mdl, f);
end

function build_pitchhold(mdl, root)
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    f = fullfile(root, [mdl '.slx']);
    if exist(f, 'file'), delete(f); end
    new_system(mdl); open_system(mdl);
    add_block('simulink/Sources/From Workspace', [mdl '/delta_e_pilot (rad)'], ...
        'VariableName', 'delta_e_input', 'Position', [30 50 140 80]);
    add_block('simulink/Sources/From Workspace', [mdl '/delta_p (lb)'], ...
        'VariableName', 'delta_p_input', 'Position', [30 170 140 200]);
    add_block('simulink/Math Operations/Sum', [mdl '/Sum de'], 'Inputs', '++', ...
        'IconShape', 'round', 'Position', [190 55 210 75]);
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], 'Inputs', '2', ...
        'Position', [250 50 255 210]);
    add_block('simulink/Continuous/State-Space', [mdl '/Aircraft'], ...
        'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D', 'InitialCondition', 'x0', ...
        'Position', [300 105 420 155]);
    add_block('simulink/Signal Routing/Selector', [mdl '/dtheta'], ...
        'InputPortWidth', '4', 'IndexParamArray', {'4'}, 'Indices', '4', ...
        'Position', [480 250 520 280]);
    add_block('simulink/Math Operations/Gain', [mdl '/K_theta'], 'Gain', 'K_theta', ...
        'Orientation', 'left', 'Position', [300 250 360 280]);
    % hdot = u0*dtheta - w  (row vector on the state output)
    add_block('simulink/Math Operations/Gain', [mdl '/hdot = u0*dtheta - w'], ...
        'Gain', '[0 -1 0 u0]', 'Multiplication', 'Matrix(K*u)', ...
        'Position', [480 320 560 350]);
    add_block('simulink/Continuous/Integrator', [mdl '/Integrate hdot'], ...
        'InitialCondition', '0', 'Position', [600 320 630 350]);
    add_block('simulink/Sinks/To Workspace', [mdl '/dh'], 'VariableName', 'dh', ...
        'SaveFormat', 'Timeseries', 'Position', [680 320 760 350]);
    add_block('simulink/Sinks/Scope', [mdl '/Altitude Scope'], 'Position', [680 380 720 410]);
    add_block('simulink/Math Operations/Gain', [mdl '/To eng units'], ...
        'Gain', 'Keng', 'Multiplication', 'Matrix(K*u)', 'Position', [480 110 560 150]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [650 60 690 100]);
    add_block('simulink/Sinks/To Workspace', [mdl '/yeng'], 'VariableName', 'yeng', ...
        'SaveFormat', 'Timeseries', 'Position', [650 120 730 150]);
    add_block('simulink/Sinks/To Workspace', [mdl '/xout'], 'VariableName', 'xout', ...
        'SaveFormat', 'Timeseries', 'Position', [650 190 730 220]);
    add_block('simulink/Sinks/To Workspace', [mdl '/delta_e'], 'VariableName', 'delta_e_cmd', ...
        'SaveFormat', 'Timeseries', 'Position', [250 0 330 25]);

    add_line(mdl, 'delta_e_pilot (rad)/1', 'Sum de/1', 'autorouting', 'on');
    add_line(mdl, 'Sum de/1', 'Mux/1', 'autorouting', 'on');
    add_line(mdl, 'Sum de/1', 'delta_e/1', 'autorouting', 'on');
    add_line(mdl, 'delta_p (lb)/1', 'Mux/2', 'autorouting', 'on');
    add_line(mdl, 'Mux/1', 'Aircraft/1', 'autorouting', 'on');
    add_line(mdl, 'Aircraft/1', 'To eng units/1', 'autorouting', 'on');
    add_line(mdl, 'Aircraft/1', 'xout/1', 'autorouting', 'on');
    add_line(mdl, 'Aircraft/1', 'dtheta/1', 'autorouting', 'on');
    add_line(mdl, 'Aircraft/1', 'hdot = u0*dtheta - w/1', 'autorouting', 'on');
    add_line(mdl, 'dtheta/1', 'K_theta/1', 'autorouting', 'on');
    add_line(mdl, 'K_theta/1', 'Sum de/2', 'autorouting', 'on');
    add_line(mdl, 'hdot = u0*dtheta - w/1', 'Integrate hdot/1', 'autorouting', 'on');
    add_line(mdl, 'Integrate hdot/1', 'dh/1', 'autorouting', 'on');
    add_line(mdl, 'Integrate hdot/1', 'Altitude Scope/1', 'autorouting', 'on');
    add_line(mdl, 'To eng units/1', 'Scope/1', 'autorouting', 'on');
    add_line(mdl, 'To eng units/1', 'yeng/1', 'autorouting', 'on');
    set_param(mdl, 'StopTime', '600', 'Solver', 'ode45', 'MaxStep', '0.05', ...
        'RelTol', '1e-6', 'AbsTol', '1e-8');
    save_system(mdl, f);
end

function overlay_packet(imgFile, cal, t, y, outFile)
% Draws the packet image filling the axes so its plot box lines up with
% cal.xlim / cal.ylim, then plots the Simulink trace on top. If the packet
% plot has margins, crop the image to its plot box before saving it.
    I = imread(imgFile);
    fo = figure('Name', 'Overlay on packet plot', 'Position', [160 160 900 500]);
    ax = axes(fo); hold(ax, 'on');
    image(ax, 'XData', cal.xlim, 'YData', fliplr(cal.ylim), 'CData', I);
    plot(ax, t, y, 'r', 'LineWidth', 1.6);
    set(ax, 'YDir', 'normal'); xlim(ax, cal.xlim); ylim(ax, cal.ylim);
    legend(ax, 'Simulink (this lab)', 'Location', 'northeast');
    title(ax, 'Squawk 4 flight test: Simulink trace over packet plot');
    exportgraphics(fo, outFile, 'Resolution', 150);
end
