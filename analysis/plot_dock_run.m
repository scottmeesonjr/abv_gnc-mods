function T = plot_dock_run(file)
%PLOT_DOCK_RUN  Time-history plots of one docking run from its telemetry log.
%
%   plot_dock_run()              newest dock_telemetry_*.csv in ~/abv_dock_logs
%   plot_dock_run('radial_p10')  newest telemetry file whose name contains 'radial_p10'
%   plot_dock_run(fullPath)      a specific file
%
%   Figure 1: x, y, yaw vs time.   Figure 2: vx, vy, yaw rate, speed vs time
%   (0.125 m/s alert line).   Figure 3: thruster command (fx, fy, tz) vs time.
%   Figure 4: face gap and closing speed vs time, with each approach shaded.
%   Returns the telemetry table.

logDir = fullfile(char(java.lang.System.getProperty('user.home')), 'abv_dock_logs');
if nargin < 1 || isempty(file)
    file = newest(fullfile(logDir, 'dock_telemetry_*.csv'));
elseif ~isfile(file)
    file = newest(fullfile(logDir, ['dock_telemetry_*' char(file) '*.csv']));
end

opts = detectImportOptions(file, 'TextType', 'string');
opts = setvartype(opts, 'step', 'string');
T = readtable(file, opts);
T = T(~isnan(T.x_m), :);          % drop the final run-ended row
t = T.t_s;
[~, name] = fileparts(file);
name = strrep(name, '_', '\_');

% shade the approach steps on every axis
approach = T.step == "APPROACH";
edges = diff([0; approach; 0]);
aStart = t(edges(1:end-1) == 1);
aEnd   = t(find(edges(2:end) == -1));

figure('Name', 'Position');
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile; plot(t, T.x_m); grid on; ylabel('x (m)'); shade(aStart, aEnd); title(['Position: ' name]);
nexttile; plot(t, T.y_m); grid on; ylabel('y (m)'); shade(aStart, aEnd);
nexttile; plot(t, T.yaw_deg); grid on; ylabel('yaw (deg)'); xlabel('Time (s)'); shade(aStart, aEnd);

figure('Name', 'Velocity');
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile; hold on; grid on;
plot(t, T.vx_m_s, 'DisplayName', 'v_x'); plot(t, T.vy_m_s, 'DisplayName', 'v_y');
plot(t, T.speed_m_s, 'k', 'LineWidth', 1.2, 'DisplayName', 'speed');
yline(0.125, 'r--', 'alert 0.125', 'HandleVisibility', 'off');
ylabel('m/s'); legend('Location', 'best'); shade(aStart, aEnd); title(['Velocity: ' name]);
nexttile; plot(t, T.wz_deg_s); grid on; ylabel('yaw rate (deg/s)'); xlabel('Time (s)'); shade(aStart, aEnd);

figure('Name', 'Control input');
hold on; grid on;
plot(t, T.fx_cmd, 'DisplayName', 'f_x'); plot(t, T.fy_cmd, 'DisplayName', 'f_y');
plot(t, T.tz_cmd, 'DisplayName', '\tau_z');
xlabel('Time (s)'); ylabel('Thruster command'); legend('Location', 'best'); shade(aStart, aEnd);
title(['Control input: ' name]);

figure('Name', 'Approach');
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile; plot(t, T.gap_m); grid on; ylabel('face gap (m)'); ylim([-0.02 max(0.6, max(T.gap_m))]);
shade(aStart, aEnd); title(['Approach: ' name ' (shaded = approach)']);
nexttile; hold on; grid on; plot(t, T.closing_m_s);
yline([0.05 0.10], 'k--', {'0.05', '0.10'}); ylabel('closing speed (m/s)'); xlabel('Time (s)'); shade(aStart, aEnd);
end

function shade(aStart, aEnd)
for k = 1:numel(aStart)
    xregion(aStart(k), aEnd(k), 'FaceColor', [1 0.75 0.2], 'FaceAlpha', 0.15, ...
            'HandleVisibility', 'off');
end
end

function f = newest(pattern)
d = dir(pattern);
if isempty(d)
    error('No file matches %s', pattern);
end
[~, i] = max([d.datenum]);
f = fullfile(d(i).folder, d(i).name);
end
