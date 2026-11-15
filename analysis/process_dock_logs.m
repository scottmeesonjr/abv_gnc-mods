function results = process_dock_logs(logDir)
%PROCESS_DOCK_LOGS  Summarize ABV docking logs and check them against the requirements.
%
%   results = process_dock_logs()          reads every dock_log_*.csv in ~/abv_dock_logs
%   results = process_dock_logs(folder)    reads every dock_log_*.csv in folder
%
%   One row per run (grouped by run_name; logs from before run names existed are
%   grouped by file). Prints a pass/fail table and draws three figures:
%     1. capture result vs. commanded radial and angular offset (SYS.07 / SYS.08)
%     2. contact speed vs. the 0.05-0.10 m/s band (SYS.10 / TEST.12)
%     3. start offset error vs. the TEST.10 / TEST.11 limits
%
%   Simulation logs DEMONSTRATE requirements; they do not verify them
%   (the sim is not NASA-STD-7009B assessed). Use hardware logs for verification.

if nargin < 1 || isempty(logDir)
    logDir = fullfile(char(java.lang.System.getProperty('user.home')), 'abv_dock_logs');
end

% ---------------- requirement limits ----------------
REQ.cycles      = 10;      % SYS.06 consecutive engagement cycles
REQ.radial      = 0.10;    % SYS.07 m
REQ.angle       = 4.0;     % SYS.08 deg
REQ.vMin        = 0.05;    % SYS.10 m/s
REQ.vMax        = 0.10;    % SYS.10 m/s
REQ.alert       = 0.125;   % SYS.01 m/s
REQ.tolRadial   = 0.015;   % TEST.10 m
REQ.tolAngle    = 1.0;     % TEST.11 deg
REQ.tolSpeed    = 0.005;   % TEST.12 m/s

% ---------------- read all logs ----------------
files = dir(fullfile(logDir, 'dock_log_*.csv'));
if isempty(files)
    error('No dock_log_*.csv files in %s', logDir);
end
T = table();
for k = 1:numel(files)
    f = fullfile(files(k).folder, files(k).name);
    opts = detectImportOptions(f, 'TextType', 'string');
    textCols = intersect({'run_name', 'wall_time', 'condition', 'result', 'alert_tripped'}, opts.VariableNames);
    opts = setvartype(opts, textCols, 'string');
    t = readtable(f, opts);
    if height(t) == 0, continue; end
    if ~ismember('run_name', t.Properties.VariableNames)
        t.run_name = repmat("", height(t), 1);
    end
    blank = strlength(t.run_name) == 0 | ismissing(t.run_name);
    [~, stem] = fileparts(files(k).name);
    t.run_name(blank) = string(stem);       % unnamed runs: one group per file
    t.file = repmat(string(files(k).name), height(t), 1);
    t = t(:, sort(t.Properties.VariableNames));
    if isempty(T), T = t; else, T = [T; t(:, T.Properties.VariableNames)]; end %#ok<AGROW>
end

T.captured = T.result == "CAPTURED";
% runs commanded inside the SYS.10 band; deliberate over-speed runs (alert
% tests) are kept out of the speed-accuracy checks
T.inBandCmd = T.commanded_speed_m_s >= REQ.vMin - 1e-6 & T.commanded_speed_m_s <= REQ.vMax + 1e-6;
T.alert    = lower(T.alert_tripped) == "true";
T.startRadialErr = abs(T.start_lateral_m - T.commanded_radial_m);
T.startAngleErr  = abs(T.start_angle_deg - T.commanded_angle_deg);
T.speedErr       = abs(T.contact_speed_m_s - T.commanded_speed_m_s);

% ---------------- per-run summary ----------------
runs = unique(T.run_name, 'stable');
n = numel(runs);
S = table('Size', [n 13], ...
    'VariableTypes', {'string','double','double','double','double','double','double', ...
                      'double','double','double','double','logical','double'}, ...
    'VariableNames', {'run','cycles','captured','longestStreak','radial_m','angle_deg', ...
                      'speed_cmd','contactV_min','contactV_max','maxStartRadialErr', ...
                      'maxStartAngleErr','alertTripped','maxSpeedErr'});
for i = 1:n
    r = T(T.run_name == runs(i), :);
    c = r(r.captured, :);
    S.run(i)               = runs(i);
    S.cycles(i)            = height(r);
    S.captured(i)          = height(c);
    S.longestStreak(i)     = longestRun(r.captured);
    S.radial_m(i)          = max(abs(r.commanded_radial_m));
    S.angle_deg(i)         = max(abs(r.commanded_angle_deg));
    S.speed_cmd(i)         = r.commanded_speed_m_s(1);
    S.contactV_min(i)      = minOrNaN(c.contact_speed_m_s);
    S.contactV_max(i)      = maxOrNaN(c.contact_speed_m_s);
    S.maxStartRadialErr(i) = max(r.startRadialErr);
    S.maxStartAngleErr(i)  = max(r.startAngleErr);
    S.alertTripped(i)      = any(r.alert);
    S.maxSpeedErr(i)       = maxOrNaN(c.speedErr);
end

fprintf('\n===== Docking runs (%s) =====\n', logDir);
disp(S);

% ---------------- requirement checks ----------------
allCap = T(T.captured, :);
check = {};
check(end+1, :) = {'SYS.06', sprintf('%d consecutive captures in one run', REQ.cycles), ...
    any(S.longestStreak >= REQ.cycles), sprintf('longest streak %d', max(S.longestStreak))};
atRadial = T(abs(T.commanded_radial_m) >= REQ.radial - 1e-6, :);
check(end+1, :) = {'SYS.07', sprintf('capture at %.2f m radial offset', REQ.radial), ...
    ~isempty(atRadial) && all(atRadial.captured), coverage(atRadial)};
atAngle = T(abs(T.commanded_angle_deg) >= REQ.angle - 1e-6, :);
check(end+1, :) = {'SYS.08', sprintf('capture at %.0f deg angular offset', REQ.angle), ...
    ~isempty(atAngle) && all(atAngle.captured), coverage(atAngle)};
% SYS.10: captured at both band edges, and every in-band contact within the
% TEST.12 tolerance of the band. A run commanded at exactly 0.10 m/s lands a
% few mm/s either side of it; anything within the tolerance counts as the edge
% (and docking slightly above 0.10 is conservative for this requirement).
bandCap = allCap(allCap.inBandCmd, :);
inBand = bandCap.contact_speed_m_s >= REQ.vMin - REQ.tolSpeed & ...
         bandCap.contact_speed_m_s <= REQ.vMax + REQ.tolSpeed;
hasLow  = any(allCap.inBandCmd & abs(allCap.commanded_speed_m_s - REQ.vMin) < 1e-6);
hasHigh = any(allCap.inBandCmd & abs(allCap.commanded_speed_m_s - REQ.vMax) < 1e-6);
check(end+1, :) = {'SYS.10', 'captured at 0.05 and 0.10 m/s, contacts in band (+/- TEST.12 tol)', ...
    ~isempty(bandCap) && all(inBand) && hasLow && hasHigh, ...
    sprintf('%d/%d in band; 0.05 captured: %s; 0.10 captured: %s', sum(inBand), height(bandCap), yn(hasLow), yn(hasHigh))};
% SYS.01: no alert while within the band, and an alert every time a cycle
% actually went above the 0.125 m/s threshold
belowAlert = T(T.max_speed_m_s <= REQ.alert, :);
aboveAlert = T(T.max_speed_m_s > REQ.alert, :);
check(end+1, :) = {'SYS.01', 'no false alert below 0.125 m/s', ...
    ~any(belowAlert.alert), sprintf('%d false alerts in %d cycles', sum(belowAlert.alert), height(belowAlert))};
if isempty(aboveAlert)
    tripText = 'no cycle went above 0.125 m/s yet';
else
    tripText = sprintf('%d/%d over-speed cycles flagged', sum(aboveAlert.alert), height(aboveAlert));
end
check(end+1, :) = {'SYS.01', 'alert trips above 0.125 m/s (TEST.07)', ...
    ~isempty(aboveAlert) && all(aboveAlert.alert), tripText};
check(end+1, :) = {'TEST.10', sprintf('start radial within %.3f m', REQ.tolRadial), ...
    all(T.startRadialErr <= REQ.tolRadial), sprintf('worst %.4f m', max(T.startRadialErr))};
check(end+1, :) = {'TEST.11', sprintf('start angle within %.1f deg', REQ.tolAngle), ...
    all(T.startAngleErr <= REQ.tolAngle), sprintf('worst %.2f deg', max(T.startAngleErr))};
check(end+1, :) = {'TEST.12', sprintf('contact speed within %.3f m/s of command', REQ.tolSpeed), ...
    ~isempty(bandCap) && all(bandCap.speedErr <= REQ.tolSpeed), sprintf('worst %.4f m/s (in-band runs)', maxOrNaN(bandCap.speedErr))};

C = cell2table(check, 'VariableNames', {'Requirement', 'Check', 'Pass', 'Evidence'});
fprintf('\n===== Requirement checks =====\n');
for i = 1:height(C)
    fprintf('%-8s %-4s %-52s %s\n', C.Requirement{i}, passText(C.Pass(i)), C.Check{i}, C.Evidence{i});
end
fprintf('\n');

% ---------------- figures ----------------
figure('Name', 'Capture vs offset');
hold on; grid on;
plot(T.commanded_radial_m(T.captured)*1000, T.commanded_angle_deg(T.captured), 'o', ...
     'MarkerFaceColor', [0.2 0.6 0.3], 'MarkerEdgeColor', 'none', 'DisplayName', 'Captured');
plot(T.commanded_radial_m(~T.captured)*1000, T.commanded_angle_deg(~T.captured), 'x', ...
     'Color', [0.8 0.2 0.2], 'LineWidth', 1.5, 'DisplayName', 'No lock');
rectangle('Position', [-REQ.radial*1000, -REQ.angle, 2*REQ.radial*1000, 2*REQ.angle], 'LineStyle', '--');
xlabel('Commanded radial offset at the docking face (mm)'); ylabel('Commanded angular offset (deg)');
title('Capture vs. commanded offset (dashed box: SYS.07 / SYS.08)'); legend('Location', 'best');

figure('Name', 'Contact speed');
hold on; grid on;
histogram(allCap.contact_speed_m_s, 'BinWidth', 0.0025);
xlim([0.04 0.14]);
xline(REQ.vMin, 'k--', '0.05'); xline(REQ.vMax, 'k--', '0.10'); xline(REQ.alert, 'r--', 'alert 0.125');
xlabel('Contact speed (m/s)'); ylabel('Cycles'); title('Contact speed (SYS.10)');

figure('Name', 'Start offset error');
hold on; grid on;
for i = 1:numel(runs)
    r = T(T.run_name == runs(i), :);
    plot(r.startRadialErr*1000, r.startAngleErr, 'o', 'MarkerFaceColor', 'auto', ...
         'DisplayName', char(runs(i)));
end
xline(REQ.tolRadial*1000, 'k--', 'TEST.10', 'HandleVisibility', 'off');
yline(REQ.tolAngle, 'k--', 'TEST.11', 'HandleVisibility', 'off');
xlabel('|Start radial error| (mm)'); ylabel('|Start angle error| (deg)');
title('How closely the commanded offsets were set up');
legend('Interpreter', 'none', 'Location', 'best');

results.cycles = T;
results.runs = S;
results.checks = C;
end

% ---------------- helpers ----------------
function k = longestRun(tf)
k = 0; cur = 0;
for i = 1:numel(tf)
    if tf(i), cur = cur + 1; k = max(k, cur); else, cur = 0; end
end
end

function v = minOrNaN(x)
if isempty(x), v = NaN; else, v = min(x); end
end

function v = maxOrNaN(x)
if isempty(x), v = NaN; else, v = max(x); end
end

function s = coverage(r)
if isempty(r), s = 'not run yet'; else, s = sprintf('%d/%d captured', sum(r.captured), height(r)); end
end

function s = yn(tf)
if tf, s = 'yes'; else, s = 'no'; end
end

function s = passText(tf)
if tf, s = 'PASS'; else, s = '----'; end
end
