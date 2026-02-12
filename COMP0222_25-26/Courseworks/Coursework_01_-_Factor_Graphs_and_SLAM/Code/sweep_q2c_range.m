% Sweep detection range for Q2c and pick threshold where EKF ~ g2o
import ebe.graphics.*;

ranges = [4 6 8];

% Paths
codeDir = pwd; % .../Code
rootDir = fileparts(fileparts(fileparts(fileparts(codeDir))));
libDir = fullfile(rootDir, 'COMP0222_25-26', 'Libraries');
figDir = fullfile(rootDir, 'figs');
scenarioPath = fullfile(codeDir, '+cw1', 'config', 'q2_c', 'scenario.json');
configPath = fullfile(codeDir, '+cw1', 'config', 'q2_c.json');
summaryPath = fullfile(rootDir, 'q2c_range_summary.txt');

% Ensure required paths
addpath(libDir);
addpath(codeDir);

% Read original scenario/config
cfg = jsondecode(fileread(scenarioPath));
origRange = cfg.sensors.slam.detectionRange;

cfgMain = jsondecode(fileread(configPath));
origMaxSteps = cfgMain.maximumStepNumber;

cleanupObj = onCleanup(@() restoreAll(scenarioPath, cfg, origRange, configPath, cfgMain, origMaxSteps));

% Reduce steps for sweep to keep runtime reasonable
cfgMain.maximumStepNumber = 400;
fid = fopen(configPath, 'w');
fwrite(fid, jsonencode(cfgMain, 'PrettyPrint', true));
fclose(fid);

results = struct('range', {}, 'g2o_frac', {}, 'ekf_frac', {}, 'delta', {});

for i = 1:numel(ranges)
    r = ranges(i);
    cfg.sensors.slam.detectionRange = r;
    fid = fopen(scenarioPath, 'w');
    fwrite(fid, jsonencode(cfg, 'PrettyPrint', true));
    fclose(fid);

    % Run experiment
    cw1.q2_c;

    % Save figures for this range (use figure Name)
    for e = 1:2
        h = findall(0, 'Type', 'figure', 'Name', sprintf('Results for %d', e));
        if ~isempty(h)
            saveas(h(1), fullfile(figDir, sprintf('q2c_range_%02d_results%d.png', r, e)));
        end
    end

    % Compute consistency fraction (within 2-sigma) for each estimator
    XTrue = resultsAccumulator.xTrueStore;
    frac = zeros(1,2);
    for e = 1:2
        X = resultsAccumulator.xEstStore{e};
        PX = resultsAccumulator.PEstStore{e};
        inCount = 0;
        total = 0;
        for f = 1:3
            err = X(f,:) - XTrue(f,:);
            if f == 3
                err = atan2(sin(err), cos(err));
            end
            sigma = 2 * sqrt(PX(f,:));
            inCount = inCount + sum(abs(err) <= sigma);
            total = total + numel(err);
        end
        frac(e) = inCount / total;
    end

    results(end+1) = struct('range', r, 'g2o_frac', frac(1), 'ekf_frac', frac(2), 'delta', abs(frac(1)-frac(2))); %#ok<AGROW>
end

% Choose smallest range where EKF is close to g2o and reasonably consistent
chosen = [];
for i = 1:numel(results)
    if results(i).ekf_frac >= 0.90 && results(i).delta <= 0.05
        chosen = results(i);
        break;
    end
end

if isempty(chosen)
    % fallback: pick range with max EKF fraction, then smallest range
    [~, idx] = max([results.ekf_frac]);
    chosen = results(idx);
end

% Copy chosen figures to canonical names
copyfile(fullfile(figDir, sprintf('q2c_range_%02d_results1.png', chosen.range)), ...
         fullfile(figDir, 'q2c_range_threshold_g2o.png'));
copyfile(fullfile(figDir, sprintf('q2c_range_%02d_results2.png', chosen.range)), ...
         fullfile(figDir, 'q2c_range_threshold_ekf.png'));

% Write summary
fid = fopen(summaryPath, 'w');
fprintf(fid, 'chosen_range=%d\n', chosen.range);
fprintf(fid, 'g2o_frac=%.4f\n', chosen.g2o_frac);
fprintf(fid, 'ekf_frac=%.4f\n', chosen.ekf_frac);
fprintf(fid, 'delta=%.4f\n', chosen.delta);

fprintf(fid, '\nAll results:\n');
for i = 1:numel(results)
    fprintf(fid, 'range=%d g2o=%.4f ekf=%.4f delta=%.4f\n', ...
        results(i).range, results(i).g2o_frac, results(i).ekf_frac, results(i).delta);
end
fclose(fid);

fprintf('Q2c sweep complete. Chosen range = %d\n', chosen.range);

function restoreAll(scenarioPath, cfg, origRange, configPath, cfgMain, origMaxSteps)
    cfg.sensors.slam.detectionRange = origRange;
    fid = fopen(scenarioPath, 'w');
    fwrite(fid, jsonencode(cfg, 'PrettyPrint', true));
    fclose(fid);

    cfgMain.maximumStepNumber = origMaxSteps;
    fid = fopen(configPath, 'w');
    fwrite(fid, jsonencode(cfgMain, 'PrettyPrint', true));
    fclose(fid);
end
