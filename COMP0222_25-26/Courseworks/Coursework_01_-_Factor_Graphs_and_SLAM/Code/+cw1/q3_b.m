% Run q2b

import ebe.core.*;
import ebe.graphics.*;
import cw1.*;

% Locate figs output directory
figsDir = findFigsDir();

% Find, load and parse the configuration file
config = ebe.utils.readJSONFile('config/q3_a.json');

% Mid-run map snapshot (shorter run)
configMid = config;
configMid.maximumStepNumber = floor(config.maximumStepNumber / 2);
runMapSnapshot(configMid, fullfile(figsDir, 'q3_b_landmarks_mid.png'));

% Create the mainloop object, which manages everything
mainLoop = ebe.MainLoop(config);

% Create the simulator and register it
simulator = drivebot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the G2O SLAM system and register it
g2oSLAMSystem = drivebot.G2OSLAMSystem(config);
mainLoop.addEstimator(g2oSLAMSystem);

g2oPrunedSLAMSystem = drivebot.G2OSLAMSystem(config);
g2oPrunedSLAMSystem.setFixOlderPlatformVertices(20);
g2oPrunedSLAMSystem.setName('g2op-slam');
mainLoop.addEstimator(g2oPrunedSLAMSystem);

% Create the SLAM system and register it
ekfSLAMSystem = drivebot.EKFSLAMSystem(config);
mainLoop.addEstimator(ekfSLAMSystem);

% Collect performance data every 100 timesteps
mainLoop.setAccumulateResultsUpdatePeriod(25);

% Create the store for estimates
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up the figure in which we draw everything
fig = FigureManager.getFigure("Q2c");
clf
hold on
axis([-5 55 -5 55])
%axis([4 14 5 15])
axis equal

% Set up the views which show the output of the simulator
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorView = drivebot.SimulatorView(config, simulator);
%simulatorView.setCentreAxesOnTruth(true);
simulatorViewer.addView(simulatorView);
simulatorViewer.addView(drivebot.SLAMSystemView(config, ekfSLAMSystem));
simulatorViewer.addView(drivebot.SLAMSystemView(config, g2oSLAMSystem));
simulatorViewer.addView(drivebot.SLAMSystemView(config, g2oPrunedSLAMSystem));

% Register the viewer with the mainloop
mainLoop.addViewer(simulatorViewer);

mainLoop.setGraphicsUpdatePeriod(25);

% Run the main loop until it terminates
mainLoop.run();

% Save final map snapshot for report
FigureManager.getFigure("Q2c");
drawnow
saveas(gcf, fullfile(figsDir, 'q3_b_landmarks_final.png'));

% Plot out state information

stateLabels = {'$x$','$y$', '$\theta$'};
axisLabels = {'Position', 'Position', 'Angle (rad)'};

TEstimator = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;

for e = 1 : numel(resultsAccumulator.xEstStore)
    ebe.graphics.FigureManager.getFigure(sprintf('Results for %d', e));
    clf
    hold on

    PX = resultsAccumulator.PEstStore{e};
    X = resultsAccumulator.xEstStore{e};

    for f = 1 : 3
    
        subplot(3,1,f)
        sigmaBound = 2 * sqrt(PX(f, :));
        plot(TEstimator, -sigmaBound, 'r--', 'LineWidth', 2)
        hold on
        plot(TEstimator, sigmaBound, 'r--', 'LineWidth', 2)
        stateError = X(f, :) - XTrueHistory(f, :);
        if (f == 3)
            stateError = atan2(sin(stateError), cos(stateError));
        end
        plot(TEstimator, stateError, 'LineWidth', 2);
    
        % Work out the axes
        maxError = max(abs(stateError));
        bound = 1.1 * max(maxError, max(sigmaBound));
        axis([TEstimator(1) TEstimator(end) -bound bound])
        
        xlabel('Time (s)')
        ylabel(axisLabels{f}, 'Interpreter','latex')
        title(stateLabels{f}, 'Interpreter','latex')
    end
end

% Timing results
cw1.q3_a_show_results

function runMapSnapshot(config, savePath)
    import ebe.core.*;
    import ebe.graphics.*;
    import cw1.*;

    mainLoop = ebe.MainLoop(config);
    simulator = drivebot.Simulator(config);
    mainLoop.setEventGenerator(simulator);

    g2oSLAMSystem = drivebot.G2OSLAMSystem(config);
    mainLoop.addEstimator(g2oSLAMSystem);

    g2oPrunedSLAMSystem = drivebot.G2OSLAMSystem(config);
    g2oPrunedSLAMSystem.setFixOlderPlatformVertices(20);
    g2oPrunedSLAMSystem.setName('g2op-slam');
    mainLoop.addEstimator(g2oPrunedSLAMSystem);

    ekfSLAMSystem = drivebot.EKFSLAMSystem(config);
    mainLoop.addEstimator(ekfSLAMSystem);

    FigureManager.getFigure("Q2c");
    clf
    hold on
    axis([-5 55 -5 55])
    axis equal

    simulatorViewer = ebe.graphics.ViewManager(config);
    simulatorView = drivebot.SimulatorView(config, simulator);
    simulatorViewer.addView(simulatorView);
    simulatorViewer.addView(drivebot.SLAMSystemView(config, ekfSLAMSystem));
    simulatorViewer.addView(drivebot.SLAMSystemView(config, g2oSLAMSystem));
    simulatorViewer.addView(drivebot.SLAMSystemView(config, g2oPrunedSLAMSystem));
    mainLoop.addViewer(simulatorViewer);
    mainLoop.setGraphicsUpdatePeriod(25);

    mainLoop.run();

    drawnow
    saveas(gcf, savePath);
end

function figsDir = findFigsDir()
    figsDir = pwd;
    while ~exist(fullfile(figsDir, 'figs'), 'dir')
        parent = fileparts(figsDir);
        if strcmp(parent, figsDir)
            break
        end
        figsDir = parent;
    end
    figsDir = fullfile(figsDir, 'figs');
end
