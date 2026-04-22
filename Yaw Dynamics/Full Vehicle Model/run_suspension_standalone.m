function results = run_suspension_standalone()
%RUN_SUSPENSION_STANDALONE Build and run the phase-1 suspension model.
%   The script runs three baseline checks:
%   A) static drop / convergence
%   B) pure heave road input
%   C) left-right opposite road input

    close all;
    clc;

    thisDir = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(fileparts(thisDir));
    addpath(thisDir);
    addpath(fullfile(thisDir, 'Functions'));
    addpath(fullfile(repoRoot, 'Vertical Dynamics', 'Functions'));

    modelInfo = build_suspension_model();
    addpath(modelInfo.modelDir);
    load_system(modelInfo.modelPath);
    cleanupObj = onCleanup(@() close_system(modelInfo.modelName, 0)); %#ok<NASGU>

    car = get_suspension_default_params();
    curves = prepare_suspension_curves(car);

    time = (0:0.005:5).';
    cases = build_test_cases(time);

    caseResults = repmat(empty_case_result(), numel(cases), 1);
    for idx = 1:numel(cases)
        simOut = run_single_case(modelInfo.modelName, car, curves, cases(idx), time);
        caseResults(idx) = extract_case_result(simOut, cases(idx).name);
        print_case_summary(caseResults(idx));
    end

    carNoArb = car;
    carNoArb.frontARBStiffness = 0;
    carNoArb.rearARBStiffness = 0;
    comparisonCase = cases(3);
    comparisonCase.name = 'Test C (ARB Off)';
    comparisonCase.description = 'Left-right opposite road input with both ARB channels disabled.';
    comparisonResult = extract_case_result( ...
        run_single_case(modelInfo.modelName, carNoArb, curves, comparisonCase, time), ...
        comparisonCase.name);

    validate_results(caseResults, comparisonResult);
    plot_results(caseResults, comparisonResult);
    print_arb_comparison(caseResults(3), comparisonResult);

    results = struct();
    results.cases = caseResults;
    results.arbOffComparison = comparisonResult;
end

function cases = build_test_cases(time)
    zeroSignal = zeros(size(time));
    heaveRoad = sine_burst(time, 0.010, 1.8, 0.50, 3.00);
    rollRoad = sine_burst(time, 0.012, 1.6, 0.75, 2.75);

    cases(1) = struct( ...
        'name', 'Test A', ...
        'description', 'Static drop / convergence on flat road.', ...
        'z_road_FL', zeroSignal, ...
        'z_road_FR', zeroSignal, ...
        'z_road_RL', zeroSignal, ...
        'z_road_RR', zeroSignal, ...
        'ax', zeroSignal, ...
        'ay', zeroSignal, ...
        'aero_Fz_front', zeroSignal, ...
        'aero_Fz_rear', zeroSignal);

    cases(2) = struct( ...
        'name', 'Test B', ...
        'description', 'Pure heave road input with all four corners in phase.', ...
        'z_road_FL', heaveRoad, ...
        'z_road_FR', heaveRoad, ...
        'z_road_RL', heaveRoad, ...
        'z_road_RR', heaveRoad, ...
        'ax', zeroSignal, ...
        'ay', zeroSignal, ...
        'aero_Fz_front', zeroSignal, ...
        'aero_Fz_rear', zeroSignal);

    cases(3) = struct( ...
        'name', 'Test C', ...
        'description', 'Left-right opposite road input to excite roll and ARB coupling.', ...
        'z_road_FL', rollRoad, ...
        'z_road_FR', -rollRoad, ...
        'z_road_RL', rollRoad, ...
        'z_road_RR', -rollRoad, ...
        'ax', zeroSignal, ...
        'ay', zeroSignal, ...
        'aero_Fz_front', zeroSignal, ...
        'aero_Fz_rear', zeroSignal);
end

function signal = sine_burst(time, amplitude, frequency, startTime, duration)
    signal = zeros(size(time));
    activeMask = time >= startTime & time <= (startTime + duration);
    localTime = time(activeMask) - startTime;
    phase = localTime ./ duration;
    window = sin(pi * phase).^2;
    signal(activeMask) = amplitude .* sin(2 * pi * frequency .* localTime) .* window;
end

function simOut = run_single_case(modelName, car, curves, caseDef, time)
    inputNames = { ...
        'z_road_FL', 'z_road_FR', 'z_road_RL', 'z_road_RR', ...
        'ax', 'ay', 'aero_Fz_front', 'aero_Fz_rear'};

    dataset = Simulink.SimulationData.Dataset;
    for idx = 1:numel(inputNames)
        signalName = inputNames{idx};
        ts = timeseries(caseDef.(signalName), time);
        ts.Name = signalName;
        dataset = dataset.addElement(ts, signalName);
    end

    simInput = Simulink.SimulationInput(modelName);
    simInput = simInput.setVariable('car', car);
    simInput = simInput.setVariable('curves', curves);
    simInput = simInput.setExternalInput(dataset);
    simInput = simInput.setModelParameter( ...
        'StartTime', '0', ...
        'StopTime', num2str(time(end)), ...
        'SignalLogging', 'on', ...
        'SignalLoggingName', 'logsout');

    simOut = sim(simInput);
end

function result = extract_case_result(simOut, caseName)
    logsout = simOut.logsout;

    result = empty_case_result();
    result.name = caseName;

    heaveSignal = get_logged_signal(logsout, 'heave');
    result.time = heaveSignal.Time;
    result.heave = heaveSignal.Data;
    result.pitch = get_logged_signal(logsout, 'pitch').Data;
    result.roll = get_logged_signal(logsout, 'roll').Data;

    result.Fz_FL = get_logged_signal(logsout, 'Fz_FL').Data;
    result.Fz_FR = get_logged_signal(logsout, 'Fz_FR').Data;
    result.Fz_RL = get_logged_signal(logsout, 'Fz_RL').Data;
    result.Fz_RR = get_logged_signal(logsout, 'Fz_RR').Data;

    result.wheel_travel_FL = get_logged_signal(logsout, 'wheel_travel_FL').Data;
    result.wheel_travel_FR = get_logged_signal(logsout, 'wheel_travel_FR').Data;
    result.wheel_travel_RL = get_logged_signal(logsout, 'wheel_travel_RL').Data;
    result.wheel_travel_RR = get_logged_signal(logsout, 'wheel_travel_RR').Data;

    result.damper_travel_FL = get_logged_signal(logsout, 'damper_travel_FL').Data;
    result.damper_travel_FR = get_logged_signal(logsout, 'damper_travel_FR').Data;
    result.damper_travel_RL = get_logged_signal(logsout, 'damper_travel_RL').Data;
    result.damper_travel_RR = get_logged_signal(logsout, 'damper_travel_RR').Data;

    result.damper_vel_FL = get_logged_signal(logsout, 'damper_vel_FL').Data;
    result.damper_vel_FR = get_logged_signal(logsout, 'damper_vel_FR').Data;
    result.damper_vel_RL = get_logged_signal(logsout, 'damper_vel_RL').Data;
    result.damper_vel_RR = get_logged_signal(logsout, 'damper_vel_RR').Data;

    settledMask = result.time >= (result.time(end) - 0.5);
    if ~any(settledMask)
        settledMask = result.time >= 0.9 * result.time(end);
    end

    result.summary.settledLoads = [ ...
        mean(result.Fz_FL(settledMask)), ...
        mean(result.Fz_FR(settledMask)), ...
        mean(result.Fz_RL(settledMask)), ...
        mean(result.Fz_RR(settledMask))];
    result.summary.maxAbsRoll = max(abs(result.roll));
    result.summary.maxAbsPitch = max(abs(result.pitch));
    result.summary.maxAbsHeave = max(abs(result.heave));
    result.summary.maxWheelTravel = max(abs([ ...
        result.wheel_travel_FL, result.wheel_travel_FR, ...
        result.wheel_travel_RL, result.wheel_travel_RR]), [], 1);
    result.summary.maxDamperVel = max(abs([ ...
        result.damper_vel_FL, result.damper_vel_FR, ...
        result.damper_vel_RL, result.damper_vel_RR]), [], 1);
end

function signal = get_logged_signal(logsout, signalName)
    element = logsout.getElement(signalName);
    if isempty(element)
        error('Missing logged signal "%s".', signalName);
    end

    signal = struct();
    signal.Time = element.Values.Time(:);
    signal.Data = element.Values.Data(:);
end

function validate_results(caseResults, comparisonResult)
    staticLoads = caseResults(1).summary.settledLoads;
    if any(staticLoads <= 0)
        error('Test A failed: settled wheel loads must remain positive.');
    end

    maxRollHeave = max(abs(caseResults(2).roll));
    if maxRollHeave > 1e-3
        error('Test B failed: pure heave input should keep roll close to zero.');
    end

    if caseResults(3).summary.maxAbsRoll < 1e-4
        error('Test C failed: opposite road input did not produce a clear roll response.');
    end

    if comparisonResult.summary.maxAbsRoll <= 0
        error('ARB-off comparison produced an invalid roll response.');
    end
end

function plot_results(caseResults, comparisonResult)
    plot_loads(caseResults);
    plot_body_states(caseResults, comparisonResult);
    plot_travels(caseResults);
end

function plot_loads(caseResults)
    figure('Name', 'SuspensionModelBase - Wheel Loads', 'Color', 'w', ...
        'Position', [80 80 1100 900]);
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    for idx = 1:numel(caseResults)
        nexttile;
        plot(caseResults(idx).time, caseResults(idx).Fz_FL, 'LineWidth', 1.2);
        hold on;
        plot(caseResults(idx).time, caseResults(idx).Fz_FR, 'LineWidth', 1.2);
        plot(caseResults(idx).time, caseResults(idx).Fz_RL, 'LineWidth', 1.2);
        plot(caseResults(idx).time, caseResults(idx).Fz_RR, 'LineWidth', 1.2);
        grid on;
        title(sprintf('%s: Wheel Loads', caseResults(idx).name));
        ylabel('Fz [N]');
        legend({'FL', 'FR', 'RL', 'RR'}, 'Location', 'eastoutside');
    end
    xlabel('Time [s]');
end

function plot_body_states(caseResults, comparisonResult)
    figure('Name', 'SuspensionModelBase - Body States', 'Color', 'w', ...
        'Position', [120 120 1100 900]);
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    for idx = 1:numel(caseResults)
        nexttile;
        plot(caseResults(idx).time, caseResults(idx).heave * 1000, 'LineWidth', 1.2);
        hold on;
        plot(caseResults(idx).time, rad2deg(caseResults(idx).pitch), 'LineWidth', 1.2);
        plot(caseResults(idx).time, rad2deg(caseResults(idx).roll), 'LineWidth', 1.2);
        if idx == 3
            plot(comparisonResult.time, rad2deg(comparisonResult.roll), '--', 'LineWidth', 1.2);
            legend({'Heave [mm]', 'Pitch [deg]', 'Roll [deg]', 'Roll ARB off [deg]'}, 'Location', 'eastoutside');
        else
            legend({'Heave [mm]', 'Pitch [deg]', 'Roll [deg]'}, 'Location', 'eastoutside');
        end
        grid on;
        title(sprintf('%s: Body Response', caseResults(idx).name));
    end
    xlabel('Time [s]');
end

function plot_travels(caseResults)
    figure('Name', 'SuspensionModelBase - Wheel Travel', 'Color', 'w', ...
        'Position', [160 160 1100 900]);
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    for idx = 1:numel(caseResults)
        nexttile;
        plot(caseResults(idx).time, caseResults(idx).wheel_travel_FL * 1000, 'LineWidth', 1.2);
        hold on;
        plot(caseResults(idx).time, caseResults(idx).wheel_travel_FR * 1000, 'LineWidth', 1.2);
        plot(caseResults(idx).time, caseResults(idx).wheel_travel_RL * 1000, 'LineWidth', 1.2);
        plot(caseResults(idx).time, caseResults(idx).wheel_travel_RR * 1000, 'LineWidth', 1.2);
        grid on;
        title(sprintf('%s: Wheel Travel', caseResults(idx).name));
        ylabel('Wheel travel [mm]');
        legend({'FL', 'FR', 'RL', 'RR'}, 'Location', 'eastoutside');
    end
    xlabel('Time [s]');
end

function print_case_summary(result)
    fprintf('\n%s\n', repmat('=', 1, 72));
    fprintf('%s\n', result.name);
    fprintf('  Settled loads [N] (FL/FR/RL/RR): %.1f / %.1f / %.1f / %.1f\n', ...
        result.summary.settledLoads(1), ...
        result.summary.settledLoads(2), ...
        result.summary.settledLoads(3), ...
        result.summary.settledLoads(4));
    fprintf('  Peak heave [mm]: %.3f\n', 1000 * result.summary.maxAbsHeave);
    fprintf('  Peak pitch [deg]: %.4f\n', rad2deg(result.summary.maxAbsPitch));
    fprintf('  Peak roll [deg]: %.4f\n', rad2deg(result.summary.maxAbsRoll));
    fprintf('  Max wheel travel [mm] (FL/FR/RL/RR): %.3f / %.3f / %.3f / %.3f\n', ...
        1000 * result.summary.maxWheelTravel(1), ...
        1000 * result.summary.maxWheelTravel(2), ...
        1000 * result.summary.maxWheelTravel(3), ...
        1000 * result.summary.maxWheelTravel(4));
end

function print_arb_comparison(resultWithArb, resultWithoutArb)
    fprintf('\n%s\n', repmat('-', 1, 72));
    fprintf('Test C ARB comparison\n');
    fprintf('  Peak roll with ARB    [deg]: %.4f\n', rad2deg(resultWithArb.summary.maxAbsRoll));
    fprintf('  Peak roll without ARB [deg]: %.4f\n', rad2deg(resultWithoutArb.summary.maxAbsRoll));
    fprintf('  Peak left-right front travel split with ARB    [mm]: %.3f\n', ...
        1000 * max(abs(resultWithArb.wheel_travel_FL - resultWithArb.wheel_travel_FR)));
    fprintf('  Peak left-right front travel split without ARB [mm]: %.3f\n', ...
        1000 * max(abs(resultWithoutArb.wheel_travel_FL - resultWithoutArb.wheel_travel_FR)));
end

function result = empty_case_result()
    result = struct( ...
        'name', '', ...
        'time', [], ...
        'Fz_FL', [], 'Fz_FR', [], 'Fz_RL', [], 'Fz_RR', [], ...
        'heave', [], 'pitch', [], 'roll', [], ...
        'wheel_travel_FL', [], 'wheel_travel_FR', [], 'wheel_travel_RL', [], 'wheel_travel_RR', [], ...
        'damper_travel_FL', [], 'damper_travel_FR', [], 'damper_travel_RL', [], 'damper_travel_RR', [], ...
        'damper_vel_FL', [], 'damper_vel_FR', [], 'damper_vel_RL', [], 'damper_vel_RR', [], ...
        'summary', struct());
end
