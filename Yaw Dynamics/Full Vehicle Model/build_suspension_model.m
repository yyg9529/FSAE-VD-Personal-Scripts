function modelInfo = build_suspension_model()
%BUILD_SUSPENSION_MODEL Programmatically build the phase-1 suspension model.
%   The script recreates SuspensionModelBase.slx from scratch so the model
%   topology is reproducible and not dependent on hand-edited .slx content.

    thisDir = fileparts(mfilename('fullpath'));
    modelDir = fullfile(thisDir, 'Base Models');
    modelName = 'SuspensionModelBase';
    modelPath = fullfile(modelDir, [modelName '.slx']);

    if ~exist(modelDir, 'dir')
        mkdir(modelDir);
    end

    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end

    if exist(modelPath, 'file') == 2
        delete(modelPath);
    end

    load_system('simulink');
    new_system(modelName);
    cleanupObj = onCleanup(@() cleanup_build_model(modelName));

    configure_model(modelName);
    add_root_io(modelName);
    add_top_level_blocks(modelName);
    build_parameters_subsystem([modelName '/ParametersGeometry']);
    build_body_subsystem([modelName '/BodyDynamics']);
    build_corner_subsystem([modelName '/Corner_FL'], 'front');
    build_corner_subsystem([modelName '/Corner_FR'], 'front');
    build_corner_subsystem([modelName '/Corner_RL'], 'rear');
    build_corner_subsystem([modelName '/Corner_RR'], 'rear');
    build_arb_subsystem([modelName '/ARB_Front'], 'car.frontARBStiffness');
    build_arb_subsystem([modelName '/ARB_Rear'], 'car.rearARBStiffness');
    build_outputs_subsystem([modelName '/OutputsLogging']);
    connect_top_level(modelName);

    save_system(modelName, modelPath);
    close_system(modelName, 0);
    clear cleanupObj;

    modelInfo = struct();
    modelInfo.modelName = modelName;
    modelInfo.modelPath = modelPath;
    modelInfo.modelDir = modelDir;
end

function configure_model(modelName)
    set_param(modelName, ...
        'SolverType', 'Variable-step', ...
        'Solver', 'ode23t', ...
        'StopTime', '5', ...
        'ReturnWorkspaceOutputs', 'on', ...
        'SignalLogging', 'on', ...
        'SignalLoggingName', 'logsout', ...
        'LimitDataPoints', 'off', ...
        'SaveFormat', 'Dataset', ...
        'SimulationMode', 'normal');
end

function add_root_io(modelName)
    inputNames = get_input_names();
    outputNames = get_output_names();

    for idx = 1:numel(inputNames)
        y = 70 + (idx - 1) * 55;
        add_block('simulink/Sources/In1', [modelName '/' inputNames{idx}], ...
            'Position', [30 y 60 y + 14]);
    end

    for idx = 1:numel(outputNames)
        y = 40 + (idx - 1) * 45;
        add_block('simulink/Sinks/Out1', [modelName '/' outputNames{idx}], ...
            'Position', [1620 y 1650 y + 14]);
    end
end

function add_top_level_blocks(modelName)
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/ParametersGeometry'], ...
        'Position', [170 40 330 230]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/BodyDynamics'], ...
        'Position', [420 40 690 430]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Corner_FL'], ...
        'Position', [860 40 1130 200]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Corner_FR'], ...
        'Position', [860 240 1130 400]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/ARB_Front'], ...
        'Position', [860 440 1040 520]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Corner_RL'], ...
        'Position', [860 580 1130 740]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Corner_RR'], ...
        'Position', [860 780 1130 940]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/ARB_Rear'], ...
        'Position', [860 980 1040 1060]);
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/OutputsLogging'], ...
        'Position', [1280 80 1470 960]);

    derivativeNames = {'RoadVel_FL', 'RoadVel_FR', 'RoadVel_RL', 'RoadVel_RR'};
    for idx = 1:numel(derivativeNames)
        y = 70 + (idx - 1) * 55;
        add_block('simulink/Continuous/Derivative', [modelName '/' derivativeNames{idx}], ...
            'Position', [120 y 150 y + 30]);
    end
end

function connect_top_level(modelName)
    % Road inputs and road velocities
    connect(modelName, 'z_road_FL/1', 'RoadVel_FL/1');
    connect(modelName, 'z_road_FL/1', 'Corner_FL/3');
    connect(modelName, 'RoadVel_FL/1', 'Corner_FL/4');

    connect(modelName, 'z_road_FR/1', 'RoadVel_FR/1');
    connect(modelName, 'z_road_FR/1', 'Corner_FR/3');
    connect(modelName, 'RoadVel_FR/1', 'Corner_FR/4');

    connect(modelName, 'z_road_RL/1', 'RoadVel_RL/1');
    connect(modelName, 'z_road_RL/1', 'Corner_RL/3');
    connect(modelName, 'RoadVel_RL/1', 'Corner_RL/4');

    connect(modelName, 'z_road_RR/1', 'RoadVel_RR/1');
    connect(modelName, 'z_road_RR/1', 'Corner_RR/3');
    connect(modelName, 'RoadVel_RR/1', 'Corner_RR/4');

    % Body inputs
    connect(modelName, 'ax/1', 'BodyDynamics/5');
    connect(modelName, 'ay/1', 'BodyDynamics/6');
    connect(modelName, 'aero_Fz_front/1', 'BodyDynamics/15');
    connect(modelName, 'aero_Fz_rear/1', 'BodyDynamics/16');

    % Parameters and geometry
    connect(modelName, 'ParametersGeometry/1', 'BodyDynamics/7');
    connect(modelName, 'ParametersGeometry/2', 'BodyDynamics/8');
    connect(modelName, 'ParametersGeometry/3', 'BodyDynamics/9');
    connect(modelName, 'ParametersGeometry/4', 'BodyDynamics/10');
    connect(modelName, 'ParametersGeometry/5', 'BodyDynamics/11');
    connect(modelName, 'ParametersGeometry/6', 'BodyDynamics/12');
    connect(modelName, 'ParametersGeometry/7', 'BodyDynamics/13');
    connect(modelName, 'ParametersGeometry/8', 'BodyDynamics/14');

    % Body corner kinematics to corners
    connect(modelName, 'BodyDynamics/7', 'Corner_FL/1');
    connect(modelName, 'BodyDynamics/8', 'Corner_FL/2');
    connect(modelName, 'BodyDynamics/9', 'Corner_FR/1');
    connect(modelName, 'BodyDynamics/10', 'Corner_FR/2');
    connect(modelName, 'BodyDynamics/11', 'Corner_RL/1');
    connect(modelName, 'BodyDynamics/12', 'Corner_RL/2');
    connect(modelName, 'BodyDynamics/13', 'Corner_RR/1');
    connect(modelName, 'BodyDynamics/14', 'Corner_RR/2');

    % Corner suspension forces back to body
    connect(modelName, 'Corner_FL/1', 'BodyDynamics/1');
    connect(modelName, 'Corner_FR/1', 'BodyDynamics/2');
    connect(modelName, 'Corner_RL/1', 'BodyDynamics/3');
    connect(modelName, 'Corner_RR/1', 'BodyDynamics/4');

    % ARB coupling
    connect(modelName, 'Corner_FL/3', 'ARB_Front/1');
    connect(modelName, 'Corner_FR/3', 'ARB_Front/2');
    connect(modelName, 'ARB_Front/1', 'Corner_FL/5');
    connect(modelName, 'ARB_Front/2', 'Corner_FR/5');

    connect(modelName, 'Corner_RL/3', 'ARB_Rear/1');
    connect(modelName, 'Corner_RR/3', 'ARB_Rear/2');
    connect(modelName, 'ARB_Rear/1', 'Corner_RL/5');
    connect(modelName, 'ARB_Rear/2', 'Corner_RR/5');

    % Outputs logging subsystem inputs
    connect(modelName, 'Corner_FL/2', 'OutputsLogging/1');
    connect(modelName, 'Corner_FR/2', 'OutputsLogging/2');
    connect(modelName, 'Corner_RL/2', 'OutputsLogging/3');
    connect(modelName, 'Corner_RR/2', 'OutputsLogging/4');

    connect(modelName, 'BodyDynamics/1', 'OutputsLogging/5');
    connect(modelName, 'BodyDynamics/3', 'OutputsLogging/6');
    connect(modelName, 'BodyDynamics/5', 'OutputsLogging/7');

    connect(modelName, 'Corner_FL/3', 'OutputsLogging/8');
    connect(modelName, 'Corner_FR/3', 'OutputsLogging/9');
    connect(modelName, 'Corner_RL/3', 'OutputsLogging/10');
    connect(modelName, 'Corner_RR/3', 'OutputsLogging/11');

    connect(modelName, 'Corner_FL/4', 'OutputsLogging/12');
    connect(modelName, 'Corner_FR/4', 'OutputsLogging/13');
    connect(modelName, 'Corner_RL/4', 'OutputsLogging/14');
    connect(modelName, 'Corner_RR/4', 'OutputsLogging/15');

    connect(modelName, 'Corner_FL/5', 'OutputsLogging/16');
    connect(modelName, 'Corner_FR/5', 'OutputsLogging/17');
    connect(modelName, 'Corner_RL/5', 'OutputsLogging/18');
    connect(modelName, 'Corner_RR/5', 'OutputsLogging/19');

    outputNames = get_output_names();
    for idx = 1:numel(outputNames)
        lineHandle = add_line(modelName, ...
            sprintf('OutputsLogging/%d', idx), ...
            sprintf('%s/1', outputNames{idx}), ...
            'autorouting', 'on');
        set_param(lineHandle, 'Name', outputNames{idx});
        Simulink.sdi.markSignalForStreaming(lineHandle, 'on');
    end
end

function build_parameters_subsystem(subsystemPath)
    clear_subsystem(subsystemPath);

    outNames = {'a', 'b', 'halfTrackFront', 'halfTrackRear', 'g', 'sprungMass', 'pitchInertia', 'rollInertia'};
    add_named_outports(subsystemPath, outNames, 420, 40, 50);

    add_block('simulink/Sources/Constant', [subsystemPath '/CGx'], ...
        'Value', 'car.CGx', ...
        'Position', [40 40 90 60]);
    add_block('simulink/Sources/Constant', [subsystemPath '/Wheelbase'], ...
        'Value', 'car.wheelbase', ...
        'Position', [40 90 90 110]);
    add_block('simulink/Sources/Constant', [subsystemPath '/TrackFront'], ...
        'Value', 'car.trackFront', ...
        'Position', [40 140 90 160]);
    add_block('simulink/Sources/Constant', [subsystemPath '/TrackRear'], ...
        'Value', 'car.trackRear', ...
        'Position', [40 190 90 210]);
    add_block('simulink/Sources/Constant', [subsystemPath '/Gravity'], ...
        'Value', '9.81', ...
        'Position', [40 240 90 260]);
    add_block('simulink/Sources/Constant', [subsystemPath '/SprungMass'], ...
        'Value', 'car.sprungMass', ...
        'Position', [40 290 90 310]);
    add_block('simulink/Sources/Constant', [subsystemPath '/PitchInertia'], ...
        'Value', 'car.pitchInertia', ...
        'Position', [40 340 90 360]);
    add_block('simulink/Sources/Constant', [subsystemPath '/RollInertia'], ...
        'Value', 'car.rollInertia', ...
        'Position', [40 390 90 410]);

    add_block('simulink/Math Operations/Sum', [subsystemPath '/RearDistance'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [170 88 200 112]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/HalfTrackFrontCalc'], ...
        'Gain', '0.5', ...
        'Position', [170 138 210 162]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/HalfTrackRearCalc'], ...
        'Gain', '0.5', ...
        'Position', [170 188 210 212]);

    connect(subsystemPath, 'CGx/1', 'Out1/1');
    connect(subsystemPath, 'Wheelbase/1', 'RearDistance/1');
    connect(subsystemPath, 'CGx/1', 'RearDistance/2');
    connect(subsystemPath, 'RearDistance/1', 'Out2/1');
    connect(subsystemPath, 'TrackFront/1', 'HalfTrackFrontCalc/1');
    connect(subsystemPath, 'HalfTrackFrontCalc/1', 'Out3/1');
    connect(subsystemPath, 'TrackRear/1', 'HalfTrackRearCalc/1');
    connect(subsystemPath, 'HalfTrackRearCalc/1', 'Out4/1');
    connect(subsystemPath, 'Gravity/1', 'Out5/1');
    connect(subsystemPath, 'SprungMass/1', 'Out6/1');
    connect(subsystemPath, 'PitchInertia/1', 'Out7/1');
    connect(subsystemPath, 'RollInertia/1', 'Out8/1');
end

function build_body_subsystem(subsystemPath)
    clear_subsystem(subsystemPath);

    inNames = { ...
        'F_susp_FL', 'F_susp_FR', 'F_susp_RL', 'F_susp_RR', ...
        'ax', 'ay', 'a', 'b', 'halfTrackFront', 'halfTrackRear', ...
        'g', 'sprungMass', 'pitchInertia', 'rollInertia', ...
        'aero_Fz_front', 'aero_Fz_rear'};
    outNames = { ...
        'z_s', 'dz_s', 'theta', 'dtheta', 'phi', 'dphi', ...
        'z_body_FL', 'dz_body_FL', 'z_body_FR', 'dz_body_FR', ...
        'z_body_RL', 'dz_body_RL', 'z_body_RR', 'dz_body_RR'};

    add_named_inports(subsystemPath, inNames, 30, 40, 38);
    add_named_outports(subsystemPath, outNames, 1020, 40, 38);

    % Heave dynamics
    add_block('simulink/Math Operations/Product', [subsystemPath '/BodyWeight'], ...
        'Inputs', '**', ...
        'Position', [160 470 200 500]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/AeroTotal'], ...
        'Inputs', '++', ...
        'IconShape', 'rectangular', ...
        'Position', [160 520 190 550]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/HeaveForce'], ...
        'Inputs', '++++--', ...
        'IconShape', 'rectangular', ...
        'Position', [300 80 330 170]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/HeaveAccel'], ...
        'Inputs', '*/', ...
        'Position', [390 108 430 142]);
    add_block('simulink/Continuous/Integrator', [subsystemPath '/dz_s_int'], ...
        'Position', [500 105 530 135]);
    add_block('simulink/Continuous/Integrator', [subsystemPath '/z_s_int'], ...
        'Position', [590 105 620 135]);

    % Pitch dynamics
    add_block('simulink/Math Operations/Sum', [subsystemPath '/FrontAxleForce'], ...
        'Inputs', '++', ...
        'IconShape', 'rectangular', ...
        'Position', [160 220 190 250]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/RearAxleForce'], ...
        'Inputs', '++', ...
        'IconShape', 'rectangular', ...
        'Position', [160 270 190 300]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/FrontPitchMoment'], ...
        'Inputs', '**', ...
        'Position', [260 220 300 250]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/RearPitchMoment'], ...
        'Inputs', '**', ...
        'Position', [260 270 300 300]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/SuspPitchMoment'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [360 238 390 272]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/MassAx'], ...
        'Inputs', '**', ...
        'Position', [160 330 200 360]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/InertialPitch'], ...
        'Gain', '-car.CGh', ...
        'Position', [260 330 305 360]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/FrontAeroMoment'], ...
        'Inputs', '**', ...
        'Position', [160 560 200 590]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/RearAeroMoment'], ...
        'Inputs', '**', ...
        'Position', [160 610 200 640]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/AeroPitchMoment'], ...
        'Inputs', '-+', ...
        'IconShape', 'rectangular', ...
        'Position', [260 578 290 612]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/PitchMomentTotal'], ...
        'Inputs', '+++', ...
        'IconShape', 'rectangular', ...
        'Position', [460 248 490 292]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/PitchAccel'], ...
        'Inputs', '*/', ...
        'Position', [560 255 600 285]);
    add_block('simulink/Continuous/Integrator', [subsystemPath '/dtheta_int'], ...
        'Position', [670 255 700 285]);
    add_block('simulink/Continuous/Integrator', [subsystemPath '/theta_int'], ...
        'Position', [760 255 790 285]);

    % Roll dynamics
    add_block('simulink/Math Operations/Sum', [subsystemPath '/FrontRollDelta'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [160 700 190 730]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/RearRollDelta'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [160 750 190 780]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/FrontRollMoment'], ...
        'Inputs', '**', ...
        'Position', [260 700 300 730]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/RearRollMoment'], ...
        'Inputs', '**', ...
        'Position', [260 750 300 780]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/MassAy'], ...
        'Inputs', '**', ...
        'Position', [160 805 200 835]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/InertialRoll'], ...
        'Gain', 'car.CGh', ...
        'Position', [260 805 305 835]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/RollMomentTotal'], ...
        'Inputs', '+++', ...
        'IconShape', 'rectangular', ...
        'Position', [460 730 490 774]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/RollAccel'], ...
        'Inputs', '*/', ...
        'Position', [560 738 600 768]);
    add_block('simulink/Continuous/Integrator', [subsystemPath '/dphi_int'], ...
        'Position', [670 738 700 768]);
    add_block('simulink/Continuous/Integrator', [subsystemPath '/phi_int'], ...
        'Position', [760 738 790 768]);

    % Corner kinematics
    add_block('simulink/Math Operations/Product', [subsystemPath '/aTheta'], ...
        'Inputs', '**', ...
        'Position', [840 200 880 230]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/bTheta'], ...
        'Inputs', '**', ...
        'Position', [840 300 880 330]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/frontRollHeight'], ...
        'Inputs', '**', ...
        'Position', [840 635 880 665]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/rearRollHeight'], ...
        'Inputs', '**', ...
        'Position', [840 785 880 815]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/aDTheta'], ...
        'Inputs', '**', ...
        'Position', [840 240 880 270]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/bDTheta'], ...
        'Inputs', '**', ...
        'Position', [840 340 880 370]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/frontRollRate'], ...
        'Inputs', '**', ...
        'Position', [840 675 880 705]);
    add_block('simulink/Math Operations/Product', [subsystemPath '/rearRollRate'], ...
        'Inputs', '**', ...
        'Position', [840 825 880 855]);

    add_block('simulink/Math Operations/Sum', [subsystemPath '/zBodyFL'], ...
        'Inputs', '+++', ...
        'IconShape', 'rectangular', ...
        'Position', [925 40 955 74]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/dzBodyFL'], ...
        'Inputs', '+++', ...
        'IconShape', 'rectangular', ...
        'Position', [925 84 955 118]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/zBodyFR'], ...
        'Inputs', '++-', ...
        'IconShape', 'rectangular', ...
        'Position', [925 150 955 184]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/dzBodyFR'], ...
        'Inputs', '++-', ...
        'IconShape', 'rectangular', ...
        'Position', [925 194 955 228]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/zBodyRL'], ...
        'Inputs', '+-+', ...
        'IconShape', 'rectangular', ...
        'Position', [925 260 955 294]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/dzBodyRL'], ...
        'Inputs', '+-+', ...
        'IconShape', 'rectangular', ...
        'Position', [925 304 955 338]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/zBodyRR'], ...
        'Inputs', '+--', ...
        'IconShape', 'rectangular', ...
        'Position', [925 370 955 404]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/dzBodyRR'], ...
        'Inputs', '+--', ...
        'IconShape', 'rectangular', ...
        'Position', [925 414 955 448]);

    % Heave connections
    connect(subsystemPath, 'In1/1', 'HeaveForce/1');
    connect(subsystemPath, 'In2/1', 'HeaveForce/2');
    connect(subsystemPath, 'In3/1', 'HeaveForce/3');
    connect(subsystemPath, 'In4/1', 'HeaveForce/4');
    connect(subsystemPath, 'In12/1', 'BodyWeight/1');
    connect(subsystemPath, 'In11/1', 'BodyWeight/2');
    connect(subsystemPath, 'BodyWeight/1', 'HeaveForce/5');
    connect(subsystemPath, 'In15/1', 'AeroTotal/1');
    connect(subsystemPath, 'In16/1', 'AeroTotal/2');
    connect(subsystemPath, 'AeroTotal/1', 'HeaveForce/6');
    connect(subsystemPath, 'HeaveForce/1', 'HeaveAccel/1');
    connect(subsystemPath, 'In12/1', 'HeaveAccel/2');
    connect(subsystemPath, 'HeaveAccel/1', 'dz_s_int/1');
    connect(subsystemPath, 'dz_s_int/1', 'z_s_int/1');
    connect(subsystemPath, 'z_s_int/1', 'Out1/1');
    connect(subsystemPath, 'dz_s_int/1', 'Out2/1');

    % Pitch connections
    connect(subsystemPath, 'In1/1', 'FrontAxleForce/1');
    connect(subsystemPath, 'In2/1', 'FrontAxleForce/2');
    connect(subsystemPath, 'In3/1', 'RearAxleForce/1');
    connect(subsystemPath, 'In4/1', 'RearAxleForce/2');
    connect(subsystemPath, 'FrontAxleForce/1', 'FrontPitchMoment/1');
    connect(subsystemPath, 'In7/1', 'FrontPitchMoment/2');
    connect(subsystemPath, 'RearAxleForce/1', 'RearPitchMoment/1');
    connect(subsystemPath, 'In8/1', 'RearPitchMoment/2');
    connect(subsystemPath, 'FrontPitchMoment/1', 'SuspPitchMoment/1');
    connect(subsystemPath, 'RearPitchMoment/1', 'SuspPitchMoment/2');
    connect(subsystemPath, 'In12/1', 'MassAx/1');
    connect(subsystemPath, 'In5/1', 'MassAx/2');
    connect(subsystemPath, 'MassAx/1', 'InertialPitch/1');
    connect(subsystemPath, 'In15/1', 'FrontAeroMoment/1');
    connect(subsystemPath, 'In7/1', 'FrontAeroMoment/2');
    connect(subsystemPath, 'In16/1', 'RearAeroMoment/1');
    connect(subsystemPath, 'In8/1', 'RearAeroMoment/2');
    connect(subsystemPath, 'FrontAeroMoment/1', 'AeroPitchMoment/1');
    connect(subsystemPath, 'RearAeroMoment/1', 'AeroPitchMoment/2');
    connect(subsystemPath, 'SuspPitchMoment/1', 'PitchMomentTotal/1');
    connect(subsystemPath, 'InertialPitch/1', 'PitchMomentTotal/2');
    connect(subsystemPath, 'AeroPitchMoment/1', 'PitchMomentTotal/3');
    connect(subsystemPath, 'PitchMomentTotal/1', 'PitchAccel/1');
    connect(subsystemPath, 'In13/1', 'PitchAccel/2');
    connect(subsystemPath, 'PitchAccel/1', 'dtheta_int/1');
    connect(subsystemPath, 'dtheta_int/1', 'theta_int/1');
    connect(subsystemPath, 'theta_int/1', 'Out3/1');
    connect(subsystemPath, 'dtheta_int/1', 'Out4/1');

    % Roll connections
    connect(subsystemPath, 'In1/1', 'FrontRollDelta/1');
    connect(subsystemPath, 'In2/1', 'FrontRollDelta/2');
    connect(subsystemPath, 'In3/1', 'RearRollDelta/1');
    connect(subsystemPath, 'In4/1', 'RearRollDelta/2');
    connect(subsystemPath, 'FrontRollDelta/1', 'FrontRollMoment/1');
    connect(subsystemPath, 'In9/1', 'FrontRollMoment/2');
    connect(subsystemPath, 'RearRollDelta/1', 'RearRollMoment/1');
    connect(subsystemPath, 'In10/1', 'RearRollMoment/2');
    connect(subsystemPath, 'In12/1', 'MassAy/1');
    connect(subsystemPath, 'In6/1', 'MassAy/2');
    connect(subsystemPath, 'MassAy/1', 'InertialRoll/1');
    connect(subsystemPath, 'FrontRollMoment/1', 'RollMomentTotal/1');
    connect(subsystemPath, 'RearRollMoment/1', 'RollMomentTotal/2');
    connect(subsystemPath, 'InertialRoll/1', 'RollMomentTotal/3');
    connect(subsystemPath, 'RollMomentTotal/1', 'RollAccel/1');
    connect(subsystemPath, 'In14/1', 'RollAccel/2');
    connect(subsystemPath, 'RollAccel/1', 'dphi_int/1');
    connect(subsystemPath, 'dphi_int/1', 'phi_int/1');
    connect(subsystemPath, 'phi_int/1', 'Out5/1');
    connect(subsystemPath, 'dphi_int/1', 'Out6/1');

    % Corner kinematics shared terms
    connect(subsystemPath, 'In7/1', 'aTheta/1');
    connect(subsystemPath, 'theta_int/1', 'aTheta/2');
    connect(subsystemPath, 'In8/1', 'bTheta/1');
    connect(subsystemPath, 'theta_int/1', 'bTheta/2');
    connect(subsystemPath, 'In9/1', 'frontRollHeight/1');
    connect(subsystemPath, 'phi_int/1', 'frontRollHeight/2');
    connect(subsystemPath, 'In10/1', 'rearRollHeight/1');
    connect(subsystemPath, 'phi_int/1', 'rearRollHeight/2');

    connect(subsystemPath, 'In7/1', 'aDTheta/1');
    connect(subsystemPath, 'dtheta_int/1', 'aDTheta/2');
    connect(subsystemPath, 'In8/1', 'bDTheta/1');
    connect(subsystemPath, 'dtheta_int/1', 'bDTheta/2');
    connect(subsystemPath, 'In9/1', 'frontRollRate/1');
    connect(subsystemPath, 'dphi_int/1', 'frontRollRate/2');
    connect(subsystemPath, 'In10/1', 'rearRollRate/1');
    connect(subsystemPath, 'dphi_int/1', 'rearRollRate/2');

    % Corner position sums
    connect(subsystemPath, 'z_s_int/1', 'zBodyFL/1');
    connect(subsystemPath, 'aTheta/1', 'zBodyFL/2');
    connect(subsystemPath, 'frontRollHeight/1', 'zBodyFL/3');
    connect(subsystemPath, 'zBodyFL/1', 'Out7/1');

    connect(subsystemPath, 'dz_s_int/1', 'dzBodyFL/1');
    connect(subsystemPath, 'aDTheta/1', 'dzBodyFL/2');
    connect(subsystemPath, 'frontRollRate/1', 'dzBodyFL/3');
    connect(subsystemPath, 'dzBodyFL/1', 'Out8/1');

    connect(subsystemPath, 'z_s_int/1', 'zBodyFR/1');
    connect(subsystemPath, 'aTheta/1', 'zBodyFR/2');
    connect(subsystemPath, 'frontRollHeight/1', 'zBodyFR/3');
    connect(subsystemPath, 'zBodyFR/1', 'Out9/1');

    connect(subsystemPath, 'dz_s_int/1', 'dzBodyFR/1');
    connect(subsystemPath, 'aDTheta/1', 'dzBodyFR/2');
    connect(subsystemPath, 'frontRollRate/1', 'dzBodyFR/3');
    connect(subsystemPath, 'dzBodyFR/1', 'Out10/1');

    connect(subsystemPath, 'z_s_int/1', 'zBodyRL/1');
    connect(subsystemPath, 'bTheta/1', 'zBodyRL/2');
    connect(subsystemPath, 'rearRollHeight/1', 'zBodyRL/3');
    connect(subsystemPath, 'zBodyRL/1', 'Out11/1');

    connect(subsystemPath, 'dz_s_int/1', 'dzBodyRL/1');
    connect(subsystemPath, 'bDTheta/1', 'dzBodyRL/2');
    connect(subsystemPath, 'rearRollRate/1', 'dzBodyRL/3');
    connect(subsystemPath, 'dzBodyRL/1', 'Out12/1');

    connect(subsystemPath, 'z_s_int/1', 'zBodyRR/1');
    connect(subsystemPath, 'bTheta/1', 'zBodyRR/2');
    connect(subsystemPath, 'rearRollHeight/1', 'zBodyRR/3');
    connect(subsystemPath, 'zBodyRR/1', 'Out13/1');

    connect(subsystemPath, 'dz_s_int/1', 'dzBodyRR/1');
    connect(subsystemPath, 'bDTheta/1', 'dzBodyRR/2');
    connect(subsystemPath, 'rearRollRate/1', 'dzBodyRR/3');
    connect(subsystemPath, 'dzBodyRR/1', 'Out14/1');
end

function build_corner_subsystem(subsystemPath, axleName)
    clear_subsystem(subsystemPath);

    inNames = {'z_body', 'dz_body', 'z_road', 'dz_road', 'F_arb'};
    outNames = {'F_susp', 'Fz', 'wheel_travel', 'damper_travel', 'damper_vel'};

    add_named_inports(subsystemPath, inNames, 30, 50, 60);
    add_named_outports(subsystemPath, outNames, 760, 70, 60);

    if strcmp(axleName, 'front')
        springBreakpointExpr = 'curves.front.spring.x';
        springTableExpr = 'curves.front.spring.y';
        damperBreakpointExpr = 'curves.front.damper.x';
        damperTableExpr = 'curves.front.damper.y';
        tireStiffnessExpr = 'car.frontTireStiffness';
        tireDampingExpr = 'car.frontTireDamping';
        motionRatioExpr = 'car.frontMotionRatio';
        unsprungMassExpr = 'car.unsprungMassFront';
    else
        springBreakpointExpr = 'curves.rear.spring.x';
        springTableExpr = 'curves.rear.spring.y';
        damperBreakpointExpr = 'curves.rear.damper.x';
        damperTableExpr = 'curves.rear.damper.y';
        tireStiffnessExpr = 'car.rearTireStiffness';
        tireDampingExpr = 'car.rearTireDamping';
        motionRatioExpr = 'car.rearMotionRatio';
        unsprungMassExpr = 'car.unsprungMassRear';
    end

    add_block('simulink/Continuous/Integrator', [subsystemPath '/dz_u_int'], ...
        'Position', [520 160 550 190]);
    add_block('simulink/Continuous/Integrator', [subsystemPath '/z_u_int'], ...
        'Position', [610 160 640 190]);

    add_block('simulink/Math Operations/Sum', [subsystemPath '/WheelTravel'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [210 80 240 110]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/RelVelocity'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [210 140 240 170]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/DamperTravel'], ...
        'Gain', motionRatioExpr, ...
        'Position', [330 78 380 112]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/DamperVelocity'], ...
        'Gain', motionRatioExpr, ...
        'Position', [330 138 380 172]);

    add_block('simulink/Lookup Tables/1-D Lookup Table', [subsystemPath '/SpringLUT'], ...
        'BreakpointsForDimension1', springBreakpointExpr, ...
        'Table', springTableExpr, ...
        'InterpMethod', 'Linear', ...
        'ExtrapMethod', 'Clip', ...
        'Position', [420 60 480 120]);
    add_block('simulink/Lookup Tables/1-D Lookup Table', [subsystemPath '/DamperLUT'], ...
        'BreakpointsForDimension1', damperBreakpointExpr, ...
        'Table', damperTableExpr, ...
        'InterpMethod', 'Linear', ...
        'ExtrapMethod', 'Clip', ...
        'Position', [420 130 480 190]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/SuspensionForce'], ...
        'Inputs', '+++', ...
        'IconShape', 'rectangular', ...
        'Position', [560 60 590 110]);

    add_block('simulink/Math Operations/Sum', [subsystemPath '/TireDeflection'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [210 280 240 310]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/TireVelocity'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [210 340 240 370]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/TireSpring'], ...
        'Gain', tireStiffnessExpr, ...
        'Position', [330 278 380 312]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/TireDamper'], ...
        'Gain', tireDampingExpr, ...
        'Position', [330 338 380 372]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/TireForce'], ...
        'Inputs', '++', ...
        'IconShape', 'rectangular', ...
        'Position', [420 308 450 342]);

    add_block('simulink/Sources/Constant', [subsystemPath '/UnsprungWeight'], ...
        'Value', [unsprungMassExpr ' * 9.81'], ...
        'Position', [420 390 490 410]);
    add_block('simulink/Math Operations/Sum', [subsystemPath '/UnsprungNetForce'], ...
        'Inputs', '+--', ...
        'IconShape', 'rectangular', ...
        'Position', [520 310 550 360]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/UnsprungAccel'], ...
        'Gain', ['1 / (' unsprungMassExpr ')'], ...
        'Position', [610 320 670 350]);

    % Relative motion
    connect(subsystemPath, 'z_u_int/1', 'WheelTravel/1');
    connect(subsystemPath, 'In1/1', 'WheelTravel/2');
    connect(subsystemPath, 'WheelTravel/1', 'DamperTravel/1');
    connect(subsystemPath, 'WheelTravel/1', 'SpringLUT/1');
    connect(subsystemPath, 'WheelTravel/1', 'Out3/1');
    connect(subsystemPath, 'DamperTravel/1', 'Out4/1');

    connect(subsystemPath, 'dz_u_int/1', 'RelVelocity/1');
    connect(subsystemPath, 'In2/1', 'RelVelocity/2');
    connect(subsystemPath, 'RelVelocity/1', 'DamperVelocity/1');
    connect(subsystemPath, 'RelVelocity/1', 'DamperLUT/1');
    connect(subsystemPath, 'DamperVelocity/1', 'Out5/1');

    % Suspension force
    connect(subsystemPath, 'SpringLUT/1', 'SuspensionForce/1');
    connect(subsystemPath, 'DamperLUT/1', 'SuspensionForce/2');
    connect(subsystemPath, 'In5/1', 'SuspensionForce/3');
    connect(subsystemPath, 'SuspensionForce/1', 'Out1/1');

    % Tire force
    connect(subsystemPath, 'In3/1', 'TireDeflection/1');
    connect(subsystemPath, 'z_u_int/1', 'TireDeflection/2');
    connect(subsystemPath, 'TireDeflection/1', 'TireSpring/1');
    connect(subsystemPath, 'In4/1', 'TireVelocity/1');
    connect(subsystemPath, 'dz_u_int/1', 'TireVelocity/2');
    connect(subsystemPath, 'TireVelocity/1', 'TireDamper/1');
    connect(subsystemPath, 'TireSpring/1', 'TireForce/1');
    connect(subsystemPath, 'TireDamper/1', 'TireForce/2');
    connect(subsystemPath, 'TireForce/1', 'Out2/1');

    % Unsprung dynamics
    connect(subsystemPath, 'TireForce/1', 'UnsprungNetForce/1');
    connect(subsystemPath, 'SuspensionForce/1', 'UnsprungNetForce/2');
    connect(subsystemPath, 'UnsprungWeight/1', 'UnsprungNetForce/3');
    connect(subsystemPath, 'UnsprungNetForce/1', 'UnsprungAccel/1');
    connect(subsystemPath, 'UnsprungAccel/1', 'dz_u_int/1');
    connect(subsystemPath, 'dz_u_int/1', 'z_u_int/1');
end

function build_arb_subsystem(subsystemPath, stiffnessExpr)
    clear_subsystem(subsystemPath);

    inNames = {'wheel_travel_left', 'wheel_travel_right'};
    outNames = {'F_arb_left', 'F_arb_right'};

    add_named_inports(subsystemPath, inNames, 30, 50, 60);
    add_named_outports(subsystemPath, outNames, 420, 50, 60);

    add_block('simulink/Math Operations/Sum', [subsystemPath '/TravelDelta'], ...
        'Inputs', '+-', ...
        'IconShape', 'rectangular', ...
        'Position', [140 55 170 85]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/HalfARBForce'], ...
        'Gain', ['0.5 * (' stiffnessExpr ')'], ...
        'Position', [230 53 300 87]);
    add_block('simulink/Math Operations/Gain', [subsystemPath '/OppositeForce'], ...
        'Gain', '-1', ...
        'Position', [330 105 370 135]);

    connect(subsystemPath, 'In1/1', 'TravelDelta/1');
    connect(subsystemPath, 'In2/1', 'TravelDelta/2');
    connect(subsystemPath, 'TravelDelta/1', 'HalfARBForce/1');
    connect(subsystemPath, 'HalfARBForce/1', 'Out1/1');
    connect(subsystemPath, 'HalfARBForce/1', 'OppositeForce/1');
    connect(subsystemPath, 'OppositeForce/1', 'Out2/1');
end

function build_outputs_subsystem(subsystemPath)
    clear_subsystem(subsystemPath);

    signalNames = get_output_names();
    add_named_inports(subsystemPath, signalNames, 30, 40, 40);
    add_named_outports(subsystemPath, signalNames, 320, 40, 40);

    for idx = 1:numel(signalNames)
        connect(subsystemPath, sprintf('In%d/1', idx), sprintf('Out%d/1', idx));
    end
end

function add_named_inports(systemPath, names, x, yStart, yStep)
    for idx = 1:numel(names)
        y = yStart + (idx - 1) * yStep;
        add_block('simulink/Sources/In1', sprintf('%s/In%d', systemPath, idx), ...
            'Port', num2str(idx), ...
            'Position', [x y x + 30 y + 14]);
    end
end

function add_named_outports(systemPath, names, x, yStart, yStep)
    for idx = 1:numel(names)
        y = yStart + (idx - 1) * yStep;
        add_block('simulink/Sinks/Out1', sprintf('%s/Out%d', systemPath, idx), ...
            'Port', num2str(idx), ...
            'Position', [x y x + 30 y + 14]);
    end
end

function clear_subsystem(subsystemPath)
    lineHandles = find_system(subsystemPath, 'FindAll', 'on', 'Type', 'line');
    if ~isempty(lineHandles)
        delete_line(lineHandles);
    end

    blockPaths = find_system(subsystemPath, 'SearchDepth', 1, 'Type', 'Block');
    for idx = 1:numel(blockPaths)
        if strcmp(blockPaths{idx}, subsystemPath)
            continue;
        end
        delete_block(blockPaths{idx});
    end
end

function connect(systemPath, source, destination)
    add_line(systemPath, source, destination, 'autorouting', 'on');
end

function cleanup_build_model(modelName)
    if ~bdIsLoaded(modelName)
        return;
    end

    try
        close_system(modelName, 0);
    catch
        bdclose(modelName);
    end
end

function inputNames = get_input_names()
    inputNames = { ...
        'z_road_FL', 'z_road_FR', 'z_road_RL', 'z_road_RR', ...
        'ax', 'ay', 'aero_Fz_front', 'aero_Fz_rear'};
end

function outputNames = get_output_names()
    outputNames = { ...
        'Fz_FL', 'Fz_FR', 'Fz_RL', 'Fz_RR', ...
        'heave', 'pitch', 'roll', ...
        'wheel_travel_FL', 'wheel_travel_FR', 'wheel_travel_RL', 'wheel_travel_RR', ...
        'damper_travel_FL', 'damper_travel_FR', 'damper_travel_RL', 'damper_travel_RR', ...
        'damper_vel_FL', 'damper_vel_FR', 'damper_vel_RL', 'damper_vel_RR'};
end
