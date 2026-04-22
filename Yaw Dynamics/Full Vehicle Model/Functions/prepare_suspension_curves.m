function curves = prepare_suspension_curves(car, options)
%PREPARE_SUSPENSION_CURVES Prepare spring and damper LUTs for phase-1 model.
%   CURVES = PREPARE_SUSPENSION_CURVES(CAR) reuses the existing
%   Vertical Dynamics Excel data and helper functions without modifying the
%   original half-car workflow.
%
%   Current phase-1 assumption:
%   the generated LUTs are treated as wheel-side equivalent force curves in
%   SuspensionModelBase. This matches the legacy helpers, which scale the
%   output force with motionRatio^2 but do not rescale the LUT abscissa.

    if nargin < 2
        options = struct();
    end

    thisDir = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(fileparts(fileparts(thisDir)));
    verticalDir = fullfile(repoRoot, 'Vertical Dynamics');
    verticalFunctionsDir = fullfile(verticalDir, 'Functions');

    if exist('SetSpringCurve', 'file') ~= 2 || exist('SetDamperClick', 'file') ~= 2
        addpath(verticalFunctionsDir);
    end

    defaults = struct( ...
        'springTableFile', fullfile(verticalDir, 'SpringTable.xlsx'), ...
        'damperTableFile', fullfile(verticalDir, 'DamperTable.xlsx'), ...
        'frontSpringSheet', 'linear_350', ...
        'rearSpringSheet', 'linear_300', ...
        'damperSheet', 'Multimatic_DSSV_VC01', ...
        'frontCompressionClick', 6, ...
        'frontReboundClick', 6, ...
        'rearCompressionClick', 6, ...
        'rearReboundClick', 6);

    options = merge_defaults(options, defaults);

    frontSpringTable = readtable(options.springTableFile, 'Sheet', options.frontSpringSheet);
    rearSpringTable = readtable(options.springTableFile, 'Sheet', options.rearSpringSheet);
    damperTable = readtable(options.damperTableFile, 'Sheet', options.damperSheet);

    frontSpringCurve = SetSpringCurve(frontSpringTable, car.frontMotionRatio);
    rearSpringCurve = SetSpringCurve(rearSpringTable, car.rearMotionRatio);

    frontDamperCurve = SetDamperClick( ...
        damperTable, ...
        car.frontMotionRatio, ...
        options.frontCompressionClick, ...
        options.frontReboundClick);

    rearDamperCurve = SetDamperClick( ...
        damperTable, ...
        car.rearMotionRatio, ...
        options.rearCompressionClick, ...
        options.rearReboundClick);

    curves = struct();
    curves.front.spring.x = frontSpringCurve(1, :);
    curves.front.spring.y = frontSpringCurve(2, :);
    curves.front.damper.x = frontDamperCurve(1, :);
    curves.front.damper.y = frontDamperCurve(2, :);

    curves.rear.spring.x = rearSpringCurve(1, :);
    curves.rear.spring.y = rearSpringCurve(2, :);
    curves.rear.damper.x = rearDamperCurve(1, :);
    curves.rear.damper.y = rearDamperCurve(2, :);

    curves.meta = options;
end

function merged = merge_defaults(options, defaults)
    merged = defaults;
    optionFields = fieldnames(options);
    for idx = 1:numel(optionFields)
        fieldName = optionFields{idx};
        merged.(fieldName) = options.(fieldName);
    end
end
