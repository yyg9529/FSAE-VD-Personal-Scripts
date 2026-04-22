function car = get_suspension_default_params()
%GET_SUSPENSION_DEFAULT_PARAMS Default phase-1 parameters for SuspensionModelBase.
%   Current assumptions are documented in SuspensionModelBase_README.md.

    car = struct();

    % Sprung body properties
    car.sprungMass = 300;              % kg
    car.pitchInertia = 44;             % kg*m^2
    car.rollInertia = 52;              % kg*m^2

    % Unsprung masses use front/rear axle symmetry in phase 1
    car.unsprungMassFront = 12;        % kg, per corner
    car.unsprungMassRear = 12;         % kg, per corner

    % Vehicle geometry
    car.wheelbase = 1.53;              % m
    car.trackFront = 1.20;             % m
    car.trackRear = 1.18;              % m

    % Current assumption:
    % legacy half-car used CGx = 0.45 as a wheelbase ratio. Phase 1 locks
    % CGx to "front axle to CG distance [m]" and converts from that ratio.
    car.CGx = 0.45 * car.wheelbase;    % m, front axle to CG
    car.CGh = 0.30;                    % m

    % Tire vertical properties
    car.frontTireStiffness = 114000;   % N/m
    car.frontTireDamping = 400;        % N*s/m
    car.rearTireStiffness = 114000;    % N/m
    car.rearTireDamping = 400;         % N*s/m

    % Equivalent wheel-side ARB rates
    car.frontARBStiffness = 25000;     % N/m
    car.rearARBStiffness = 18000;      % N/m

    % Motion ratio is defined as damper travel / wheel travel
    car.frontMotionRatio = 1.50;       % -
    car.rearMotionRatio = 1.50;        % -

    % Reserved fields for future yaw-model coupling
    car.aeroBalance = 0.50;            % front axle downforce fraction [-]
    car.frontRCHeight = 0.025;         % m
    car.rearRCHeight = 0.040;          % m
end
