close all;
clear;

%% Parameters
% Inertia parameters
params.mass = 1.0; % kg
params.inertia = [0.01;0.01;0.02]; % [Ixx;Iyy;Izz] kg*m^2
params.gravity = 9.81; % m/s^2

% Rotor Parameters                     
params.rotorSpinDirs = [1; -1; 1; -1]; % Assume alternating spin: CCW, CW, CCW, CW
params.rotorSpinMin = 0.0; %rad/s
params.rotorSpinMax = 2e3; % rad/s
params.rotorInertia = 5e-5; % kg*m^2
params.kYaw = 0.01; % Adjust for rotor drag-based yaw moment
params.kThrust = 3e-6; % N/(rad/s)^2
params.rotorPositions = [0.5, 0.5, 0; %(x,y,z) rotor 1 body frame (m)
                        -0.5, 0.5, 0; %(x,y,z) rotor 2 body frame (m)
                        -0.5,-0.5, 0; %(x,y,z) rotor 3 body frame (m)
                         0.5,-0.5, 0]; %(x,y,z) rotor 4 body frame (m)

% PID Gains
params.pidGains.posP = 3 * ones(3,1);
params.pidGains.posD = 2 * ones(3,1);
params.pidGains.angleP = 5 * ones(3,1);
params.pidGains.angleD = 2 * ones(3,1);


%% Sim
initialState = zeros(12, 1); % [x y z xDot yDot zDot roll pitch yaw p q r]
tSpan = [0,30]; % 10 seconds

[t, s] = ode45(@(t, s) quad_ode(t, s, params), tSpan, initialState);


%% Get outputs
x = s(:,1);
y = s(:,2);
z = s(:,3);
vx = s(:,4);
vy = s(:,5);
vz = s(:,6);
phi = s(:,7);
theta = s(:,8);
psi = s(:,9);
p = s(:,10);
q = s(:,11);
r = s(:,12);

%% Plots
ax = [];

figure;
ax(end+1) = subplot(311);
plot(t, x); % z position
xlabel('Time (s)');
ylabel('x (m)');
grid on

ax(end+1) = subplot(312);
plot(t, y); % z position
xlabel('Time (s)');
ylabel('y (m)');
grid on

ax(end+1) = subplot(313);
plot(t, z); % z position
xlabel('Time (s)');
ylabel('z (m)');
grid on

figure;
ax(end+1) = subplot(311);
plot(t, vx); % z position
xlabel('Time (s)');
ylabel('vx (m)');
grid on

ax(end+1) = subplot(312);
plot(t, vy); % z position
xlabel('Time (s)');
ylabel('vy (m)');
grid on

ax(end+1) = subplot(313);
plot(t, vz); % z position
xlabel('Time (s)');
ylabel('vz (m)');
grid on

figure;
ax(end+1) = subplot(311);
plot(t, phi/pi*180); % z position
xlabel('Time (s)');
ylabel('phi (deg)');
grid on

ax(end+1) = subplot(312);
plot(t, theta/pi*180); % z position
xlabel('Time (s)');
ylabel('theta (deg)');
grid on

ax(end+1) = subplot(313);
plot(t, psi/pi*180); % z position
xlabel('Time (s)');
ylabel('psi (deg)');
grid on

figure;
ax(end+1) = subplot(311);
plot(t, p/pi*180); % z position
xlabel('Time (s)');
ylabel('phi (deg)');
grid on

ax(end+1) = subplot(312);
plot(t, q/pi*180); % z position
xlabel('Time (s)');
ylabel('theta (deg)');
grid on

ax(end+1) = subplot(313);
plot(t, r/pi*180); % z position
xlabel('Time (s)');
ylabel('psi (deg)');
grid on


%% Helper functions
function sDot = quad_ode(t, s, params)
    % Unpack parameters
    mass = params.mass;
    inertia = params.inertia; % [Ixx, Iyy, Izz]
    rotorPositions = params.rotorPositions; % 4x3 matrix [x y z] for each rotor
    rotorInertia = params.rotorInertia; % Rotational inertia per rotor (kg*m^2)
     
    % State extraction
    position     = s(1:3);       % [x; y; z]
    velocity     = s(4:6);       % [xDot; yDot; zDot]
    eulerAngles  = s(7:9);    % [roll; pitch; yaw]
    angularRates = s(10:12); % [p; q; r]
    
    
    
    % Set controller mode
    cmd.controllerMode = 2;
    
    switch cmd.controllerMode
        case 1
            % Set cmd (traj psi controller commands)
            cmd.position = [1; 1; 10];    
            cmd.velocity = [0; 0; 0];   
            cmd.yaw = pi/2;                 
        case 2
            % Set cmd (traj psi controller commands)
            cmd.position = [1; 1; 10];   
            cmd.velocity = [0; 0; 0];    
            cmd.yawRate = 0.1;
            
        otherwise
            error('controllerMode not supported');
    end
        
        
        
        
    
    
    % Set yaw rate controller command
    cmd.yawRate = 0.1;
    
    % Get rotor thrusts
    switch cmd.controllerMode
        case 1
            rotorThrustsCmd = traj_psi_controller(t,s,params,cmd);
        case 2
            rotorThrustsCmd = traj_yaw_rate_controller(t,s,params,cmd);
        otherwise
            error('controllerMode not supported');
    end
    
    % Get rotor spin rates
    wabs = sqrt(rotorThrustsCmd/params.kThrust);
    wabs = clip(wabs,params.rotorSpinMin,params.rotorSpinMax);
    rotorSpins = params.rotorSpinDirs .* wabs;
    
    % Get real rotor thrust
    rotorThrusts = params.kThrust * rotorSpins.^2;

    % Rotation matrix (body to world frame)
    phi = eulerAngles(1);
    theta = eulerAngles(2);
    psi = eulerAngles(3);
    rotationBodyToWorld = euler_to_rotation(phi, theta, psi);
    
    % Gravity force in world frame
    gravityForceWorld = [0; 0; -mass * params.gravity];

    % Compute rotor forces and torques
    [rotorForces, rotorTorques] = compute_rotor_forces_and_torques(rotorPositions, rotorThrusts, params);

    % Compute gyroscopic torque
    gyroTorque = compute_gyro_torque(angularRates, rotorSpins, rotorInertia);

    % Total torque (aerodynamic + gyroscopic)
    totalTorque = rotorTorques + gyroTorque;

    % Translational dynamics
    accelerationWorld = (1 / mass) * (rotationBodyToWorld * rotorForces) + gravityForceWorld / mass;

    % Rotational dynamics (Euler's rigid body equation)
    Ixx = inertia(1);
    Iyy = inertia(2);
    Izz = inertia(3);
    p = angularRates(1);
    q = angularRates(2);
    r = angularRates(3);

    angularAcceleration = [
        (totalTorque(1) - (q * r * (Izz - Iyy))) / Ixx;
        (totalTorque(2) - (p * r * (Ixx - Izz))) / Iyy;
        (totalTorque(3) - (p * q * (Iyy - Ixx))) / Izz;
    ];

    % Kinematics - Euler angle rates from body angular rates
    eulerRate = angular_rates_to_euler_rates(eulerAngles, angularRates);

    % Assemble state derivative vector
    sDot = zeros(12, 1);
    sDot(1:3) = velocity;              % Position rates
    sDot(4:6) = accelerationWorld;     % Velocity rates
    sDot(7:9) = eulerRate;             % Euler angle rates
    sDot(10:12) = angularAcceleration; % Angular rates
end

function y = clip(x,xmin,xmax)
    y = x;
    for i = 1 : numel(x)
       if x(i) < xmin
           y(i) = xmin;
       elseif x(i) > xmax
           y(i) = xmax;
       end
    end
end

function [rotorForces, rotorTorques] = compute_rotor_forces_and_torques(rotorPositions, rotorThrusts, params)
    % Computes net rotor forces and torques on the body.
    % rotorPositions: 4x3 matrix, each row is [x y z] position in body frame
    % rotorThrusts: 4x1 vector, thrust at each rotor (N)

    rotorForces = zeros(3, 1);
    rotorTorques = zeros(3, 1);

    for i = 1:4
        rotorForce = [0; 0; rotorThrusts(i)]; % Thrust along body Z
        rotorForces = rotorForces + rotorForce;

        % Torque from rotor force: r x F
        rotorTorques = rotorTorques + cross(rotorPositions(i, :)', rotorForce);
        
        % Torque from yaw moment
        rotorTorques = rotorTorques + params.kYaw * params.rotorSpinDirs(i) * rotorForce;
    end
end

function tauGyro = compute_gyro_torque(angularRates, rotorSpins, rotorInertia)
    % Gyroscopic torque from all rotors
    tauGyro = zeros(3, 1);
    for i = 1:4
        omegaRotor = [0; 0; rotorSpins(i)]; % Rotor spin vector (along z-axis)
        tauGyro = tauGyro + rotorInertia * cross(angularRates, omegaRotor);
    end
end

function rotation = euler_to_rotation(phi, theta, psi)
    % Rotation matrix from body to world frame
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    rotation = Rz * Ry * Rx;
end

function eulerRate = angular_rates_to_euler_rates(eulerAngles, angularRates)
    % Convert body rates to euler angle rates
    phi = eulerAngles(1);
    theta = eulerAngles(2);

    T = [
        1 sin(phi) * tan(theta) cos(phi) * tan(theta);
        0 cos(phi) -sin(phi);
        0 sin(phi) / cos(theta) cos(phi) / cos(theta)
    ];

    eulerRate = T * angularRates;
end

function thrusts = traj_psi_controller(~,s,params,cmd)

    % Extract state
    position = s(1:3);
    velocity = s(4:6);
    eulerAngles = s(7:9);
    angularRates = s(10:12);

    roll = eulerAngles(1);
    pitch = eulerAngles(2);
    yaw = eulerAngles(3);

    % Position/velocity/yaw commands
    posCmd = cmd.position;
    velCmd = cmd.velocity;
    yawCmd = cmd.yaw;

    % Position and velocity errors
    posError = posCmd - position;
    velError = velCmd - velocity;

    % PID position control (world frame force)
    worldForceCmd = params.pidGains.posP .* posError + ...
                    params.pidGains.posD .* velError;

    % Gravity compensation
    worldForceCmd(3) = worldForceCmd(3) + params.mass * params.gravity;

    % Convert desired world force to body frame
    R = euler_to_rotation(roll, pitch, yaw);
    bodyForceCmd = R' * worldForceCmd;

    % Total desired thrust (vertical force)
    totalThrust = norm(bodyForceCmd);

    % Desired attitude from desired horizontal force components
% %     rollCmd = atan2(bodyForceCmd(2), bodyForceCmd(3));
% %     pitchCmd = atan2(-bodyForceCmd(1), bodyForceCmd(3));
% %     
    if totalThrust > 0
        rollCmd = asin( -bodyForceCmd(2) / totalThrust );
    else
        rollCmd = 0.0;
    end
    
    if totalThrust > 0
        pitchCmd = asin( bodyForceCmd(1) / totalThrust );
    else
        pitchCmd = 0.0;
    end

    % Attitude errors
    angleError = [rollCmd - roll; pitchCmd - pitch; yawCmd - yaw];

    % Desired angular rates (PD attitude control)
    angleRateCmd = params.pidGains.angleP .* angleError - params.pidGains.angleD .* angularRates;

    % Desired moments (torques)
    desiredMoments = params.inertia .* angleRateCmd;

    % Compute mixing matrix directly from rotor positions and spin directions
    numRotors = size(params.rotorPositions, 1);
    mixingMatrix = zeros(4, numRotors);

    for i = 1:numRotors
        x = params.rotorPositions(i, 1);
        y = params.rotorPositions(i, 2);

        % Total thrust contribution (each rotor contributes directly to thrust)
        mixingMatrix(1, i) = 1;

        % Roll torque contribution (force * lever arm in Y)
        mixingMatrix(2, i) = y;

        % Pitch torque contribution (force * lever arm in X)
        mixingMatrix(3, i) = -x;

        % Yaw torque contribution (depends on spin direction)
        mixingMatrix(4, i) = params.kYaw * params.rotorSpinDirs(i);
    end

    % Solve for rotor thrusts
    thrusts = mixingMatrix \ [totalThrust; desiredMoments];

    % Ensure non-negative thrusts (no reverse thrust)
    thrusts = max(thrusts, 0);
end

function thrusts = traj_yaw_rate_controller(~,s,params,cmd)

    % Extract state
    position = s(1:3);
    velocity = s(4:6);
    eulerAngles = s(7:9);
    angularRates = s(10:12);

    roll = eulerAngles(1);
    pitch = eulerAngles(2);
    yaw = eulerAngles(3);

    % Position/velocity/yaw commands
    posCmd = cmd.position;
    velCmd = cmd.velocity;

    % Position and velocity errors
    posError = posCmd - position;
    velError = velCmd - velocity;

    % PID position control (world frame force)
    worldForceCmd = params.pidGains.posP .* posError + ...
                    params.pidGains.posD .* velError;

    % Gravity compensation
    worldForceCmd(3) = worldForceCmd(3) + params.mass * params.gravity;

    % Convert desired world force to body frame
    R = euler_to_rotation(roll, pitch, yaw);
    bodyForceCmd = R' * worldForceCmd;

    % Total desired thrust (vertical force)
    totalThrust = norm(bodyForceCmd);

    % Desired attitude from desired horizontal force components
% %     rollCmd = atan2(bodyForceCmd(2), bodyForceCmd(3));
% %     pitchCmd = atan2(-bodyForceCmd(1), bodyForceCmd(3));
% %     
    if totalThrust > 0
        rollCmd = asin( -bodyForceCmd(2) / totalThrust );
    else
        rollCmd = 0.0;
    end
    
    if totalThrust > 0
        pitchCmd = asin( bodyForceCmd(1) / totalThrust );
    else
        pitchCmd = 0.0;
    end

    % Attitude errors
    angleError = [rollCmd - roll; pitchCmd - pitch; 0];

    % Desired angular rates (PD attitude control)
    angleRateCmd = params.pidGains.angleP .* angleError - params.pidGains.angleD .* angularRates;
    
    % Overwrite yaw rate with cmd
    angleRateCmd(3) = cmd.yawRate;
    
    % Desired moments (torques)
    desiredMoments = params.inertia .* angleRateCmd;

    % Compute mixing matrix directly from rotor positions and spin directions
    numRotors = size(params.rotorPositions, 1);
    mixingMatrix = zeros(4, numRotors);

    for i = 1:numRotors
        x = params.rotorPositions(i, 1);
        y = params.rotorPositions(i, 2);

        % Total thrust contribution (each rotor contributes directly to thrust)
        mixingMatrix(1, i) = 1;

        % Roll torque contribution (force * lever arm in Y)
        mixingMatrix(2, i) = y;

        % Pitch torque contribution (force * lever arm in X)
        mixingMatrix(3, i) = -x;

        % Yaw torque contribution (depends on spin direction)
        mixingMatrix(4, i) = params.kYaw * params.rotorSpinDirs(i);
    end

    % Solve for rotor thrusts
    thrusts = mixingMatrix \ [totalThrust; desiredMoments];

    % Ensure non-negative thrusts (no reverse thrust)
    thrusts = max(thrusts, 0);
end
