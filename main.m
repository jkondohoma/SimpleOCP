
clc; 
clear;

global T;

% Constants
exhaust_velocity = 2500.0;  % Example exhaust velocity (m/s)
initial_altitude = 1000.0;  % Example initial altitude (m)
initial_velocity = 0.0;     % Example initial velocity (m/s)
initial_mass = 1000.0;      % Example initial mass (kg)
terminal_altitude = 2000.0; % Desired terminal altitude (m)
terminal_velocity = 0.0;    % Desired terminal velocity (m/s)
T = 10.0;                   % Final time (s)
num_points = 50;            % Number of time points

% Time points and step size
time_points = linspace(0, T, num_points);

% Initial guess for thrust sequence
initial_guess = ones(1, num_points) * 0.5;  % Initial thrust values

% Constraints
A = []; b = []; Aeq = []; beq = [];
lb = zeros(1, num_points);    % Lower bound for thrust
ub = ones(1, num_points);     % Upper bound for thrust

% Terminal conditions
nonlcon = @(thrust_sequence)terminal_constraints(thrust_sequence, ...
    initial_altitude, initial_velocity, initial_mass, terminal_altitude, terminal_velocity);

% Optimize using fmincon
options = optimoptions('fmincon', 'Display', 'iter', 'MaxIterations', 100);
[optimal_thrust_sequence, objective] = fmincon(@objective_function, initial_guess, ...
    A, b, Aeq, beq, lb, ub, nonlcon, options);

fprintf("Optimal Thrust Sequence:\n");
disp(optimal_thrust_sequence);
fprintf("Maximized Final Altitude: %.2f\n", -objective); % Negative since we maximize the negative of altitude


% Objective function to be minimized (maximize negative altitude)
function value = objective_function(thrust_sequence)
    [~, final_altitude] = simulate_rocket(thrust_sequence);
    value = -final_altitude;
end

% Simulate the rocket dynamics and return final altitude
function [final_altitude, final_velocity] = simulate_rocket(thrust_sequence)
    % Constants
    exhaust_velocity = 2500.0;  % Example exhaust velocity (m/s)
    initial_altitude = 1000.0;  % Example initial altitude (m)
    initial_velocity = 0.0;     % Example initial velocity (m/s)
    initial_mass = 1000.0;      % Example initial mass (kg)
    T = 10.0;                   % Final time (s)
    num_points = 50;            % Number of time points
    S = 10;

    % Time points and step size
    time_points = linspace(0, T, num_points);
    
    % Simulate the rocket dynamics
%     [~, states] = ode45(@(t, y)rocket_dynamics(t, y, thrust_sequence), ...
% time_points, [initial_altitude, initial_velocity, initial_mass]);
    [~, states]=ode45(@(t,y),rocket_dynamics(t,y,thrust_sequence),time_points,[initial_altitude,initial_velocity,initial_mass] );

    
    % Extract final altitude and velocity
    final_altitude = states(end, 1);
    final_velocity = states(end, 2);
end

% Rocket dynamics equations
function dydt = rocket_dynamics(t, y, thrust_sequence)
    altitude = y(1);
    velocity = y(2);
    mass = y(3);
    
    acceleration = thrust_sequence(round(t / (10 / num_points)) + 1) / mass - 9.8;
    
    altitude_dot = velocity;
    velocity_dot = acceleration;
    mass_dot = -thrust_sequence(round(t / 10 / num_points)) + 1) / exhaust_velocity;
    
    dydt = [altitude_dot; velocity_dot; mass_dot];
end



