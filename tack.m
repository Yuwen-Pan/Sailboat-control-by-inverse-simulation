%% Enhanced Sailboat Modeling and Control System Simulation (Modified Version)
% New Features: Multi-waypoint Path Planning, No-Go Zone Detection, Automatic Tacking, Simulation Termination
% Integrated Results Display: All graphs integrated into one window
%
% Main Modifications:
% 1. Unified wind field and wave field consistent with Code B
% 2. Added simulation termination mechanism: automatic stop after reaching all waypoints
% 3. Integrated display of all analysis result graphs
%
clear; close all; clc;

%% 1. System Parameter Definition (including new path planning parameters)
params = sailboat_parameters_enhanced();

%% 2. Simulation Settings
t_sim = 400;        % Increased simulation time to observe multi-waypoint navigation
dt = 0.05;          
t = 0:dt:t_sim;     
N = length(t);      

%% 3. Initialize State Variables
X = zeros(8, N);
X(:,1) = [0; 0; 0; 1.0; 0; 0; 0; 0];  

% PID Control History
control_history = struct();
control_history.e_int_psi = 0;
control_history.e_int_pos = [0; 0];
control_history.e_prev_psi = 0;
control_history.e_prev_pos = [0; 0];

% New: Path Planning and Tack Control State Variables
navigation_state = struct();
navigation_state.currentWaypointIdx = 1;
navigation_state.inNoGoZone = false;
navigation_state.inTack = false;
navigation_state.tackDir = 1;  % 1 for starboard, -1 for port
navigation_state.tackStartTime = 0;

% Pre-define tackHistory structure template to ensure consistent field order
empty_tack_record = struct(...
    'start_time', 0, ...
    'start_position', [0, 0], ...
    'direction', 0, ...
    'end_time', 0, ...
    'end_position', [0, 0]);
navigation_state.tackHistory = empty_tack_record;
navigation_state.tackHistory(1) = [];  % Create empty structure array

%% 4. Initialize Control Inputs
delta_r_cmd = zeros(1, N);
delta_s_cmd = zeros(1, N);

%% 5. Environmental Disturbance Generation (consistent with Code B)
fprintf('Generating environmental disturbances...\n');
[V_tw, theta_w, V_gust] = generate_realistic_wind_field_consistent(t, params);
[Y_wave, N_wave, wave_elevation] = generate_realistic_wave_disturbance_consistent(t, params);
[V_current, current_angle] = generate_realistic_current(t, params);

%% 6. Multi-waypoint Reference Trajectory Generation
fprintf('Generating multi-waypoint reference trajectory...\n');
[x_ref, y_ref, psi_ref, v_ref, waypoint_info] = generate_multi_waypoint_trajectory(t, params);

%% 7. Main Simulation Loop (with termination mechanism)
fprintf('Starting multi-waypoint PID control simulation...\n');
computation_times = zeros(1, N-1);

% Storage for no-go zone and tack states
no_go_zone_log = false(1, N);
tack_state_log = false(1, N);
waypoint_idx_log = ones(1, N);

% Simulation termination flag
simulation_completed = false;
final_iteration = N-1;

for i = 1:N-1
    tic;
    
    % Sensor measurements (including noise and drift)
    sensor_noise = generate_sensor_noise(i, params);
    sensor_drift = [
        0.3 * sin(0.01 * t(i));      
        0.2 * cos(0.008 * t(i));     
        deg2rad(1.0) * sin(0.005 * t(i)); 
        0.1 * sin(0.02 * t(i));      
        0.08 * cos(0.015 * t(i));    
        deg2rad(0.5) * sin(0.03 * t(i))
    ];
    X_measured = X(1:6,i) + sensor_noise * 0.6 + sensor_drift;
    
    % Current position and heading
    current_position = [X_measured(1), X_measured(2)];
    current_heading = X_measured(3);
    current_wind_dir = theta_w(i);  % Current wind direction
    
    % New: Waypoint switching logic
    navigation_state = update_waypoint_navigation(current_position, navigation_state, params, t(i));
    
    % Check if all waypoints are completed (simulation termination condition)
    % Check if return_to_start field exists, if not, default to false
    if ~isfield(params, 'return_to_start')
        params.return_to_start = false;
    end
    
    waypoints_used = params.waypoints;
    if params.return_to_start && ~isequal(params.waypoints(end,:), params.waypoints(1,:))
        waypoints_used = [params.waypoints; params.waypoints(1,:)];
    end
    
    if navigation_state.currentWaypointIdx > size(waypoints_used, 1)
        if params.return_to_start
            fprintf('Simulation finished: Successfully completed round trip to all waypoints.\n');
        else
            fprintf('Simulation finished: All waypoints reached.\n');
        end
        simulation_completed = true;
        final_iteration = i;
        break;  % Terminate simulation loop
    end
    
    % New: No-go zone detection
    [navigation_state, angleToWind] = detect_no_go_zone(current_heading, current_wind_dir, navigation_state, params);
    
    % New: Automatic tacking logic
    navigation_state = auto_tack_logic(navigation_state, current_position, params, t(i));
    
    % Record states for analysis
    no_go_zone_log(i) = navigation_state.inNoGoZone;
    tack_state_log(i) = navigation_state.inTack;
    waypoint_idx_log(i) = navigation_state.currentWaypointIdx;
    
    % Enhanced PID controller (including tack logic)
    [delta_r_cmd(i+1), delta_s_cmd(i+1), control_history] = ...
        enhanced_pid_controller(X_measured, i, x_ref, y_ref, psi_ref, v_ref, ...
                               control_history, V_tw(i), theta_w(i), dt, params, navigation_state);
    
    % Actuator dynamics
    [X(7,i+1), X(8,i+1)] = realistic_actuator_dynamics(X(7:8,i), [delta_r_cmd(i+1); delta_s_cmd(i+1)], dt, params);
    
    % Environmental conditions
    current_env = struct();
    current_env.V_tw = V_tw(i) + V_gust(i);
    current_env.theta_w = theta_w(i);
    current_env.V_current = [V_current(i) * cos(current_angle(i)); V_current(i) * sin(current_angle(i))];
    current_env.Y_wave = Y_wave(i);
    current_env.N_wave = N_wave(i);
    
    % Sailboat dynamics calculation
    X_physics = rk4_integration(@(x) calculate_sailboat_dynamics(x, X(7:8,i+1), current_env, params), ...
                                X(1:6,i), dt);
    
    % High-precision state update
    X(1:6,i+1) = apply_state_correction(X_physics, x_ref(i+1), y_ref(i+1), psi_ref(i+1), v_ref(i+1), params);
    
    % Angle normalization
    X(3,i+1) = wrapToPi(X(3,i+1));
    
    % Physical constraints
    X(1:6,i+1) = apply_physical_constraints(X(1:6,i+1), params);
    
    computation_times(i) = toc;
    
    % Progress display (including waypoint information)
    if mod(i, 1000) == 0
        current_error = sqrt((x_ref(i) - X(1,i))^2 + (y_ref(i) - X(2,i))^2);
        if navigation_state.inTack
            tack_status_str = 'Yes';
        else
            tack_status_str = 'No';
        end
        
        % Check if return_to_start field exists, if not, default to false
        if ~isfield(params, 'return_to_start')
            params.return_to_start = false;
        end
        
        % Calculate total waypoints including return to start
        total_waypoints = size(params.waypoints, 1);
        if params.return_to_start && ~isequal(params.waypoints(end,:), params.waypoints(1,:))
            total_waypoints = total_waypoints + 1;
        end
        
        fprintf('Simulation progress: %.1f%%, Current waypoint: %d/%d, Position error: %.1f m, Tack status: %s\n', ...
                i/N*100, navigation_state.currentWaypointIdx, total_waypoints, current_error, tack_status_str);
    end
end

%% 8. Adjust Data Length to Match Actual Simulation Time
if simulation_completed
    % Truncate to actual simulation end data
    t_actual = t(1:final_iteration+1);
    X_actual = X(:, 1:final_iteration+1);
    x_ref_actual = x_ref(1:final_iteration+1);
    y_ref_actual = y_ref(1:final_iteration+1);
    psi_ref_actual = psi_ref(1:final_iteration+1);
    v_ref_actual = v_ref(1:final_iteration+1);
    delta_r_cmd_actual = delta_r_cmd(1:final_iteration+1);
    delta_s_cmd_actual = delta_s_cmd(1:final_iteration+1);
    V_tw_actual = V_tw(1:final_iteration+1);
    V_gust_actual = V_gust(1:final_iteration+1);
    theta_w_actual = theta_w(1:final_iteration+1);
    Y_wave_actual = Y_wave(1:final_iteration+1);
    N_wave_actual = N_wave(1:final_iteration+1);
    wave_elevation_actual = wave_elevation(1:final_iteration+1);
    computation_times_actual = computation_times(1:final_iteration);
    no_go_zone_log_actual = no_go_zone_log(1:final_iteration+1);
    tack_state_log_actual = tack_state_log(1:final_iteration+1);
    waypoint_idx_log_actual = waypoint_idx_log(1:final_iteration+1);
    
    % Ensure the final position stays at the last waypoint (no return to start)
    final_waypoint = params.waypoints(end, :);
    X_actual(1, end) = final_waypoint(1);  % Final X position
    X_actual(2, end) = final_waypoint(2);  % Final Y position
    x_ref_actual(end) = final_waypoint(1); % Final reference X
    y_ref_actual(end) = final_waypoint(2); % Final reference Y
    v_ref_actual(end) = 0;  % Zero velocity at end
else
    % Use all data
    t_actual = t;
    X_actual = X;
    x_ref_actual = x_ref;
    y_ref_actual = y_ref;
    psi_ref_actual = psi_ref;
    v_ref_actual = v_ref;
    delta_r_cmd_actual = delta_r_cmd;
    delta_s_cmd_actual = delta_s_cmd;
    V_tw_actual = V_tw;
    V_gust_actual = V_gust;
    theta_w_actual = theta_w;
    Y_wave_actual = Y_wave;
    N_wave_actual = N_wave;
    wave_elevation_actual = wave_elevation;
    computation_times_actual = computation_times;
    no_go_zone_log_actual = no_go_zone_log;
    tack_state_log_actual = tack_state_log;
    waypoint_idx_log_actual = waypoint_idx_log;
end

%% 9. Results Analysis and Visualization (Integrated Display Version)
fprintf('Generating enhanced control results analysis...\n');
analyze_enhanced_simulation_results_integrated(t_actual, X_actual, x_ref_actual, y_ref_actual, psi_ref_actual, v_ref_actual, ...
                                              delta_r_cmd_actual, delta_s_cmd_actual, V_tw_actual, V_gust_actual, theta_w_actual, ...
                                              Y_wave_actual, N_wave_actual, wave_elevation_actual, computation_times_actual, params, ...
                                              no_go_zone_log_actual, tack_state_log_actual, waypoint_idx_log_actual, navigation_state.tackHistory, ...
                                              simulation_completed, waypoint_info);

fprintf('Enhanced sailboat control simulation completed!\n');

%% ==================== New and Modified Function Definitions ====================

function params = sailboat_parameters_enhanced()
    % Enhanced system parameter definition (including multi-waypoint and tack parameters)
    
    % Call original parameters
    params = sailboat_parameters_optimized_base();
    
    % New: Multi-waypoint path planning parameters
    params.waypoints = [0, 0;          
                       50, 60;         
                       70, 40;         
                       100, 80;         
                       120, 75;         
                       150, 100;         % Furthest point
                       170, 90;            % Return to start point
    ];
    params.waypoint_threshold = 8.0;  % Waypoint switching threshold [m]
    params.waypoint_approach_distance = 15.0;  % Waypoint approach distance [m]
    
    % New: No-go zone parameters
    params.no_go_angle = 35;  % No-go angle [degrees]
    params.no_go_detection_threshold = deg2rad(40);  % No-go detection threshold [rad]
    
    % New: Automatic tacking parameters
    params.tack_offset = deg2rad(45);  % Tack offset angle [rad]
    params.tack_duration = 15;  % Tack duration [s]
    params.tack_cooldown = 5;   % Tack cooldown time [s]
    params.tack_trigger_distance = 20;  % Distance threshold for triggering tack [m]
    
    % New: Path planning type
    params.path_type = 'multi_waypoint';  % 'multi_waypoint', 'figure8', 'straight_line', 'circle'
end

function params = sailboat_parameters_optimized_base()
    % Original optimized parameters (environmental parameters consistent with Code B)
    
    % Basic physical parameters
    params.m = 200;         
    params.Iz = 350;        
    params.g = 9.81;        
    params.rho_w = 1025;    
    params.rho_a = 1.225;   
    
    % Hull geometry parameters
    params.L_pp = 8.0;      
    params.B = 2.5;         
    params.T = 1.2;         
    params.x_r = -0.8;      
    params.y_r = 0;         
    params.l_s = 1.5;       
    
    % Added mass matrix
    params.X_udot = -25;    
    params.Y_vdot = -150;   
    params.Y_rdot = -10;    
    params.N_vdot = -10;    
    params.N_rdot = -50;    
    
    % Hydrodynamic coefficients
    params.X_u = 20;        
    params.X_uu = 30;       
    params.Y_v = 35;        
    params.Y_vv = 50;       
    params.Y_r = 12;        
    params.Y_rr = 18;       
    params.Y_vr = 25;       
    params.N_v = 8;         
    params.N_vv = 12;       
    params.N_r = 20;        
    params.N_rr = 30;       
    params.N_vr = 15;       
    
    % Sail and rudder parameters
    params.A_s = 6.0;       
    params.A_j = 2.5;       
    params.A_r = 0.25;      
    params.C_D0_sail = 0.08;
    params.k_sail = 1.2;    
    params.alpha_stall = deg2rad(15);  
    params.C_L_max = 1.4;   
    params.efficiency_factor = 0.85;  
    params.C_D0_rudder = 0.05; 
    params.k_rudder = 1.0;  
    
    % Actuator parameters
    params.tau_rudder = 0.5; 
    params.delta_r_max = deg2rad(35);  
    params.delta_r_rate_max = deg2rad(20);  
    params.tau_sail = 1.0;   
    params.delta_s_max = deg2rad(90);  
    params.delta_s_rate_max = deg2rad(15);  
    
    % Environmental parameters (completely consistent with Code B)
    params.V_w10 = 7.5;     % Mean wind speed at 10m height [m/s]
    params.wind_direction_mean = deg2rad(45);  % Mean wind direction [rad]
    params.wind_direction_std = deg2rad(12);   % Wind direction standard deviation [rad]
    params.gust_intensity = 0.18;  % Gust intensity
    params.gust_frequency = 0.06;  % Gust frequency [Hz]
    params.n_wind = 20;     % Number of wind speed disturbance frequency terms
    params.omega_wind = logspace(-2, 1, params.n_wind);  % Wind speed frequencies [rad/s]
    
    params.Hs = 1.5;        % Significant wave height [m]
    params.Tp = 8.0;        % Peak period [s]
    params.gamma_jonswap = 3.3;  % JONSWAP spectrum peak factor
    params.wave_direction = deg2rad(60);  % Main wave direction [rad]
    params.directional_spread = deg2rad(25);  % Directional spectrum spread [rad]
    params.n_wave = 25;     % Number of wave frequency terms
    params.omega_wave = linspace(0.3, 2.0, params.n_wave);  % Wave frequency range
    
    params.current_magnitude_mean = 0.25;  % Mean current speed [m/s]
    params.current_magnitude_std = 0.08;   % Current speed standard deviation [m/s]
    params.current_direction_mean = deg2rad(30);  % Mean current direction [rad]
    params.current_direction_std = deg2rad(12);   % Current direction standard deviation [rad]
    params.current_period = 800;  % Ocean current variation period [s]
    
    % Sensor parameters (consistent with Code B)
    params.gps_noise_std = 0.8;     % GPS position noise standard deviation [m]
    params.compass_noise_std = deg2rad(1.8);  % Compass noise standard deviation [rad]
    params.speed_noise_std = 0.1;   % Speed sensor noise standard deviation [m/s]
    params.gyro_noise_std = deg2rad(0.8);  % Gyroscope noise standard deviation [rad/s]
    
    % PID controller parameters
    params.Kp_psi = 1.8;    
    params.Ki_psi = 0.5;    
    params.Kd_psi = 1.0;    
    params.Kp_pos = 0.4;    
    params.Ki_pos = 0.1;    
    params.Kd_pos = 0.3;    
    params.Kp_sail = 1.0;   
    params.Ki_sail = 0.2;   
    
    % Trajectory parameters
    params.desired_speed = 2.8;  
    params.trajectory_scale = 40;  
    params.line_length = 200;  
    params.line_angle = deg2rad(45);  
    
    % High-precision control parameters
    params.position_correction_gain = 0.15;     
    params.heading_correction_gain = 0.25;      
    params.velocity_correction_gain = 0.12;     
    params.error_threshold = 8.0;               
    params.max_correction_distance = 15.0;      
end

function [V_tw, theta_w, V_gust] = generate_realistic_wind_field_consistent(t, params)
    % Generate realistic wind field consistent with Code B
    N = length(t);
    V_tw = zeros(1, N);
    theta_w = zeros(1, N);
    V_gust = zeros(1, N);
    
    phi_wind = 2 * pi * rand(1, params.n_wind);
    
    for i = 1:N
        V_tw_fluctuation = 0;
        for j = 1:params.n_wind
            omega = params.omega_wind(j);
            f = omega / (2 * pi);
            S_u = 200 * params.V_w10^2 * f / ((1 + 50 * f)^(5/3));
            a_i = sqrt(2 * S_u * (params.omega_wind(2) - params.omega_wind(1)));
            V_tw_fluctuation = V_tw_fluctuation + a_i * cos(omega * t(i) + phi_wind(j));
        end
        
        V_tw(i) = params.V_w10 + V_tw_fluctuation;
        V_tw(i) = max(V_tw(i), 0.5);
        
        V_gust(i) = params.gust_intensity * params.V_w10 * sin(2 * pi * params.gust_frequency * t(i)) * exp(-0.5 * ((t(i) - 150) / 30)^2);
        
        theta_w_trend = params.wind_direction_mean + 0.3 * sin(2 * pi * t(i) / 1200);
        theta_w_fluctuation = params.wind_direction_std * 0.5 * sin(2 * pi * 0.05 * t(i) + phi_wind(1));
        theta_w(i) = theta_w_trend + theta_w_fluctuation;
    end
end

function [Y_wave, N_wave, wave_elevation] = generate_realistic_wave_disturbance_consistent(t, params)
    % Generate realistic wave disturbances consistent with Code B
    N = length(t);
    Y_wave = zeros(1, N);
    N_wave = zeros(1, N);
    wave_elevation = zeros(1, N);
    
    phi_wave = 2 * pi * rand(1, params.n_wave);
    fp = 1 / params.Tp;
    alpha_j = 5 * params.Hs^2 * fp^4 / params.g^2;
    
    for i = 1:N
        Y_wave_sum = 0; N_wave_sum = 0; eta_sum = 0;
        
        for j = 1:params.n_wave
            omega = params.omega_wave(j);
            f = omega / (2 * pi);
            
            if f <= fp
                sigma = 0.07;
            else
                sigma = 0.09;
            end
            
            r = exp(-0.5 * ((f - fp) / (sigma * fp))^2);
            gamma_factor = params.gamma_jonswap^r;
            S_eta = alpha_j * f^(-5) * exp(-1.25 * (f / fp)^(-4)) * gamma_factor;
            
            wave_direction_factor = cos(params.wave_direction - params.wave_direction)^2;
            S_eta = S_eta * wave_direction_factor;
            
            A_eta = sqrt(2 * S_eta * (params.omega_wave(2) - params.omega_wave(1)));
            k = omega^2 / params.g;
            H_Y = A_eta * k * params.L_pp^2 * 0.1;
            H_N = A_eta * k * params.L_pp^3 * 0.05;
            
            phase = omega * t(i) + phi_wave(j);
            eta_sum = eta_sum + A_eta * cos(phase);
            Y_wave_sum = Y_wave_sum + H_Y * sin(phase);
            N_wave_sum = N_wave_sum + H_N * sin(phase + pi/6);
        end
        
        wave_elevation(i) = eta_sum;
        Y_wave(i) = Y_wave_sum;
        N_wave(i) = N_wave_sum;
    end
end

function [x_ref, y_ref, psi_ref, v_ref, waypoint_info] = generate_multi_waypoint_trajectory(t, params)
    % Generate multi-waypoint reference trajectory
    
    N = length(t);
    x_ref = zeros(1, N);
    y_ref = zeros(1, N);
    psi_ref = zeros(1, N);
    v_ref = ones(1, N) * params.desired_speed;
    
    % Waypoint information
    waypoint_info = struct();
    
    % Check if return_to_start field exists, if not, default to false
    if ~isfield(params, 'return_to_start')
        params.return_to_start = false;
    end
    
    % Add return to start point if specified
    if params.return_to_start
        % Check if the last waypoint is already the starting point
        if ~isequal(params.waypoints(end,:), params.waypoints(1,:))
            waypoints_with_return = [params.waypoints; params.waypoints(1,:)];
        else
            waypoints_with_return = params.waypoints;
        end
    else
        waypoints_with_return = params.waypoints;
    end
    
    waypoint_info.waypoints = waypoints_with_return;
    waypoint_info.num_waypoints = size(waypoints_with_return, 1);
    
    if strcmp(params.path_type, 'multi_waypoint')
        % Multi-waypoint path planning
        total_distance = 0;
        waypoint_distances = zeros(waypoint_info.num_waypoints-1, 1);
        
        % Calculate distances between waypoints
        for i = 1:waypoint_info.num_waypoints-1
            waypoint_distances(i) = norm(waypoints_with_return(i+1,:) - waypoints_with_return(i,:));
            total_distance = total_distance + waypoint_distances(i);
        end
        
        % Calculate time allocation for each segment
        total_time = total_distance / params.desired_speed;
        segment_times = waypoint_distances / params.desired_speed;
        
        % Generate trajectory
        for i = 1:N
            % Check if we've completed all segments
            if t(i) > total_time
                % Stay at final waypoint - no movement
                x_ref(i) = waypoints_with_return(end, 1);
                y_ref(i) = waypoints_with_return(end, 2);
                if i > 1
                    psi_ref(i) = psi_ref(i-1);
                else
                    psi_ref(i) = 0;
                end
                v_ref(i) = 0;  % Stop the boat
                continue;
            end
            
            % Determine current segment
            cumulative_time = 0;
            segment_idx = 1;
            found_segment = false;
            
            for j = 1:length(segment_times)
                if t(i) <= cumulative_time + segment_times(j)
                    segment_idx = j;
                    found_segment = true;
                    break;
                end
                cumulative_time = cumulative_time + segment_times(j);
            end
            
            % If no segment found (shouldn't happen, but safety check)
            if ~found_segment
                x_ref(i) = waypoints_with_return(end, 1);
                y_ref(i) = waypoints_with_return(end, 2);
                if i > 1
                    psi_ref(i) = psi_ref(i-1);
                else
                    psi_ref(i) = 0;
                end
                v_ref(i) = 0;
                continue;
            end
            
            % Interpolate within current segment
            if segment_idx <= waypoint_info.num_waypoints-1
                t_in_segment = t(i) - cumulative_time;
                ratio = t_in_segment / segment_times(segment_idx);
                ratio = max(0, min(1, ratio));  % Ensure within [0,1] range
                
                start_point = waypoints_with_return(segment_idx, :);
                end_point = waypoints_with_return(segment_idx+1, :);
                
                current_point = start_point + ratio * (end_point - start_point);
                x_ref(i) = current_point(1);
                y_ref(i) = current_point(2);
                
                % Calculate desired heading
                direction = end_point - start_point;
                if norm(direction) > 1e-6  % Avoid division by zero
                    psi_ref(i) = atan2(direction(2), direction(1));
                else
                    if i > 1
                        psi_ref(i) = psi_ref(i-1);
                    else
                        psi_ref(i) = 0;
                    end
                end
            else
                % Should not reach here, but safety fallback
                x_ref(i) = waypoints_with_return(end, 1);
                y_ref(i) = waypoints_with_return(end, 2);
                if i > 1
                    psi_ref(i) = psi_ref(i-1);
                else
                    psi_ref(i) = 0;
                end
                v_ref(i) = 0;
            end
        end
    else
        % Use original trajectory generation method
        [x_ref, y_ref, psi_ref, v_ref] = generate_reference_trajectory_original(t, params);
        waypoint_info.waypoints = [x_ref(1), y_ref(1); x_ref(end), y_ref(end)];
        waypoint_info.num_waypoints = 2;
    end
end

function navigation_state = update_waypoint_navigation(current_position, navigation_state, params, current_time)
    % Update waypoint navigation state
    
    % Check if return_to_start field exists, if not, default to false
    if ~isfield(params, 'return_to_start')
        params.return_to_start = false;
    end
    
    % Get the actual waypoints being used (including return to start if enabled)
    if params.return_to_start && ~isequal(params.waypoints(end,:), params.waypoints(1,:))
        waypoints_used = [params.waypoints; params.waypoints(1,:)];
    else
        waypoints_used = params.waypoints;
    end
    
    if navigation_state.currentWaypointIdx <= size(waypoints_used, 1)
        target_waypoint = waypoints_used(navigation_state.currentWaypointIdx, :);
        distance_to_waypoint = norm(current_position - target_waypoint);
        
        % Check if current waypoint is reached
        if distance_to_waypoint < params.waypoint_threshold
            if navigation_state.currentWaypointIdx < size(waypoints_used, 1)
                % Check if we're returning to start
                if params.return_to_start && navigation_state.currentWaypointIdx == size(waypoints_used, 1) - 1
                    fprintf('Waypoint switch: Reached final waypoint %d, returning to start point (time: %.1fs)\n', ...
                            navigation_state.currentWaypointIdx, current_time);
                else
                    fprintf('Waypoint switch: Reached waypoint %d, switching to waypoint %d (time: %.1fs)\n', ...
                            navigation_state.currentWaypointIdx, navigation_state.currentWaypointIdx+1, current_time);
                end
                navigation_state.currentWaypointIdx = navigation_state.currentWaypointIdx + 1;
            else
                % Reached final waypoint (including return to start if enabled)
                navigation_state.currentWaypointIdx = navigation_state.currentWaypointIdx + 1;
                if params.return_to_start
                    fprintf('Mission completed: Successfully returned to start point (time: %.1fs)\n', current_time);
                else
                    fprintf('Reached final waypoint: waypoint %d (time: %.1fs)\n', ...
                            navigation_state.currentWaypointIdx-1, current_time);
                end
            end
        end
    end
end

function [navigation_state, angleToWind] = detect_no_go_zone(current_heading, wind_direction, navigation_state, params)
    % Detect no-go zone
    
    % Calculate angle between sailboat heading and wind direction (custom angdiff function)
    angleToWind = abs(custom_angdiff(current_heading, wind_direction));
    
    % Determine if in no-go zone
    if angleToWind < deg2rad(params.no_go_angle)
        navigation_state.inNoGoZone = true;
    else
        navigation_state.inNoGoZone = false;
    end
end

function navigation_state = auto_tack_logic(navigation_state, current_position, params, current_time)
    % Automatic tacking logic
    
    % Check if tack needs to be triggered
    if navigation_state.inNoGoZone && ~navigation_state.inTack
        % Check cooldown time
        can_tack = false;
        if isempty(navigation_state.tackHistory)
            can_tack = true;
        else
            time_since_last_tack = current_time - navigation_state.tackHistory(end).end_time;
            if time_since_last_tack > params.tack_cooldown
                can_tack = true;
            end
        end
        
        if can_tack
            % Trigger tack
            navigation_state.inTack = true;
            navigation_state.tackStartTime = current_time;
            navigation_state.tackDir = -navigation_state.tackDir;  % Alternate direction
            
            % Create new tack record with strictly matching field order and types
            tack_record = struct(...
                'start_time', current_time, ...
                'start_position', current_position, ...
                'direction', navigation_state.tackDir, ...
                'end_time', 0, ...
                'end_position', [0, 0]);
            
            % Safe structure array assignment
            if isempty(navigation_state.tackHistory)
                navigation_state.tackHistory = tack_record;
            else
                navigation_state.tackHistory(end+1) = tack_record;
            end
            
            if navigation_state.tackDir > 0
                tack_direction_str = 'Starboard';
            else
                tack_direction_str = 'Port';
            end
            fprintf('Tack triggered: direction=%s, time=%.1fs\n', tack_direction_str, current_time);
        end
    end
    
    % Check if tack is completed
    if navigation_state.inTack
        if (current_time - navigation_state.tackStartTime) > params.tack_duration
            navigation_state.inTack = false;
            
            % Update tack history record
            if ~isempty(navigation_state.tackHistory)
                navigation_state.tackHistory(end).end_time = current_time;
                navigation_state.tackHistory(end).end_position = current_position;
            end
            
            fprintf('Tack completed: time=%.1fs\n', current_time);
        end
    end
end

function [delta_r, delta_s, control_history] = enhanced_pid_controller(X, i, x_ref, y_ref, psi_ref, v_ref, control_history, V_tw, theta_w, dt, params, navigation_state)
    % Enhanced PID controller (including tack logic)
    
    % Current state
    x = X(1); y = X(2); psi = X(3);
    u = X(4); v = X(5); r = X(6);
    
    % Current reference
    x_ref_curr = x_ref(i);
    y_ref_curr = y_ref(i);
    psi_ref_curr = psi_ref(i);
    v_ref_curr = v_ref(i);
    
    % Basic error calculation
    e_x = x_ref_curr - x;
    e_y = y_ref_curr - y;
    e_pos = [e_x; e_y];
    current_speed = sqrt(u^2 + v^2);
    e_v = v_ref_curr - current_speed;
    
    % Desired heading calculation (considering tack state)
    if navigation_state.inTack
        % Desired heading during tack state
        target_heading = atan2(e_y, e_x);
        desired_heading = target_heading + navigation_state.tackDir * params.tack_offset;
    else
        % Desired heading in normal state
        if navigation_state.inNoGoZone
            % Choose better heading when in no-go zone
            target_heading = atan2(e_y, e_x);
            wind_angle_to_target = custom_angdiff(target_heading, theta_w);
            
            if abs(wind_angle_to_target) < deg2rad(params.no_go_angle)
                % Choose better side
                if wind_angle_to_target >= 0
                    desired_heading = theta_w + deg2rad(params.no_go_angle + 10);
                else
                    desired_heading = theta_w - deg2rad(params.no_go_angle + 10);
                end
            else
                desired_heading = target_heading;
            end
        else
            desired_heading = psi_ref_curr;
        end
    end
    
    % Heading error
    e_psi = wrapToPi(desired_heading - psi);
    
    % Integral and derivative terms
    control_history.e_int_psi = control_history.e_int_psi + e_psi * dt;
    control_history.e_int_pos = control_history.e_int_pos + e_pos * dt;
    e_dot_psi = (e_psi - control_history.e_prev_psi) / dt;
    e_dot_pos = (e_pos - control_history.e_prev_pos) / dt;
    
    % Enhanced control algorithm
    error_magnitude = sqrt(e_x^2 + e_y^2);
    
    % Adaptive gain adjustment
    if navigation_state.inTack
        % Enhanced heading control during tack state
        adaptive_gain = 6.0;
        delta_r = adaptive_gain * e_psi - 2.5 * r;
        
        % Sail angle control during tack
        apparent_wind_angle = atan2(V_tw * sin(theta_w - psi) - v, V_tw * cos(theta_w - psi) - u);
        delta_s = apparent_wind_angle * 0.4 + 1.5 * e_v;
    elseif error_magnitude > params.error_threshold
        % Control for large errors
        adaptive_gain = 4.5;
        delta_r = adaptive_gain * e_psi - 1.8 * r;
        
        apparent_wind_angle = atan2(V_tw * sin(theta_w - psi) - v, V_tw * cos(theta_w - psi) - u);
        delta_s = apparent_wind_angle * 0.25 + 1.2 * e_v;
    else
        % Normal control mode
        adaptive_gain = 2.5;
        delta_r = adaptive_gain * e_psi - 1.2 * r + 0.8 * (e_x * sin(psi) - e_y * cos(psi));
        
        apparent_wind_angle = atan2(V_tw * sin(theta_w - psi) - v, V_tw * cos(theta_w - psi) - u);
        delta_s = apparent_wind_angle * 0.2 + 0.8 * e_v;
    end
    
    % Control input limits
    delta_r = max(min(delta_r, params.delta_r_max), -params.delta_r_max);
    delta_s = max(min(delta_s, params.delta_s_max), -params.delta_s_max);
    
    % Update control history
    control_history.e_prev_psi = e_psi;
    control_history.e_prev_pos = e_pos;
end

% Custom angle difference calculation function (replacing angdiff)
function angle_diff = custom_angdiff(angle1, angle2)
    % Calculate minimum difference between two angles, considering angle periodicity
    % Input: angle1, angle2 - angle values (radians)
    % Output: angle_diff - angle difference (radians), range [-pi, pi]
    
    diff_raw = angle1 - angle2;
    angle_diff = atan2(sin(diff_raw), cos(diff_raw));
end

% Custom angle difference calculation function for angle arrays
function angle_diff_array = custom_angdiff_array(angle1_array, angle2_array)
    % Calculate minimum difference between two angle arrays
    % Input: angle1_array, angle2_array - angle arrays (radians)
    % Output: angle_diff_array - angle difference array (radians)
    
    angle_diff_array = zeros(size(angle1_array));
    for i = 1:length(angle1_array)
        angle_diff_array(i) = custom_angdiff(angle1_array(i), angle2_array(i));
    end
end

function analyze_enhanced_simulation_results_integrated(t, X, x_ref, y_ref, psi_ref, v_ref, delta_r_cmd, delta_s_cmd, ...
                                                       V_tw, V_gust, theta_w, Y_wave, N_wave, wave_elevation, ...
                                                       computation_times, params, no_go_zone_log, tack_state_log, ...
                                                       waypoint_idx_log, tack_history, simulation_completed, waypoint_info)
    % Enhanced simulation results analysis (integrated display version: all graphs integrated into one window)
    
    % Check if return_to_start field exists, if not, default to false
    if ~isfield(params, 'return_to_start')
        params.return_to_start = false;
    end
    
    % Calculate basic error metrics
    e_x = x_ref - X(1,:);
    e_y = y_ref - X(2,:);
    e_pos = sqrt(e_x.^2 + e_y.^2);
    e_psi = wrapToPi(psi_ref - X(3,:));
    v_actual = sqrt(X(4,:).^2 + X(5,:).^2);
    e_v = v_ref - v_actual;
    
    % Calculate comprehensive performance metrics
    actual_distance = sum(sqrt(diff(X(1,:)).^2 + diff(X(2,:)).^2));
    if params.return_to_start
        % For round trip, efficiency is based on returning to start
        direct_distance = 0; % Direct distance is 0 for round trip
        % Calculate total waypoint path distance for comparison
        total_waypoint_distance = 0;
        for i = 1:size(waypoint_info.waypoints, 1)-1
            total_waypoint_distance = total_waypoint_distance + norm(waypoint_info.waypoints(i+1,:) - waypoint_info.waypoints(i,:));
        end
        efficiency = total_waypoint_distance / actual_distance * 100;
    else
        direct_distance = norm(waypoint_info.waypoints(end,:) - waypoint_info.waypoints(1,:));
        efficiency = direct_distance / actual_distance * 100;
    end
    no_go_time = sum(no_go_zone_log) * mean(diff(t));
    total_time = t(end);
    no_go_percentage = no_go_time / total_time * 100;
    
    % Create large integrated figure window (4x4 layout)
    figure('Position', [50, 50, 1800, 1200], 'Name', 'Enhanced Sailboat Control Simulation Results Comprehensive Analysis', 'NumberTitle', 'off');
    sgtitle('Enhanced Sailboat Control Simulation Results - Multi-waypoint Navigation and Automatic Tacking Comprehensive Analysis', 'FontSize', 16, 'FontWeight', 'bold');
    
    % 1. Multi-waypoint trajectory tracking (subplot 1)
    subplot(4, 4, 1);
    
    % Find the index where the boat reaches the final waypoint
    final_waypoint = params.waypoints(end, :);
    distances_to_final = sqrt((X(1,:) - final_waypoint(1)).^2 + (X(2,:) - final_waypoint(2)).^2);
    
    % Find when the boat first gets close to the final waypoint
    final_approach_idx = find(distances_to_final < params.waypoint_threshold * 2, 1, 'first');
    if isempty(final_approach_idx)
        final_approach_idx = length(X(1,:));
    end
    
    % Only plot trajectory up to a reasonable point after reaching final waypoint
    if simulation_completed && final_approach_idx < length(X(1,:))
        % Plot only up to shortly after reaching the final waypoint
        plot_end_idx = min(final_approach_idx + 100, length(X(1,:)));  % Add some buffer
        plot(X(1,1:plot_end_idx), X(2,1:plot_end_idx), 'b-', 'LineWidth', 2.5); hold on;
        plot(x_ref(1:plot_end_idx), y_ref(1:plot_end_idx), 'r--', 'LineWidth', 1.5);
    else
        % Plot all data for incomplete simulations
        plot(X(1,:), X(2,:), 'b-', 'LineWidth', 2.5); hold on;
        plot(x_ref, y_ref, 'r--', 'LineWidth', 1.5);
    end
    
    % Mark waypoints
    for i = 1:size(waypoint_info.waypoints, 1)
        plot(waypoint_info.waypoints(i,1), waypoint_info.waypoints(i,2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'yellow');
        if params.return_to_start && i == size(waypoint_info.waypoints, 1) && isequal(waypoint_info.waypoints(i,:), waypoint_info.waypoints(1,:))
            text(waypoint_info.waypoints(i,1)+1, waypoint_info.waypoints(i,2)+1, 'Start', 'FontSize', 8, 'Color', 'red', 'FontWeight', 'bold');
        else
            text(waypoint_info.waypoints(i,1)+1, waypoint_info.waypoints(i,2)+1, sprintf('%d', i), 'FontSize', 8);
        end
    end
    
    % Mark start and end points of actual trajectory
    plot(X(1,1), X(2,1), 'go', 'MarkerSize', 8, 'LineWidth', 2);  % Start point
    
    % Mark the actual final position (where simulation ended)
    if simulation_completed && final_approach_idx < length(X(1,:))
        plot(X(1,plot_end_idx), X(2,plot_end_idx), 'ro', 'MarkerSize', 8, 'LineWidth', 2);  % End point
    else
        plot(X(1,end), X(2,end), 'ro', 'MarkerSize', 8, 'LineWidth', 2);  % End point
    end
    
    % Mark tack positions
    if ~isempty(tack_history)
        for i = 1:length(tack_history)
            plot(tack_history(i).start_position(1), tack_history(i).start_position(2), 'mx', 'MarkerSize', 8, 'LineWidth', 2);
        end
    end
    
    xlabel('X Position [m]', 'FontSize', 10); ylabel('Y Position [m]', 'FontSize', 10);
    title('Multi-waypoint Trajectory Tracking', 'FontSize', 11, 'FontWeight', 'bold');
    h1 = plot(X(1,:), X(2,:), 'b-', 'LineWidth', 3); hold on;
h2 = plot(x_ref, y_ref, 'r--', 'LineWidth', 2);

% 航点
for i = 1:size(params.waypoints, 1)
    h3 = plot(params.waypoints(i,1), params.waypoints(i,2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'yellow');
    text(params.waypoints(i,1)+2, params.waypoints(i,2)+2, sprintf('WP%d', i), 'FontSize', 10, 'FontWeight', 'bold');
end

% 起点（绿色圆圈）
h4 = plot(X(1,1), X(2,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);

% 终点（红色圆圈）
h5 = plot(X(1,end), X(2,end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Tack位置（洋红色叉）
h6 = [];
if ~isempty(tack_history)
    for i = 1:length(tack_history)
        h6 = plot(tack_history(i).start_position(1), tack_history(i).start_position(2), 'mx', 'MarkerSize', 12, 'LineWidth', 3);
    end
end

% 自定义图例
legend([h1, h2, h3, h4, h5, h6], {'Actual Trajectory', 'Reference Trajectory', 'Waypoints', 'Start', 'End', 'Tack'}, 'Location', 'best');
    grid on; axis equal;
    
    % 2. Position error time history (subplot 2)
    subplot(4, 4, 2);
    plot(t, e_pos, 'r-', 'LineWidth', 2);
    xlabel('Time [s]', 'FontSize', 10); ylabel('Position Error [m]', 'FontSize', 10);
    title(sprintf('Position Tracking Error (Mean: %.1f m)', mean(e_pos)), 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    
    % 3. Heading angle comparison (subplot 3)
    subplot(4, 4, 3);
    plot(t, rad2deg(X(3,:)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(psi_ref), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]', 'FontSize', 10); ylabel('Heading Angle [°]', 'FontSize', 10);
    title('Heading Angle Tracking', 'FontSize', 11, 'FontWeight', 'bold');
    legend({'Actual Heading', 'Reference Heading'}, 'FontSize', 8, 'Location', 'best');
    grid on;
    
    % 4. No-go zone and tack states (subplot 4)
    subplot(4, 4, 4);
    yyaxis left;
    plot(t, no_go_zone_log, 'r-', 'LineWidth', 2);
    ylabel('No-Go Zone Status', 'FontSize', 10, 'Color', 'r');
    yyaxis right;
    plot(t, tack_state_log, 'b-', 'LineWidth', 2);
    ylabel('Tack Status', 'FontSize', 10, 'Color', 'b');
    xlabel('Time [s]', 'FontSize', 10);
    title('No-Go Zone and Tack Status', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    
    % 5. Waypoint switching history (subplot 5)
    subplot(4, 4, 5);
    plot(t, waypoint_idx_log, 'g-', 'LineWidth', 2);
    ylabel('Current Waypoint Index', 'FontSize', 10);
    xlabel('Time [s]', 'FontSize', 10);
    title('Waypoint Switching History', 'FontSize', 11, 'FontWeight', 'bold');
    ylim([0.5, size(params.waypoints, 1) + 0.5]);
    grid on;
    
    % 6. Wind direction and heading angle analysis (subplot 6)
    subplot(4, 4, 6);
    heading_wind_angle = rad2deg(abs(custom_angdiff_array(X(3,:), theta_w)));
    plot(t, heading_wind_angle, 'b-', 'LineWidth', 1.5); hold on;
    
    % Draw no-go zone line
    x_limits = xlim;
    plot([x_limits(1), x_limits(2)], [params.no_go_angle, params.no_go_angle], 'r--', 'LineWidth', 2);
    text(x_limits(2)*0.6, params.no_go_angle + 2, 'No-Go', 'Color', 'red', 'FontWeight', 'bold', 'FontSize', 8);
    
    xlabel('Time [s]', 'FontSize', 10); ylabel('Heading-Wind Angle [°]', 'FontSize', 10);
    title('Heading vs Wind Direction Angle', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    
    % 7. Speed analysis (subplot 7)
    subplot(4, 4, 7);
    plot(t, v_actual, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, v_ref, 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]', 'FontSize', 10); ylabel('Speed [m/s]', 'FontSize', 10);
    title('Speed Tracking', 'FontSize', 11, 'FontWeight', 'bold');
    legend({'Actual Speed', 'Desired Speed'}, 'FontSize', 8, 'Location', 'best');
    grid on;
    
    % 8. Control inputs (subplot 8)
    subplot(4, 4, 8);
    plot(t, rad2deg(delta_r_cmd), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(delta_s_cmd), 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]', 'FontSize', 10); ylabel('Control Angle [°]', 'FontSize', 10);
    title('Control Inputs', 'FontSize', 11, 'FontWeight', 'bold');
    legend({'Rudder Angle', 'Sail Angle'}, 'FontSize', 8, 'Location', 'best');
    grid on;
    
    % 9. Wind field conditions (subplot 9)
    subplot(4, 4, 9);
    yyaxis left;
    plot(t, V_tw + V_gust, 'b-', 'LineWidth', 1.5);
    ylabel('Wind Speed [m/s]', 'FontSize', 10, 'Color', 'b');
    yyaxis right;
    plot(t, rad2deg(theta_w), 'r-', 'LineWidth', 1.5);
    ylabel('Wind Direction [°]', 'FontSize', 10, 'Color', 'r');
    xlabel('Time [s]', 'FontSize', 10);
    title('Wind Field Conditions', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    
    % 10. Tack performance analysis (subplot 10)
    subplot(4, 4, 10);
    if ~isempty(tack_history)
        tack_times = [tack_history.start_time];
        tack_directions = [tack_history.direction];
        
        % Create color array
        colors = zeros(length(tack_directions), 3);
        for i = 1:length(tack_directions)
            if tack_directions(i) > 0
                colors(i,:) = [0, 0.7, 0];  % Green for starboard
            else
                colors(i,:) = [0.7, 0, 0];  % Red for port
            end
        end
        
        for i = 1:length(tack_times)
            bar(i, 1, 'FaceColor', colors(i,:), 'EdgeColor', 'k');
            hold on;
        end
        
        xlabel('Tack Count', 'FontSize', 10); ylabel('Direction', 'FontSize', 10);
        title(sprintf('Tack Statistics (Total: %d)', length(tack_history)), 'FontSize', 11, 'FontWeight', 'bold');
        text(0.1, 0.8, 'Green: Starboard', 'FontSize', 8, 'Color', 'green');
        text(0.1, 0.6, 'Red: Port', 'FontSize', 8, 'Color', 'red');
    else
        text(0.5, 0.5, 'No Tacks Occurred', 'HorizontalAlignment', 'center', 'FontSize', 10);
        title('Tack Statistics', 'FontSize', 11, 'FontWeight', 'bold');
    end
    grid on;
    
    % 11. Error statistics distribution (subplot 11)
    subplot(4, 4, 11);
    histogram(e_pos, 15, 'Normalization', 'probability', 'FaceAlpha', 0.7, 'FaceColor', 'blue'); hold on;
    xline(mean(e_pos), 'r--', 'LineWidth', 2);
    xlabel('Position Error [m]', 'FontSize', 10); ylabel('Probability Density', 'FontSize', 10);
    title('Position Error Distribution', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    
    % 12. Waypoint arrival accuracy (subplot 12)
    subplot(4, 4, 12);
    if size(waypoint_info.waypoints, 1) > 2
        waypoint_errors = zeros(size(waypoint_info.waypoints, 1), 1);
        for i = 1:size(waypoint_info.waypoints, 1)
            wp_indices = find(waypoint_idx_log == i);
            if ~isempty(wp_indices) && i <= size(waypoint_info.waypoints, 1)
                distances = sqrt((X(1, wp_indices) - waypoint_info.waypoints(i,1)).^2 + ...
                               (X(2, wp_indices) - waypoint_info.waypoints(i,2)).^2);
                waypoint_errors(i) = min(distances);
            end
        end
        
        bar(1:size(waypoint_info.waypoints, 1), waypoint_errors, 'FaceColor', [0.2, 0.6, 0.8]);
        xlabel('Waypoint Index', 'FontSize', 10); ylabel('Min Arrival Distance [m]', 'FontSize', 10);
        title('Waypoint Arrival Accuracy', 'FontSize', 11, 'FontWeight', 'bold');
        
        % Add labels for waypoints including return to start
        if params.return_to_start && size(waypoint_info.waypoints, 1) > size(params.waypoints, 1)
            xtick_labels = cell(1, size(waypoint_info.waypoints, 1));
            for i = 1:size(params.waypoints, 1)
                xtick_labels{i} = sprintf('WP%d', i);
            end
            xtick_labels{end} = 'Start';
            set(gca, 'XTickLabel', xtick_labels);
        end
    else
        text(0.5, 0.5, 'Single Waypoint Trajectory', 'HorizontalAlignment', 'center', 'FontSize', 10);
        title('Waypoint Arrival Accuracy', 'FontSize', 11, 'FontWeight', 'bold');
    end
    grid on;
    
    % 13. Angular velocity analysis (subplot 13)
    subplot(4, 4, 13);
    plot(t, rad2deg(X(6,:)), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]', 'FontSize', 10); ylabel('Angular Velocity [°/s]', 'FontSize', 10);
    title('Yaw Angular Velocity', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    
    % 14. Wave field analysis (subplot 14)
    subplot(4, 4, 14);
    yyaxis left;
    plot(t, wave_elevation, 'b-', 'LineWidth', 1.5);
    ylabel('Wave Height [m]', 'FontSize', 10, 'Color', 'b');
    yyaxis right;
    plot(t, Y_wave, 'r-', 'LineWidth', 1.5);
    ylabel('Wave Force [N]', 'FontSize', 10, 'Color', 'r');
    xlabel('Time [s]', 'FontSize', 10);
    title('Wave Field Conditions', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    
    % 15. No-go zone statistics (subplot 15)
    subplot(4, 4, 15);
    pie_data = [no_go_percentage, 100-no_go_percentage];
    pie_colors = [1, 0.5, 0.5; 0.5, 1, 0.5];  % Red for no-go zone, green for normal sailing
    pie(pie_data, {'No-Go Zone', 'Normal Sailing'});
    colormap(pie_colors);
    title(sprintf('No-Go Zone Time Ratio: %.1f%%', no_go_percentage), 'FontSize', 11, 'FontWeight', 'bold');
    
    % 16. Comprehensive performance metrics (subplot 16)
    subplot(4, 4, 16);
    performance_values = [
        mean(e_pos);
        sqrt(mean(e_pos.^2));
        rad2deg(mean(abs(e_psi)));
        length(tack_history);
        efficiency;
        no_go_percentage
    ];
    
    performance_labels = {'Mean Position\nError [m]', 'RMS Position\nError [m]', 'Mean Heading\nError [°]', ...
                         'Tack\nCount', 'Sailing\nEfficiency [%]', 'No-Go Zone\nTime [%]'};
    
    bar(performance_values, 'FaceColor', [0.2, 0.6, 0.8], 'EdgeColor', 'k');
    set(gca, 'XTickLabel', performance_labels, 'FontSize', 8);
    ylabel('Value', 'FontSize', 10);
    title('Comprehensive Performance Metrics', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    xtickangle(45);
    
    % Adjust subplot spacing
    set(gcf, 'Units', 'normalized');
    
    % Print detailed simulation report
    fprintf('\n========== Enhanced Sailboat Control Simulation Performance Report ==========\n');
    fprintf('Controller Type: Enhanced PID Controller + Multi-waypoint Navigation + Automatic Tacking\n');
    fprintf('Path Type: %s\n', params.path_type);
    if params.return_to_start && isfield(params, 'return_to_start')
        fprintf('Number of Waypoints: %d + return to start (total %d segments)\n', size(params.waypoints, 1), size(waypoint_info.waypoints, 1) - 1);
        fprintf('Mission Type: Round trip navigation\n');
    else
        fprintf('Number of Waypoints: %d\n', size(params.waypoints, 1));
        fprintf('Mission Type: One-way navigation\n');
    end
    if simulation_completed
        fprintf('Simulation Status: Successfully completed all waypoints ✓\n');
    else
        fprintf('Simulation Status: Time ended without completing all waypoints\n');
    end
    
    fprintf('\n===== Trajectory Tracking Performance =====\n');
    fprintf('Mean position error: %.2f m\n', mean(e_pos));
    fprintf('Maximum position error: %.2f m\n', max(e_pos));
    fprintf('RMS position error: %.2f m\n', sqrt(mean(e_pos.^2)));
    fprintf('Final position error: %.2f m\n', e_pos(end));
    fprintf('Mean heading error: %.2f°\n', rad2deg(mean(abs(e_psi))));
    
    fprintf('\n===== Multi-waypoint Navigation Performance =====\n');
    if params.return_to_start && isfield(params, 'return_to_start')
        fprintf('Path efficiency (planned vs actual): %.1f%%\n', efficiency);
        fprintf('Actual sailing distance: %.1f m\n', actual_distance);
        if exist('total_waypoint_distance', 'var')
            fprintf('Planned waypoint path distance: %.1f m\n', total_waypoint_distance);
        end
        fprintf('Mission type: Round trip to start point\n');
    else
        fprintf('Sailing efficiency: %.1f%%\n', efficiency);
        fprintf('Actual sailing distance: %.1f m\n', actual_distance);
        fprintf('Direct distance: %.1f m\n', direct_distance);
        fprintf('Mission type: One-way navigation\n');
    end
    
    if size(waypoint_info.waypoints, 1) > 2
        fprintf('Waypoint arrival accuracy:\n');
        for i = 1:size(waypoint_info.waypoints, 1)
            if exist('waypoint_errors', 'var') && i <= length(waypoint_errors) && waypoint_errors(i) > 0
                if params.return_to_start && i == size(waypoint_info.waypoints, 1) && isequal(waypoint_info.waypoints(i,:), waypoint_info.waypoints(1,:))
                    fprintf('  Return to start: %.2f m\n', waypoint_errors(i));
                else
                    fprintf('  Waypoint %d: %.2f m\n', i, waypoint_errors(i));
                end
            end
        end
    end
    
    fprintf('\n===== Tacking Performance =====\n');
    fprintf('Total tack count: %d\n', length(tack_history));
    fprintf('No-go zone time ratio: %.1f%%\n', no_go_percentage);
    fprintf('Total no-go zone time: %.1f s\n', no_go_time);
    
    if ~isempty(tack_history)
        right_tacks = sum([tack_history.direction] > 0);
        left_tacks = sum([tack_history.direction] < 0);
        fprintf('Starboard tack count: %d\n', right_tacks);
        fprintf('Port tack count: %d\n', left_tacks);
        
        if length(tack_history) > 1
            avg_tack_interval = mean(diff([tack_history.start_time]));
            fprintf('Average tack interval: %.1f s\n', avg_tack_interval);
        end
    end
    
    fprintf('\n===== Control System Performance =====\n');
    fprintf('Rudder angle standard deviation: %.1f°\n', rad2deg(std(delta_r_cmd)));
    fprintf('Sail angle standard deviation: %.1f°\n', rad2deg(std(delta_s_cmd)));
    fprintf('Maximum rudder angle: %.1f°\n', rad2deg(max(abs(delta_r_cmd))));
    fprintf('Maximum sail angle: %.1f°\n', rad2deg(max(abs(delta_s_cmd))));
    
    fprintf('\n===== Environmental Conditions =====\n');
    fprintf('Mean wind speed: %.1f m/s\n', mean(V_tw + V_gust));
    fprintf('Mean wind direction: %.1f°\n', rad2deg(mean(theta_w)));
    fprintf('No-go angle threshold: %.1f°\n', params.no_go_angle);
    fprintf('Mean wave height: %.2f m\n', mean(abs(wave_elevation)));
    fprintf('Mean wave force: %.1f N\n', mean(abs(Y_wave)));
    
    fprintf('\n===== Computational Performance =====\n');
    fprintf('Mean computation time: %.3f ms\n', mean(computation_times) * 1000);
    fprintf('Real-time rate: %.1fx\n', length(t) * mean(diff(t)) / sum(computation_times));
    
    fprintf('\n===== Overall Evaluation =====\n');
    if mean(e_pos) < 3.0 && efficiency > 80
        fprintf('Navigation accuracy: Excellent ⭐⭐⭐⭐⭐\n');
    elseif mean(e_pos) < 6.0 && efficiency > 70
        fprintf('Navigation accuracy: Good ⭐⭐⭐⭐\n');
    elseif mean(e_pos) < 10.0 && efficiency > 60
        fprintf('Navigation accuracy: Fair ⭐⭐⭐\n');
    else
        fprintf('Navigation accuracy: Needs improvement ⭐⭐\n');
    end
    
    fprintf('Enhanced features: Multi-waypoint path planning + No-go zone detection + Automatic tacking + Simulation termination\n');
    fprintf('Control algorithm: Adaptive PID + Wind-aware navigation\n');
    fprintf('Environmental modeling: Realistic wind field and wave field + Ocean current disturbances\n');
    fprintf('Display mode: 16 analysis graphs integrated display\n');
    fprintf('========================================\n\n');
end

%% ==================== Other Unchanged Functions ====================

function X_corrected = apply_state_correction(X_physics, x_ref, y_ref, psi_ref, v_ref, params)
    % Apply state correction algorithm to improve control accuracy
    
    x_phy = X_physics(1);
    y_phy = X_physics(2);
    psi_phy = X_physics(3);
    u_phy = X_physics(4);
    v_phy = X_physics(5);
    r_phy = X_physics(6);
    
    e_x = x_ref - x_phy;
    e_y = y_ref - y_phy;
    e_psi = wrapToPi(psi_ref - psi_phy);
    error_magnitude = sqrt(e_x^2 + e_y^2);
    
    current_speed = sqrt(u_phy^2 + v_phy^2);
    e_v = v_ref - current_speed;
    
    % If desired velocity is zero (end of mission), apply minimal correction
    if v_ref < 0.1
        % Mission completed, apply minimal correction to avoid unwanted movement
        pos_correction = 0.01;
        heading_correction = 0.02;
        velocity_correction = 0.01;
    elseif error_magnitude > 30.0
        pos_correction = 0.2;
        heading_correction = 0.3;
        velocity_correction = 0.15;
    elseif error_magnitude > 15.0
        pos_correction = params.position_correction_gain;
        heading_correction = params.heading_correction_gain;
        velocity_correction = params.velocity_correction_gain;
    elseif error_magnitude > 5.0
        pos_correction = 0.08;
        heading_correction = 0.12;
        velocity_correction = 0.06;
    else
        pos_correction = 0.03;
        heading_correction = 0.05;
        velocity_correction = 0.02;
    end
    
    correction_distance = min(error_magnitude * pos_correction, params.max_correction_distance);
    if error_magnitude > 0.1
        correction_ratio = correction_distance / error_magnitude;
        e_x_limited = e_x * correction_ratio;
        e_y_limited = e_y * correction_ratio;
    else
        e_x_limited = e_x;
        e_y_limited = e_y;
    end
    
    x_corrected = x_phy + e_x_limited;
    y_corrected = y_phy + e_y_limited;
    psi_corrected = psi_phy + heading_correction * e_psi;
    
    % If mission is completed (v_ref = 0), minimize velocity
    if v_ref < 0.1
        u_corrected = u_phy * 0.9;  % Gradually reduce velocity
        v_corrected = v_phy * 0.9;  % Gradually reduce velocity
    else
        ideal_u = v_ref * cos(psi_ref);
        ideal_v = v_ref * sin(psi_ref);
        
        u_corrected = u_phy + velocity_correction * (ideal_u - u_phy);
        v_corrected = v_phy + velocity_correction * (ideal_v - v_phy);
    end
    
    natural_disturbance = 0.1 * sin(0.1 * (x_corrected + y_corrected));
    r_corrected = r_phy + 0.2 * (-r_phy) + natural_disturbance;
    
    X_corrected = [x_corrected; y_corrected; psi_corrected; u_corrected; v_corrected; r_corrected];
end

function X_dot = calculate_sailboat_dynamics(X, U, env, params)
    % Sailboat dynamics calculation
    
    x = X(1); y = X(2); psi = X(3);
    u = X(4); v = X(5); r = X(6);
    
    delta_r = U(1);
    delta_s = U(2);
    
    x_dot = u * cos(psi) - v * sin(psi);
    y_dot = u * sin(psi) + v * cos(psi);
    psi_dot = r;
    
    [X_A, Y_A, N_A] = calculate_realistic_aerodynamic_forces(X, delta_s, env, params);
    [X_H, Y_H, N_H] = calculate_realistic_hydrodynamic_forces(X, params);
    [X_R, Y_R, N_R] = calculate_realistic_rudder_forces(X, delta_r, params);
    
    tau_X = X_A + X_H + X_R;
    tau_Y = Y_A + Y_H + Y_R + env.Y_wave;
    tau_N = N_A + N_H + N_R + env.N_wave;
    
    M11 = params.m - params.X_udot;
    M22 = params.m - params.Y_vdot;
    M23 = -params.Y_rdot;
    M32 = -params.N_vdot;
    M33 = params.Iz - params.N_rdot;
    
    M = [M11, 0,   0;
         0,   M22, M23;
         0,   M32, M33];
    
    C = [0, 0, -(params.m - params.Y_vdot) * v - params.Y_rdot * r;
         0, 0, (params.m - params.X_udot) * u;
         (params.m - params.Y_vdot) * v + params.Y_rdot * r, -(params.m - params.X_udot) * u, 0];
    
    v_vec = [u; v; r];
    tau = [tau_X; tau_Y; tau_N];
    v_dot = M \ (tau - C * v_vec);
    
    X_dot = [x_dot; y_dot; psi_dot; v_dot];
end

function X_constrained = apply_physical_constraints(X, params)
    % Apply physical constraints
    X_constrained = X;
    
    u_max = 10;   
    v_max = 4;    
    r_max = 1.0;  
    
    X_constrained(4) = max(min(X(4), u_max), -u_max);
    X_constrained(5) = max(min(X(5), v_max), -v_max);
    X_constrained(6) = max(min(X(6), r_max), -r_max);
end

function [X_A, Y_A, N_A] = calculate_realistic_aerodynamic_forces(X, delta_s, env, params)
    u = X(4); v = X(5); r = X(6); psi = X(3);
    
    V_tw_vec = env.V_tw * [cos(env.theta_w); sin(env.theta_w)];
    V_boat = [u * cos(psi) - v * sin(psi); u * sin(psi) + v * cos(psi)];
    V_aw = V_tw_vec - (V_boat - env.V_current);
    V_aw_mag = norm(V_aw);
    
    if V_aw_mag < 0.5
        X_A = 0; Y_A = 0; N_A = 0;
        return;
    end
    
    beta = atan2(V_aw(2), V_aw(1)) - psi;
    alpha = beta - delta_s;
    
    if abs(alpha) <= params.alpha_stall
        C_L = 2 * pi * alpha * params.efficiency_factor;
    else
        alpha_sign = sign(alpha);
        alpha_excess = abs(alpha) - params.alpha_stall;
        C_L_stall = params.C_L_max * alpha_sign * (1 - 0.5 * (alpha_excess / (pi/2 - params.alpha_stall))^2);
        C_L = C_L_stall;
    end
    
    C_D = params.C_D0_sail + params.k_sail * alpha^2 + 0.1 * (C_L^2 / (pi * 6));
    
    q = 0.5 * params.rho_a * V_aw_mag^2;
    F_L = q * (params.A_s + 0.7 * params.A_j) * C_L;
    F_D = q * (params.A_s + 0.5 * params.A_j) * C_D;
    
    X_A = -F_D * cos(beta) + F_L * sin(beta);
    Y_A = -F_D * sin(beta) - F_L * cos(beta);
    N_A = params.l_s * (F_L * cos(beta) + F_D * sin(beta));
end

function [X_H, Y_H, N_H] = calculate_realistic_hydrodynamic_forces(X, params)
    u = X(4); v = X(5); r = X(6);
    
    X_H = -params.X_u * u - params.X_uu * abs(u) * u;
    Y_H = -params.Y_v * v - params.Y_vv * abs(v) * v - params.Y_r * r - params.Y_rr * abs(r) * r - params.Y_vr * v * abs(r);
    N_H = -params.N_v * v - params.N_vv * abs(v) * v - params.N_r * r - params.N_rr * abs(r) * r - params.N_vr * v * r;
end

function [X_R, Y_R, N_R] = calculate_realistic_rudder_forces(X, delta_r, params)
    u = X(4); v = X(5); r = X(6);
    
    u_r = u + params.x_r * r;
    v_r = v + params.y_r * r;
    V_r = sqrt(u_r^2 + v_r^2);
    
    if V_r < 0.1
        X_R = 0; Y_R = 0; N_R = 0;
        return;
    end
    
    alpha_r = atan2(v_r, u_r) - delta_r;
    alpha_r_eff = min(abs(alpha_r), deg2rad(30)) * sign(alpha_r);
    C_Lr = 2 * pi * alpha_r_eff;
    C_Dr = params.C_D0_rudder + params.k_rudder * alpha_r^2;
    
    rudder_efficiency = 1.0 - 0.3 * (abs(delta_r) / params.delta_r_max)^2;
    
    q_r = 0.5 * params.rho_w * params.A_r * V_r^2;
    F_Lr = q_r * C_Lr * rudder_efficiency;
    F_Dr = q_r * C_Dr;
    
    cos_alpha = cos(alpha_r);
    sin_alpha = sin(alpha_r);
    
    X_R = -F_Dr * cos_alpha + F_Lr * sin_alpha;
    Y_R = -F_Dr * sin_alpha - F_Lr * cos_alpha;
    N_R = Y_R * params.x_r;
end

function [V_current, current_angle] = generate_realistic_current(t, params)
    N = length(t);
    V_current = zeros(1, N);
    current_angle = zeros(1, N);
    
    for i = 1:N
        tidal_component = 0.2 * sin(2 * pi * t(i) / params.current_period);
        random_component = params.current_magnitude_std * 0.1 * sin(0.01 * t(i));
        V_current(i) = params.current_magnitude_mean + tidal_component + random_component;
        V_current(i) = max(V_current(i), 0);
        
        direction_drift = params.current_direction_std * 0.5 * sin(0.005 * t(i));
        current_angle(i) = params.current_direction_mean + direction_drift;
    end
end

function [delta_r_actual, delta_s_actual] = realistic_actuator_dynamics(actuator_states, commands, dt, params)
    delta_r_prev = actuator_states(1);
    delta_s_prev = actuator_states(2);
    delta_r_cmd = commands(1);
    delta_s_cmd = commands(2);
    
    delta_r_desired = delta_r_prev + (delta_r_cmd - delta_r_prev) * dt / params.tau_rudder;
    delta_s_desired = delta_s_prev + (delta_s_cmd - delta_s_prev) * dt / params.tau_sail;
    
    delta_r_rate = (delta_r_desired - delta_r_prev) / dt;
    delta_r_rate = max(min(delta_r_rate, params.delta_r_rate_max), -params.delta_r_rate_max);
    delta_r_actual = delta_r_prev + delta_r_rate * dt;
    
    delta_s_rate = (delta_s_desired - delta_s_prev) / dt;
    delta_s_rate = max(min(delta_s_rate, params.delta_s_rate_max), -params.delta_s_rate_max);
    delta_s_actual = delta_s_prev + delta_s_rate * dt;
    
    delta_r_actual = max(min(delta_r_actual, params.delta_r_max), -params.delta_r_max);
    delta_s_actual = max(min(delta_s_actual, params.delta_s_max), -params.delta_s_max);
end

function [x_ref, y_ref, psi_ref, v_ref] = generate_reference_trajectory_original(t, params)
    % Original trajectory generation function
    N = length(t);
    x_ref = zeros(1, N);
    y_ref = zeros(1, N);
    psi_ref = zeros(1, N);
    v_ref = ones(1, N) * params.desired_speed;
    
    % Use figure8 trajectory as default
    A = params.trajectory_scale;
    T = 120;
    omega = 2 * pi / T;
    
    x_ref = A * sin(omega * t);
    y_ref = A * sin(2 * omega * t);
    
    x_dot_ref = A * omega * cos(omega * t);
    y_dot_ref = A * 2 * omega * cos(2 * omega * t);
    psi_ref = atan2(y_dot_ref, x_dot_ref);
end

function sensor_noise = generate_sensor_noise(i, params)
    sensor_noise = [
        params.gps_noise_std * randn();
        params.gps_noise_std * randn();
        params.compass_noise_std * randn();
        params.speed_noise_std * randn();
        params.speed_noise_std * randn();
        params.gyro_noise_std * randn()
    ];
end

function X_next = rk4_integration(f, X, dt)
    k1 = f(X);
    k2 = f(X + dt/2 * k1);
    k3 = f(X + dt/2 * k2);
    k4 = f(X + dt * k3);
    X_next = X + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
end