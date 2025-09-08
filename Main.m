clc, clear, close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%                       AU MOG SIZING CODE MAIN                          %
%                                                                        %   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create the Optimizer Object

ga_settings.lower_bds = zeros([1 3]);                % Set the lower bounds for the optimizer
ga_settings.upper_bds = [100 100 100];               % Set the upper bounds for the  optimizer
ga_settings.pop_size = 15;                           % Set the population size for GA optimizer
ga_settings.generations = 5;                         % Define the number of generations for the GA optimizer
GA_optimizer = optimizer.GA_Optimizer(ga_settings);  % Create optimizer object       


% [best_score, best_params] = GA_optimizer.run(@MOG_Solver);
MOG_Solver([]);










%% Running the MOG Solver


function cost = MOG_Solver(vehicle_params)

    % Define Vehicle
    aircraft_W_S = 5.21;    % [kg/m]
    num_pucks = 3;
    banner_length = 10;
    aircraft = Vehicle("Aircraft", aircraft_W_S, num_pucks, banner_length, 0.7, 0.05);

    % Add wing
    wing_q4 = 0.5; % [m]
    AR_ = 4.4;
    taper_ = 1;
    sweep_ = 0;
    airfoil_ = 'NACA2412';
    aircraft.add_wing(wing_q4, 0, AR_, taper_, sweep_, airfoil_);

    % Add fuselage
    aircraft.add_fuselage();

    % Add HT
    HT_AR_ = 3.0;
    HT_airfoil_ = "NACA0012";
    aircraft.add_HT(HT_AR_, HT_airfoil_);

    % Add VT
    VT_AR_ = 1.0;
    taper_ = 0.6;
    VT_airfoil_ = "NACA0012";
    aircraft.add_VT(VT_AR_, taper_, 0, VT_airfoil_);

    aircraft.analyze_airframe()
    

    %%% Define Missions

    % % Mission 1  **EXAMPLE**
    % mission_1_laps = 3;
    % mission_1_steps = 50;
    % mission_1_cruise_alt = 200;
    % mission_1_throttle = 0.6;
    % mission_1_flag_dist = 500;
    % 
    % mission_1 = mission.Mission_Builder(mission_1_laps, aircraft, mission_1_steps);
    % 
    % % Lap 1
    % mission_1.add_takeoff();
    % mission_1.add_climb(mission_1_cruise_alt, mission_1_throttle, 200);
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(180, 'instantaneous');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(360, 'sustained');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(180, 'instantaneous');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % 
    % % Lap 2
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(180, 'instantaneous');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(360, 'sustained');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(180, 'instantaneous');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % 
    % % Lap 3
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(180, 'instantaneous');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(360, 'sustained');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % mission_1.add_turn(180, 'instantaneous');
    % mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % % somehow land...
    % 
    % [t_total, E_total] = mission_1.run();




    % Generate Cost
    score_GM_Norm = 10;
    score_GM_AU = 0;
    score_GM = score_GM_Norm/score_GM_AU;

    get_income = @(n_Pass, n_Cargo, n_Laps) ...
        (n_Pass*(6+(2*n_Laps))) + (n_Cargo * (10 + (8*n_Laps)));
    get_cost = @(n_Laps, n_Pass, n_Cargo, EF) ...
        n_Laps*(10+(n_Pass * 0.5) + (n_Cargo * 2)) * EF;
    income_Norm = get_income(60, 20, 10); % 60 ducks, 20 pucks, 10 laps
    cost_Norm = get_cost(10, 60, 20, 1);
    score_M2_Norm = income_Norm - cost_Norm;
    income_AU = get_income(n_Pass, n_Cargo, n_Laps);
    cost_AU = get_cost(n_Laps, n_Pass, n_Cargo, EF);
    score_M2_AU = income_AU - cost_AU;
    score_M2 = score_M2_AU/score_M2_Norm;

    score_M3_Norm = 35*8/0.9; % 35 ft banner, 0.9 RAC, 8 laps
    score_M3_AU = L_Banner*N_Laps_M3*RAC;
    score_M3 = score_M3_AU/score_M3_Norm;

    score = score_GM + score_M2 + score_M3;
    cost = 1/score; % Higher score generates lower cost.

end