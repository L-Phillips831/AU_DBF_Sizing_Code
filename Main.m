clc, clear, close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%                       AU MOG SIZING CODE MAIN                          %
%                                                                        %   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Create the Optimizer Object
                        % Banner Length     W_S     AR      Num_Pucks
ga_settings.lower_bds = [5*0.3048,          96,     4,       1];                        % Set the lower bounds for the optimizer
ga_settings.upper_bds = [30*0.3048,         200,    8,      10];                        % Set the upper bounds for the  optimizer
ga_settings.pop_size = 15;                                                              % Set the population size for GA optimizer
ga_settings.generations = 5;                                                           % Define the number of generations for the GA optimizer
GA_optimizer = optimizer.GA_Optimizer(ga_settings);                                     % Create optimizer object       


% [best_score, best_params] = GA_optimizer.run(@get_GA_cost);
best_params = [8.38192251177078	115.884503942293	5.84665642773867	1.05790321640620];

% Optimized Params

[cost, aircraft, m2_laps, m3_laps] = MOG_Solver(best_params);
aircraft

fprintf("Laps Completed: \n");
fprintf("Mission 2: %d \n", m2_laps);
fprintf("Mission 3 %d\n", m3_laps);





%% Running the MOG Solver

function [cost, aircraft, m2_laps, m3_laps] = MOG_Solver(input_arr)

    %%% Tallying
    persistent total_iterations
    persistent converged_iterations

    if isempty(total_iterations) || isempty(converged_iterations)
        total_iterations = 0;
        converged_iterations = 0;
    end

    fprintf(" \n\n-------------- Total iterations: %d. Converged Iterations %d -------------- \n",...
            total_iterations, converged_iterations);

    %%% Extract inputs
    banner_length    = input_arr(1);
    W_S              = input_arr(2);
    AR               = input_arr(3);
    num_pucks        = ceil(input_arr(4));

    %%% Define Vehicle
    aircraft = Vehicle("Aircraft", W_S, num_pucks, banner_length, 0.7, 0.05);

    % Add fuselage
    aircraft.add_fuselage();

    % Add wing
    taper_ = 1;
    sweep_ = 0;
    airfoil_ = 'NACA2412';
    [~, flag] = aircraft.add_wing(0, AR, taper_, sweep_, airfoil_);

    if ~flag
        cost = 1e8;
        fprintf("Wing too large for specified AR! \n");
        fprintf("Setting cost to %.3e.\n", cost);
        total_iterations = total_iterations + 1;
        return;
    end

    % Add HT
    HT_AR_ = 3.0;
    HT_airfoil_ = "NACA0012";
    aircraft.add_HT(HT_AR_, HT_airfoil_);

    % Add VT
    VT_AR_ = 1.0;
    taper_ = 0.6;
    VT_airfoil_ = "NACA0012";
    aircraft.add_VT(VT_AR_, taper_, 0, VT_airfoil_);

    % Add Propulsion System
    bat_cells = 6;
    bat_Wh = 100;
    motor_kv = 710;
    motor_max_current = 68;
    prop_pitch = 8;
    prop_diam = 11;
    aircraft.add_propulsion(bat_cells, bat_Wh, motor_kv, motor_max_current, prop_pitch, prop_diam);

    aircraft.analyze_airframe();
    

    %%% Define Missions
    flag_dist = 500 * 0.3048;
    vWind = [0 0 0];

    [mission_1_t, mission_1_E, tab] = run_mission_1(aircraft, flag_dist, vWind);
    [mission_2_t, mission_2_E, tab, m2_laps] = run_mission_2(aircraft, flag_dist, vWind);
    [mission_3_t, mission_3_E, tab, m3_laps] = run_mission_3(aircraft, flag_dist, vWind);


    mission_1_E = 0;

    %%% Gather Total Performance
    max_E = max([mission_1_E, mission_2_E, mission_3_E]) / 3600;
    if max_E > 100
        cost = (max_E - 100) * 1e6;
        fprintf("Aircraft failed to meet 100 Wh limit. Total Energy usage of %.3f\n", max_E);
        fprintf("Setting cost to %.3e\n", cost);
        total_iterations = total_iterations + 1;
        return;
    end

    safety_margin = 1.2;

    while (abs(max_E*safety_margin - aircraft.battery_capacity) > 10)
        old_aircraft = aircraft;
        old_E = max_E;
        fprintf("Current Solved Energy is %.3f Wh. Resizing.\n", max_E);
        [~, flag] = aircraft.resize_geom(max_E*safety_margin);

        if ~flag
            fprintf("New Wing too large for specified AR! \n");
            fprintf("Using previous iteration.\n");
            aircraft = old_aircraft;
            max_E = old_E;
            break;
        end


        [mission_1_t, mission_1_E, tab] = run_mission_1(aircraft, flag_dist, vWind);
        [mission_2_t, mission_2_E, tab, m2_laps] = run_mission_2(aircraft, flag_dist, vWind);
        [mission_3_t, mission_3_E, tab, m3_laps] = run_mission_3(aircraft, flag_dist, vWind);


        mission_1_E = 0;

        %%% Gather Total Performance
        max_E = max([mission_1_E, mission_2_E, mission_3_E]) / 3600;
        if max_E > 100
            fprintf("New Aircraft failed to meet 100 Wh limit. Total Energy usage of %.3f\n", max_E);
            fprintf("Using previous iteration.\n");
            aircraft = old_aircraft;
            max_E = old_E;
            break;
        end



    end

    fprintf("Design Converged at %.3f Wh.\n", max_E);
    converged_iterations = converged_iterations + 1;
    total_iterations = total_iterations + 1;


    % Generate Cost
    % To do: find missing variables, create a ground mission function
    % Logan checks: span call on RAC calculation, ducks call
    % Missing: num_laps_2, num_laps_3
    % num_laps_3
    score_GM_Norm = 10;
    time_run = 1.5; % seconds
    time_load_M2 = 2*ceil(aircraft.num_pucks/4) + ceil(2.5*aircraft.num_ducks/6); % fix
    time_unload_M2 = 2*ceil(aircraft.num_pucks/4) + ceil(aircraft.num_ducks/4); % fix
    time_load_M3 = 2; % seconds
    score_GM_AU = 4*time_run + time_load_M2 + time_unload_M2 + time_load_M3;
    score_GM = score_GM_Norm/score_GM_AU;

    get_income = @(n_Pass, n_Cargo, n_Laps) ...
        (n_Pass*(6+(2*n_Laps))) + (n_Cargo * (10 + (8*n_Laps)));
    get_cost = @(n_Laps, n_Pass, n_Cargo, EF) ...
        n_Laps*(10+(n_Pass * 0.5) + (n_Cargo * 2)) * EF;
    income_Norm = get_income(60, 20, 10); % 60 ducks, 20 pucks, 10 laps
    cost_Norm = get_cost(10, 60, 20, 1);
    score_M2_Norm = income_Norm - cost_Norm;
    income_AU = get_income(aircraft.num_ducks, aircraft.num_pucks, m2_laps);
    EF = aircraft.battery_capacity / 100;
    cost_AU = get_cost(m2_laps, aircraft.num_ducks, aircraft.num_pucks, EF);
    score_M2_AU = income_AU - cost_AU;
    score_M2 = score_M2_AU/score_M2_Norm;

    score_M3_Norm = 35*8/0.9; % 35 ft banner, 0.9 RAC, 8 laps
    RAC = max(0.9, 0.05*aircraft.b_ref + 0.75);
    score_M3_AU = banner_length*m3_laps*RAC;
    score_M3 = score_M3_AU/score_M3_Norm;

    score = score_GM + score_M2 + score_M3;
    cost = 1/score * 100; % Higher score generates lower cost.

    fprintf("Generated cost: %.3f\n", cost);


end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% COST WRAPPER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost = get_GA_cost(input_arr)
    
    [cost, ~, ~, ~] = MOG_Solver(input_arr);

end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MISSION DEFINITIONS %%%%%%%%%%%%%%%%%%%%%%%%
function [t_final, E_final, tab] = run_mission_1(aircraft, flag_dist, vWind)

    % % Mission 1  **EXAMPLE**
    mission_1_laps = 3;
    mission_1_steps = 50;
    mission_1 = mission.Mission_Builder(mission_1_laps, aircraft, mission_1_steps, vWind, 'mission 1');



    % Lap 1
    m1_takeoff_throttle = 0.7;
    m1_climb_throttle = 0.7;
    mission_1.add_takeoff(m1_takeoff_throttle, m1_climb_throttle);

    m1_climb_h = 200 * 0.3048;
    mission_1.add_climb(m1_climb_h, m1_climb_throttle);
    [~, ~, tab] = mission_1.run();
    mission_1.clear();

    dist_traveled = tab.Position_NED(end,1);
    m1_lap1_cruise = flag_dist/2 - dist_traveled;
    m1_cruise_throttle = 0.65;
    m1_turn_throttle = 0.8;
    mission_1.add_cruise(m1_lap1_cruise, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 180, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 360, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 180, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);

    % Lap 2 
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 180, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 360, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 180, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);

    % Lap 3
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 180, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 360, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    mission_1.add_turn(m1_turn_throttle, 180, 'sustained');
    mission_1.add_cruise(flag_dist/2, m1_cruise_throttle);
    % somehow land...

    [t_final, E_final, tab] = mission_1.run(tab);
    
   

end

function [t_final, E_final, tab, laps] = run_mission_2(aircraft, flag_dist, vWind)

%%% Mission 2
    mission_2_laps = 3;
    mission_2_steps = 50;
    mission_2 = mission.Mission_Builder(mission_2_laps, aircraft, mission_2_steps, vWind, 'mission 2');

    % Lap 1
    m2_laps = 0;
    m2_takeoff_setting = 0.9;
    m2_climb_throttle = 0.9;
    mission_2.add_takeoff(m2_takeoff_setting, m2_climb_throttle);

    m2_climb_h = 200 * 0.3048;
    mission_2.add_climb(m2_climb_h, m2_climb_throttle);
    [~, ~, tab] = mission_2.run();
    mission_2.clear();

    dist_traveled = tab.Position_NED(end,1);
    m2_lap1_cruise = flag_dist/2 - dist_traveled;
    m2_cruise_throttle = 0.7;
    mission_2.add_cruise(m2_lap1_cruise, m2_cruise_throttle);

    % Finish the lap
    m2_turn_throttle = 0.9;
    mission_2.add_turn(m2_turn_throttle, 180, 'sustained');
    mission_2.add_cruise(flag_dist/2, m2_cruise_throttle);
    mission_2.add_turn(m2_turn_throttle, 360, 'sustained');
    mission_2.add_cruise(flag_dist/2, m2_cruise_throttle);
    mission_2.add_turn(m2_turn_throttle, 180, 'sustained');
    mission_2.add_cruise(flag_dist/2, m2_cruise_throttle);
    
    [t_final, ~, tab] = mission_2.run(tab);
    mission_2.clear();

    m2_laps = m2_laps + 1;
   while (true)
        prev_tab = tab;
        last_t = t_final;

        mission_2.add_cruise(flag_dist/2, m2_cruise_throttle);
        mission_2.add_turn(m2_turn_throttle, 180, 'sustained');
        mission_2.add_cruise(flag_dist/2, m2_cruise_throttle);
        mission_2.add_turn(m2_turn_throttle, 360, 'sustained');
        mission_2.add_cruise(flag_dist/2, m2_cruise_throttle);
        mission_2.add_turn(m2_turn_throttle, 180, 'sustained');
        mission_2.add_cruise(flag_dist/2, m2_cruise_throttle);
    
        [t_final, E_final, tab] = mission_2.run(prev_tab);
        mission_2.clear();
        

        if abs(t_final - 300) < 1e-6
            break;

        elseif t_final > 300
            % If you fail to complete the lap before 5 min, dont count the time for
            % scoring
            t_final = last_t;
            break;
        end

  
        m2_laps = m2_laps + 1;
   end

   laps = m2_laps;


end

function [t_final, E_final, tab, laps] = run_mission_3(aircraft, flag_dist, vWind)

    %%% Mission 3
    banner_CD_0 = aircraft.get_banner_drag(100 * 0.3048);
    banner_area = aircraft.banner_length^2 / 5;
    aircraft.CD_0 = aircraft.CD_0 + banner_CD_0*aircraft.S_ref/banner_area;

    % Lap 1
    mission_3_laps = 3;
    mission_3_steps = 50;
    mission_3 = mission.Mission_Builder(mission_3_laps, aircraft, mission_3_steps, vWind, 'mission 2');

    % Lap 1
    m3_laps = 0;
    m3_takeoff_setting = 0.9;
    m3_climb_throttle = 0.9;
    mission_3.add_takeoff(m3_takeoff_setting, m3_climb_throttle);

    m3_climb_h = 200 * 0.3048;
    mission_3.add_climb(m3_climb_h, m3_climb_throttle);
    [~, ~, tab] = mission_3.run();
    mission_3.clear();

    dist_traveled = tab.Position_NED(end,1);
    m3_lap1_cruise = flag_dist/2 - dist_traveled;
    m3_cruise_throttle = 0.7;
    mission_3.add_cruise(m3_lap1_cruise, m3_cruise_throttle);

    % Finish the lap
    m3_turn_throttle = 0.9;
    mission_3.add_turn(m3_turn_throttle, 180, 'sustained');
    mission_3.add_cruise(flag_dist/2, m3_cruise_throttle);
    mission_3.add_turn(m3_turn_throttle, 360, 'sustained');
    mission_3.add_cruise(flag_dist/2, m3_cruise_throttle);
    mission_3.add_turn(m3_turn_throttle, 180, 'sustained');
    mission_3.add_cruise(flag_dist/2, m3_cruise_throttle);
    
    [t_final, ~, tab] = mission_3.run(tab);
    mission_3.clear();

    m3_laps = m3_laps + 1;
   while (true)
        prev_tab = tab;
        last_t = t_final;

        mission_3.add_cruise(flag_dist/2, m3_cruise_throttle);
        mission_3.add_turn(m3_turn_throttle, 180, 'sustained');
        mission_3.add_cruise(flag_dist/2, m3_cruise_throttle);
        mission_3.add_turn(m3_turn_throttle, 360, 'sustained');
        mission_3.add_cruise(flag_dist/2, m3_cruise_throttle);
        mission_3.add_turn(m3_turn_throttle, 180, 'sustained');
        mission_3.add_cruise(flag_dist/2, m3_cruise_throttle);
    
        [t_final, E_final, tab] = mission_3.run(prev_tab);
        mission_3.clear();
        

        if abs(t_final - 300) < 1e-6
            break;

        elseif t_final > 300
            % If you fail to complete the lap before 5 min, dont count the time for
            % scoring
            t_final = last_t;
            break;
        end

  
        m3_laps = m3_laps + 1;
   end

   laps = m3_laps;

end