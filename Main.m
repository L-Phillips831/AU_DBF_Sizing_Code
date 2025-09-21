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
MOG_Solver(10*0.3048, 5.16, 4.4, 3);










%% Running the MOG Solver


function cost = MOG_Solver(banner_length, W_S, AR, num_pucks)

    %%% Define Vehicle
    aircraft = Vehicle("Aircraft", W_S, num_pucks, banner_length, 0.7, 0.05);

    % Add wing
    wing_q4 = 0.5; % [m]
    taper_ = 1;
    sweep_ = 0;
    airfoil_ = 'NACA2412';
    aircraft.add_wing(wing_q4, 0, AR, taper_, sweep_, airfoil_);

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

    % Add Propulsion System
    bat_cells = 6;
    bat_Wh = 100;
    motor_kv = 710;
    motor_max_current = 68;
    prop_pitch = 8;
    prop_diam = 11;
    aircraft.add_propulsion(bat_cells, bat_Wh, motor_kv, motor_max_current, prop_pitch, prop_diam);

    aircraft.analyze_airframe()
    aircraft.get_banner_drag(100 * 0.3048)
    

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



    %%% Mission 2
    flag_dist = 500 * 0.3048;
    mission_2_laps = 3;
    mission_2_steps = 50;
    vWind = [0 0 0];
    mission_2 = mission.Mission_Builder(mission_2_laps, aircraft, mission_2_steps, vWind, 'mission 2');

    % Lap 1
    m2_laps = 0;
    m2_takeoff_setting = 0.9;
    m2_climb_throttle = 0.8;
    mission_2.add_takeoff(m2_takeoff_setting, m2_climb_throttle);

    m2_climb_h = 200 * 0.3048;
    mission_2.add_climb(m2_climb_h, m2_climb_throttle);
    [t_final, ~, tab] = mission_2.run()
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


    mission_2_E = E_final;
    mission_2_t = t_final;





    %%% Mission 3



    %%% Gather Total Performance
    max_E = max([mission_1_E, mission_2_E, mission_3_E]);
    if max_E > 100
        cost = 10e6;
    end

    safety_margin = 1.2;

    while (max_E - aircraft.battery_capacity > 1)
        aircraft.resize_geom(max_E*safety_margin);
        mission_1.run();
        mission_2.run();
        mission_3.run();
    end



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
    income_AU = get_income(aircraft.num_ducks, aircraft.num_pucks, num_laps_2);
    EF = aircraft.battery_capacity / 100;
    cost_AU = get_cost(num_laps_2, aircraft.num_ducks, aircraft.num_pucks, EF);
    score_M2_AU = income_AU - cost_AU;
    score_M2 = score_M2_AU/score_M2_Norm;

    score_M3_Norm = 35*8/0.9; % 35 ft banner, 0.9 RAC, 8 laps
    RAC = max(0.9, 0.05*aircraft.b_ref + 0.75);
    score_M3_AU = banner_length*num_laps_3*RAC;
    score_M3 = score_M3_AU/score_M3_Norm;

    score = score_GM + score_M2 + score_M3;
    cost = 1/score; % Higher score generates lower cost.

end