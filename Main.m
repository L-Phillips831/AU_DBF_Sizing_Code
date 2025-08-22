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


[best_score, best_params] = GA_optimizer.run(@MOG_Solver);
MOG_Solver([])










%% Running the MOG Solver


function cost = MOG_Solver(vehicle_params)

    % Define Vehicle
    aircraft = Vehicle([], []);




    

    %%% Define Missions

    % Mission 1  **EXAMPLE**
    mission_1_laps = 3;
    mission_1_steps = 50;
    mission_1_cruise_alt = 200;
    mission_1_throttle = 0.6;
    mission_1_flag_dist = 500;

    mission_1 = mission.Mission_Builder(mission_1_laps, aircraft, mission_1_steps);

    % Lap 1
    mission_1.add_takeoff();
    mission_1.add_climb(mission_1_cruise_alt, mission_1_throttle, 200);
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(180, 'instantaneous');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(360, 'sustained');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(180, 'instantaneous');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);

    % Lap 2
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(180, 'instantaneous');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(360, 'sustained');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(180, 'instantaneous');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);

    % Lap 3
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(180, 'instantaneous');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(360, 'sustained');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    mission_1.add_turn(180, 'instantaneous');
    mission_1.add_cruise(mission_1_flag_dist/2, mission_1_throttle);
    % somehow land...

    [t_total, E_total] = mission_1.run();




    % Generate Cost
    score = 10;
    cost = 1/score; % Higher score generates lower cost.

end