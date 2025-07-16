clc, clear, close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%                       AU EAGLE SIZING CODE MAIN                        %
%                                                                        %   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create the Optimizer Object

ga_settings.lower_bds = zeros([1 3]);                % Set the lower bounds for the optimizer
ga_settings.upper_bds = [100 100 100];               % Set the upper bounds for the  optimizer
ga_settings.pop_size = 15;                           % Set the population size for GA optimizer
ga_settings.generations = 5;                         % Define the number of generations for the GA optimizer
GA_optimizer = optimizer.GA_Optimizer(ga_settings);  % Create optimizer object       


[best_score, best_params] = GA_optimizer.run(@EAGLE_Solver);










%% Running the EAGLE Solver


function cost = EAGLE_Solver(vehicle_params)

    % Define Vehicle
    





    % Generate Cost
    cost = 1/score; % Higher score generates lower cost.

end