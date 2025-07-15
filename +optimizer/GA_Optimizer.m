classdef GA_Optimizer
    % This class delas with the construction and usage of the optimization
    % using MATLAB's Genetic Algorithm optimization package.
    
    properties
        pop_size (1,1) uint16       % Population size for GA solver
        generations (1,1) uint16    % Number of generations for GA solver
    end
    
    methods

        % Default constructor for the GA optimizer class.
        % Params:
        %   - settings: struct containing the necesssary settings
        %               for initializing the MATLAB GA function
        %
        function obj = GA_Optimizer(settings_)
            obj.pop_size = settings_.pop_size;
            obj.generations = settings_.generations;
        end
        
        % Method for running the GA optimization
        % Params:
        %   - obj: the instance of the GA class
        % Returns:
        %   - best_score: the minimized cost returned by the genetic
        %                 algorithm
        %   - best_params: the array of optimal params determined by the GA
        %                  solver
        function [best_score, best_params, flag] = run(obj)
           
        end
    end
end

