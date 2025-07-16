classdef GA_Optimizer
    % This class delas with the construction and usage of the optimization
    % using MATLAB's Genetic Algorithm optimization package. Serves to
    % simplify the user interface with dealing with the GA optimizer
    
    properties
        pop_size (1,1) uint16       % Population size for GA solver
        generations (1,1) uint16    % Number of generations for GA solver
        lower_bds double            % Array of lower limits for GA solver
        upper_bds double            % Array of upper limits for GA solver
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
            obj.lower_bds = settings_.lower_bds;
            obj.upper_bds = settings_.upper_bds;

            % Checking user input for parameter vector
            if (length(obj.lower_bds) ~= length(obj.upper_bds))
                error('Lower and upper bounds must have the same length. Got %d and %d.', ...
                length(obj.lower_bds), length(obj.upper_bds));
            end

        end
        
        % Method for running the GA optimization
        % Params:
        %   - obj: the instance of the GA class
        %   - cost_handle: the handle to the cost function created in the
        %                   main script. This allows the cost function to be more easily
        %                   managed by the user since this function defines the parameters
        %                   for optimization
        % Returns:
        %   - best_score: the minimized cost returned by the genetic
        %                 algorithm
        %   - best_params: the array of optimal params determined by the GA
        %                  solver
        function [best_score, best_params] = run(obj, cost_handle)


            % Options for solver
            ga_opt = optimoptions('ga','Display','off','Generations', obj.generations, 'PopulationSize',obj.pop_size,'PlotFcns',@gaplotbestf);

            % Call the solver
            num_var = length(obj.upper_bds);
            [best_params, best_score] = ga(cost_handle,num_var,[],[],[],[],obj.lower_bds, obj.upper_bds,[],ga_opt);

        end


    end    
    
end

