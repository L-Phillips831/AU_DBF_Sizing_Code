classdef GA_Optimizer
    % This class delas with the construction and usage of the optimization
    % using MATLAB's Genetic Algorithm optimization package.
    
    properties
        pop_size (1,1) uint16       % Population size for GA solver
        generations (1,1) uint16    % Number of generations for GA solver
        lower_bds  (1,3) double     % Array of lower limits for GA solver
        upper_bds (1,3) double      % Array of upper limits for GA solver
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
        end
        
        % Method for running the GA optimization
        % Params:
        %   - obj: the instance of the GA class
        % Returns:
        %   - best_score: the minimized cost returned by the genetic
        %                 algorithm
        %   - best_params: the array of optimal params determined by the GA
        %                  solver
        function [best_score, best_params] = run(obj)


            % Options for solver
            ga_opt = optimoptions('ga','Display','off','Generations', obj.generations, 'PopulationSize',obj.pop_size,'PlotFcns',@gaplotbestf);

            % Call the solver
            num_var = length(obj.upper_bds);
            [best_params, best_score] = ga(@GA_Optimizer.cost_generation,num_var,[],[],[],[],obj.lower_bds, obj.upper_bds,[],ga_opt);

        end


    end
    methods (Static)

         function cost = cost_generation(params)
            T2W = params[1];
            W_S = params[2];
            AR = Params[3];

         end

    end
    
end

