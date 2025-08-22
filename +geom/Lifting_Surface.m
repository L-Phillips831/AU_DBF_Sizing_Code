classdef Lifting_Surface < Component
    % The Aero class is used for constructing a preliminary aerodynamic
    % buildup by component according to Raymer's method. The main outputs
    % of this class are the drag polar and CL_max of the geometry which are
    % then used for energy absorbtion analysis.
    
    properties
        S (1,1) double
        AR (1,1) double
        span (1,1) double
        taper (1,1) double
        sweep (1,1) double
        x_loc (1,1) double
        z_loc (1,1) double

    end
    
    methods (Access = public)
        % Default constructor of the Aero class.
        % Params:
        %   - vehicle: the vehicle object containing the important
        %              information about the current design iteration
        %
        function obj = Lifting_Surface(S_, AR_, span_, taper_, sweep_)
           obj.S = S_;
           obj.AR = AR_;
           obj.span = span_;
           obj.taper = taper_;
           obj.sweep = sweep_;
           
        end

        function CL_alpha = get_CL_alpha_sub(obj, M)

            AR_ = obj.AR;
            M_ = M;
            eta_ = 0.95;        % Approximation of lift curve slope w/ M recommended by Raymer.
            lambda_max_t = 0;   % Assuming rectangular geometry with no sweep

            % Effect of wing ending geom
            if isfield(obj.geom, 'EndGeom')
                endplate = obj.geom.EndGeom;
                if endplate.style == "Endplate"
                    AR_ = AR_*(1 + 1.9*endplate.h/endplate.b);

                elseif endplate.style == "Winglet"
                    AR_ = AR_*(1 + endplate.h/endplate.b)^2;
                end

            end

            Beta = sqrt(1-M_^2);
            fuse_effects = 0.98;  % Raymer estimation of fuselage effects
            CL_alpha =  2*pi*AR_ / (2 + sqrt(4 + AR_^2*Beta^2 / eta_^2 * (1 + tan(lambda_max_t)^2/Beta^2))) * fuse_effects;


        end



    end

    methods (Static)
        function [SM, NP, flag] = check_long_stab(wing_geom, tail_geoms)

        end

    end
end

