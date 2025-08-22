classdef Lifting_Surface < Component & handle
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
        y_loc (1,1) double
        rotation (1,1) double

        airfoil_str string
        Cl_alpha (1,1) Double
        Cl_max (1,1) Double

    end
    
    methods (Access = public)
        % Default constructor of the Lifting surface class.
        % Params:
        %   - S_: Double reference area
        %   - AR_: Double aspect ratio
        %   - span_: Double span
        %   - taper_: Double taper ratio
        %   - sweep_: Double sweep
        %   - airfoil_: String name of airfoil
        % Returns:
        %   - updated object
        %
        function obj = Lifting_Surface(S_, AR_, span_, taper_, sweep_, airfoil_)
           obj.S = S_;
           obj.AR = AR_;
           obj.span = span_;
           obj.taper = taper_;
           obj.sweep = sweep_;
           obj.airfoil_str = airfoil_;
           
        end


        % Method to set the placement and orientation of a lifting surface. Used for
        % aero buildup and stability calculations. X loc is measuered
        % relative to datum, y and z loc measured relative to fuselage centerline
        % rotation relative to x-y plane and rotating about x axis
        % Params:
        %   - x_loc_: Double x location
        %   - y_loc_: Double y location
        %   - z_loc_: Double z location
        %   - rot_: Double rotation angle
        %  Returns:
        %   - updated object
        %
        function obj = place_surface(x_loc_, y_loc_, z_loc_, rot_)
            obj.x_loc = x_loc_;
            obj.z_loc = z_loc_;
            obj.y_loc = y_loc_;
            obj.rotation = rot_;
                
        end

        function obj = get_2D_performance(obj, cruise_v)
            
            if obj.airfoil_str == ""
                error("Airfoil is not set. Unable to get performance.")
            end



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

