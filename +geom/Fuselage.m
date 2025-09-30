classdef Fuselage < geom.Component
    %FUSELAGE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name string
        style = 'Fuselage'
        diam (1,1) double
        length (1,1) double

        CD_0 (1,1) double

    end
    
    methods
        function obj = Fuselage(length, diameter, name)
            obj.length = length;
            obj.diam = diameter;
            obj.name = name;

        end
        
        function obj = get_parasitic_drag(obj, airspeed, S_ref)
            fuselage_lam_length = 0.1;  % Fraction of laminar flow over fuselage
            rho_air = 1.225;            % Air density [kg/m^3]
            air_dyn_visc = 1.81e-5;     % Air dynamic viscocity [Pa*s]

            %%% Flat plate coefficient 
            % Laminar Section
            Re_lam = rho_air * airspeed * (fuselage_lam_length * obj.length) / air_dyn_visc;
            Cf_lam = 1.328/(Re_lam^0.5);

            % Turbulent Section
            Re_total = rho_air * airspeed * obj.length / air_dyn_visc;
            Cf_turb = 0.074 / (Re_total^0.2);

            Cf = (1 - fuselage_lam_length) * Cf_turb + Cf_lam * fuselage_lam_length;

            %%% Form Factor Calculation
            f_fuse = obj.length / obj.diam;
            FF = (0.9 + 5/(f_fuse^1.5) + f_fuse / 400);

            %%% Parasitic Drag Buildup
            interference_factor = 1.3;
            S_wet = pi*obj.diam* obj.length;
            obj.CD_0 = FF * Cf * S_wet/S_ref * interference_factor;

        end

        function str = convert_to_avl_geom(obj)
            str  = "";
        end
    end
end

