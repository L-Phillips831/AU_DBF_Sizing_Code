classdef Vehicle < handle
    % The vehicle class is the output class of the MOG sizing codebase
    % that contains all pertinant information regarding the current
    % iteration of the design. Multiple vehicle objects will be created by
    % the optimizer in order to maximize mission performance
    
    properties
        name string                                     % Vehicle Name
        components geom.Component                       % struct of components

        prop powerplant.Propulsion                % Propulsion system
        battery_capacity (1,1) double

        banner_length (1,1) double                      % Banner length [m]
        num_pucks (1,1) double {mustBeInteger}
        num_ducks (1,1) double {mustBeInteger}


        CG (3,1) double                                 % X,Y,Z loc of CG [m]

        W_S (1,1) double                                % Vehicle wing loading [kg/m^2]
        P2W (1,1) double
        mission_1_W (1,1) double
        mission_2_W (1,1) double
        mission_3_W (1,1) double
        MTOW (1,1) double = 15/2.2

        S_ref (1,1) double                              % Vehicle reference area  [m]
        c_ref (1,1) double                              % Vehicle reference chord [m]
        b_ref (1,1) double                              % Vehicle reference span  [m]
        HT_coeff (1,1) double                           % Horizontal tail volume coefficient [-]
        VT_coeff (1,1) double                           % Vertical tail volume coefficieint [-]
        aircraft_length (1,1) double                    % Length value use to approximate L_HT [m]
        L_HT (1,1) double                               % Horizontal tail moment arm [m]

        mass (1,1) double                               % Vehicle mass  [kg]
        Cl_alpha (1,1) double                           % Vehicle Cl_alpha [-]
        Cl_0 (1,1) double                               % Vehicle Cl_0 [-]
        CM_alpha (1,1) double                           % Vehicle CM_alpha [-]
        SM (1,1) double                                 % Vehicle Static Margin [-]
        CD_0 (1,1) double                               % Vehicle zero-lift drag coefficient [-]
        K2 (1,1) double                                 % Vehicle parabolic drag coefficient [-]
        K1 (1,1) double                                 % Vehicle linear drag coefficient [-]
    
    end

    properties (Access = private)

        geom_file = "+lib\+AVL\Input\geom_file.avl";
        case_file = "+lib\+AVL\Input\case_file.run";
        cmd_file = "+lib\+AVL\Input\cmd_file.cmd";
        saves_dir =  "+lib\+AVL\Saves";

    end
    
    methods

        % Default constructor of the Vehicle class.
        % Params:
        %   - 
        %
        function obj = Vehicle(name_, W_S_, P2W_, num_pucks_, banner_length_, HT_coeff_, VT_Coeff_)
            obj.name = name_;
            obj.W_S = W_S_;
            obj.P2W = P2W_;
            obj.num_pucks = num_pucks_;
            obj.banner_length = banner_length_;
            obj.HT_coeff = HT_coeff_;
            obj.VT_coeff = VT_Coeff_;

            obj.num_ducks = ceil(floor(num_pucks_ * 3) / 2) * 2;


        end
        

        % Method for adding the main wing to the vehicle component list.
        % Params: 
        %   - x_loc_q4_: double location of quarter chord relative to nose
        %   - z_loc_: double location of wing centerline relative to
        %             fuselage centerline
        %   - AR_: double wing aspect ratio
        %   - taper_: double wing taper ratio. Defined as c_tip / c_root
        %   - sweep_: double angle of constant wing sweep
        %   - airfoil_: string name of airfoil 
        % Returns:
        %   - updated vehicle object with component addition
        %
        function obj = add_wing(obj, x_loc_q4_, z_loc_, AR_, taper_, sweep_, airfoil_)
            wing_area = obj.MTOW / obj.W_S;
            wing_span = sqrt(AR_ * wing_area);
            if wing_span > 5.0
                wing_span = 5;
                AR_ = wing_span^2 / wing_area;
            end

            root_chord_ = wing_area / 0.5 / wing_span / (1 + taper_);

            airfoil_str = fullfile("Airfoils\", sprintf("%s.dat",airfoil_));

            x_loc_ = x_loc_q4_ - 0.25*root_chord_;

            wing = geom.Lifting_Surface("Wing", wing_area, AR_, root_chord_, wing_span, taper_, sweep_, "Wing", airfoil_str);
            wing.place_surface(x_loc_, 0, z_loc_, 0);

            obj.components(end+1) = wing;

            obj.aircraft_length = x_loc_q4_ / 0.4;
            obj.S_ref = wing_area;
            obj.c_ref = root_chord_;
            obj.b_ref = wing_span;

            obj.CG = [x_loc_q4_, 0, 0]; % Place directly on q4 for now
        end


        % Method for adding HT to vehicle component list. Assumed
        % rectangular body
        % Params:
        %   - AR_: double aspect ratio
        %   - airfoil_: string airfoil name
        % Returns:
        %   - updated vehicle object with component addition
        %
        function obj = add_HT(obj, AR_, airfoil_)
            L_HT_ = obj.aircraft_length * 0.6;  % Raymer assumption of L_HT
            obj.L_HT = L_HT_;
            S_HT = obj.S_ref * obj.c_ref * obj.HT_coeff / L_HT_;
            span = sqrt(AR_ * S_HT);
            chord = S_HT / span;

            x_loc_ = obj.aircraft_length - (0.25*chord);

            airfoil_str = fullfile("Airfoils\", sprintf("%s.dat",airfoil_));

            HT = geom.Lifting_Surface("HT", S_HT, AR_, chord, span, 1, 0, "HT", airfoil_str);
            HT.place_surface(x_loc_, 0, 0, 0);

            obj.components(end+1) = HT;
            
        end


        % Method for adding VT to vehicle component list.
        % Params:
        %   - AR_: double aspect ratio
        %   - taper_: double taper ratio
        %   - y_loc_: double y location relative to body x-z plane
        %   - airfoil_: string airfoil name
        % Returns:
        %   -  updated vehicle object with added component
        %
        function obj = add_VT(obj, AR_, taper_, y_loc_, airfoil_)
            L_VT = obj.aircraft_length * 0.6;   % Let L_VT = L_HT for now
            S_VT = obj.S_ref * obj.b_ref * obj.VT_coeff / L_VT;
            span = sqrt(AR_ * S_VT);
            root_chord_ = S_VT / 0.5 / span / (1 + taper_);

            x_loc_ = obj.aircraft_length - (0.25*root_chord_);

            airfoil_str = fullfile("Airfoils\", sprintf("%s.dat",airfoil_));

            VT = geom.Lifting_Surface("VT", S_VT, AR_, root_chord_, span, taper_, 0, "Fin", airfoil_str);
            VT.place_surface(x_loc_, y_loc_, 0, 0);

            obj.components(end+1) = VT;

        end


        % Method for adding a fuselage to the component list.
        % Params:
        %   - length: double fuselage length
        %   - diameter: double fuselage diameter
        % Returns:
        %   - updated vehicle object with added component
        %
        function obj = add_fuselage(obj)
            duck_length = 2.5 * 0.00254;
            puck_thickness = 1 * 0.00254;

            diameter = 4.75 * 0.00254; % [in -> m]

            elec_length = 9 * 0.00254;
            length = elec_length + (obj.num_ducks / 2 * duck_length) + obj.num_pucks*puck_thickness

            fuselage = geom.Fuselage(length, diameter, "Fuselage");

            obj.components(end+1) = fuselage;

        end


        function obj = add_propulsion(obj, bat_cells, bat_Wh, motor_kv, prop_pitch, prop_diam)
            V_per_cell = 3.7;
            V_bat = V_per_cell * bat_cells;
            capacity = bat_Wh / V_bat * 1000;
            CRating = ceil(obj.P2W * obj.MTOW);
            
            bat_ = powerplant.battery(capacity, bat_cells, CRating, "LiPo");
            esc_ = powerplant.esc(100, 100);
            motor_ = powerplant.motor(motor_kv,100,0.1);

            data_file = sprintf("+powerplant//Propeller Data Files//PER3_%.0fx%.0f.dat", prop_diam, prop_pitch);
            propeller_ = powerplant.propeller(prop_pitch, prop_diam, data_file);

            obj.prop = powerplant.Propulsion(bat_,esc_,motor_,propeller_);

            obj.battery_capacity = bat_Wh;


        end

        


        % Method to complete analysis of the entire aircraft. Serves as a
        % wrapper function for avl related methods and component based 
        % weight buildup.
        % Params:
        %   - none
        % Returns:
        %   - updated vehicle object
        %
        function obj = analyze_airframe(obj)

            obj.get_weights();

            % Aero Buildup
            alpha_range = -6:2:20;
            % alpha_range = 2;
            obj.generate_geom_file();
            num_cases = obj.generate_case_file(alpha_range);
            saved_files = obj.generate_cmd_file(num_cases);
            Vehicle.run_avl(obj.cmd_file);
            obj.parse_avl(saved_files);

            airspeed = 30.48;   % airspeed [m/s]. Estimated based on 100 ft/s

            for component = obj.components
                component.get_parasitic_drag(airspeed, obj.S_ref);
                obj.CD_0 = obj.CD_0 + component.CD_0;
            end

            % Add landing gear drag penalty
            obj.CD_0 = obj.CD_0 + 0.01;  % Low fidelity estimation for now
            
        end

        % Method to get the alpha required to meet the specified lift
        % coefficient.
        %
        function alpha = get_req_alpha(obj, CL_req_)

            alpha = (CL_req_ - obj.Cl_0) / obj.Cl_alpha;
        end


        % Method to get the drag induced by the required lift coefficient 
        %
        function CD = get_CD(obj, CL_req_)

            CD = obj.CD_0 + obj.K1 * CL_req_ + obj.K2 * CL_req_^2;

        end

        function obj = resize_geom(obj, new_E)
            % Update new battery size
            obj.prop.battery.energyCapacity = new_E;
            obj.battery_capacity = new_E;

            % Redetermine MTOW
            obj.get_weights();

            % Resize Geometry

            % --- Pass 1: Find and size wing ---
            for component = obj.components
                if isa(component, "Lifting_Surface") && component.style == "Wing"
                    component.S = obj.W_S / obj.MTOW;
                    component.span = min(5.0, sqrt(component.AR * component.S));
                    component.AR   = component.span^2 / component.S;
                    component.chord = component.S / 0.5 / component.span / (1 + component.taper);

                    obj.S_ref = component.S;
                    obj.c_ref = component.chord;
                    obj.b_ref = component.span;

                    root_chord_ = component.S / (0.5*component.span*(1+component.taper));
                    component.x_loc = obj.aircraft_length*0.4 - 0.25*root_chord_;
                end
            end

            % --- Pass 2: Size the rest ---
            for component = obj.components
                if isa(component, "Lifting_Surface")
                    switch component.style
                        case "HT"
                            S_HT = obj.S_ref * obj.c_ref * obj.HT_coeff / obj.L_HT;
                            span = sqrt(component.AR * S_HT);
                            chord = S_HT / span;

                            component.S = S_HT;
                            component.span = span;
                            component.chord = chord;
                            component.x_loc = obj.aircraft_length - 0.25*chord;

                        case "Fin"
                            L_VT = obj.L_HT;
                            S_VT = obj.S_ref * obj.b_ref * obj.VT_coeff / L_VT;
                            span = sqrt(component.AR * S_VT);
                            root_chord_ = S_VT / (0.5*span*(1+component.taper));

                            component.S = S_VT;
                            component.span = span;
                            component.chord = root_chord_;
                            component.x_loc = obj.aircraft_length - 0.25*root_chord_;
                    end
                end
            end 

            % Re-run analysis
            obj.analyze_airframe();


        end


        function CD = get_banner_drag(obj, V)
            
            Re = 0.002377 * V * obj.banner_length / 3.737e-7;
            Re_crit = 10e4;

            CD = 0.108*Re^(-0.2) * min([(1 + 0.8*log(Re/Re_crit)), 6]);

        end
        
    end

    methods (Access = private)


        function obj = get_weights(obj)
            E_Battery = obj.prop.battery.energyCapacity; % [Wh]
            num_pucks_ = obj.num_pucks;
            num_Ducks = obj.num_ducks;

            L_Banner = obj.banner_length; % [m]

            % To do:
            % - Add a better description of method
            % - Get Logan's formatting

            W0 = 7; % [kg]
            A = 5.098; % Based on previous years
            C = -0.791; % Based on previous years
            rho_Battery = 155; % [Wh/kg]
            rho_Banner = 0.08; % [kg/m^2], approximation for light nylon and mesh
            mass_Puck = 0.163; % [kg/puck]
            mass_Duck = 0.017; % [kg/duck]

            % Battery Weight
            W_Battery = E_Battery / rho_Battery;

            % Payload Weight
            M1_payload = 0;
            M2_payload = mass_Duck * num_Ducks + mass_Puck * num_pucks_;
            M3_payload = 0.2 * L_Banner^2 * rho_Banner;

            max_payload = max([M1_payload, M2_payload, M3_payload]);

            % Converge on MTOW
            for i = 1:100
                W_Empty_Frac = A*W0^C;
                W_Empty = W_Empty_Frac * W0;
                W0_Prev = W0;
                W0 = W_Empty + max_payload + W_Battery;
                if abs((W0_Prev - W0)/W0) < 0.001
                    break;
                end
            end

            obj.MTOW = W0;
            obj.mission_1_W = W_Empty + M1_payload + W_Battery;
            obj.mission_2_W = W_Empty + M2_payload + W_Battery;
            obj.mission_3_W = W_Empty + M3_payload + W_Battery;


        end


        % Method to generate the complete geom input file for AVL analysis.
        % Loops through each user added component and adds its
        % corresponding surface into the file.
        % Params:
        %   - current object
        % Returns:
        %   - none
        %
        function generate_geom_file(obj)

            if exist(obj.geom_file, 'file')
                delete(obj.geom_file)
            end


            str = [
                    obj.name,...
                    "#Mach",...
                    "0.0",...    
                    "#IYsym   IZsym   Zsym",...
                    "0       0       0.0",...
                    "#Sref    Cref    Bref",...
                    sprintf("%.1f %.1f %.1f", obj.S_ref, obj.c_ref,  obj.b_ref),...
                    "#Xref    Yref    Zref",...
                    sprintf("%.1f %.1f %.1f", obj.CG(1), obj.CG(2), obj.CG(3)),...
                    ];

            for component = obj.components
                str = [str, component.convert_to_avl_geom()]; 
            end

            writelines(str, obj.geom_file);

        end


        % Method to generate case file for running alpha sweep in AVL.
        % Params:
        %   - current object
        %   - alpha_range: Double array of aoa values
        % Returns:
        %   - the number of cases generated
        %
        function num_cases = generate_case_file(obj, alpha_range)

    
            if exist(obj.case_file, 'file')
                delete(obj.case_file)
            end


            num_cases = numel(alpha_range);

            str = "";

            for i = 1:num_cases
              new_str = [
                         "----------------------------------------------",...
                         sprintf("Run case  %1.0f:  -unnamed- ", i),...
                         "",...
                         sprintf("alpha        ->  alpha       =   %.5f", alpha_range(i)),...
                         "beta         ->  beta        =   0.00000",...    
                         "pb/2V        ->  pb/2V       =   0.00000",...    
                         "qc/2V        ->  qc/2V       =   0.00000",...    
                         "rb/2V        ->  rb/2V       =   0.00000",...    
                         "flap         ->  flap        =   0.00000",...    
                         "aileron      ->  aileron     =   0.00000",...    
                         "elevator     ->  elevator    =   0.00000",...    
                         "rudder       ->  rudder      =   0.00000",...    
                            ];

              str = [str, new_str];
            end


            writelines(str, obj.case_file);

        end

        % Method to generate the keystroke commands to be fed to avl for
        % running all test cases and saving to separate files.
        % Params:
        %   - the current object
        %   - num_cases: Double number of cases to be ran
        % Returns:
        %   - saved_files: str array of filepaths containing solved data
        %
        function saved_files = generate_cmd_file(obj, num_cases)
            
    
            if exist(obj.cmd_file, 'file')
                delete(obj.cmd_file)
            end


            str = [
                   sprintf("LOAD %s", obj.geom_file),...
                   sprintf("CASE %s", obj.case_file),...
                   "OPER",...
            ];

            saved_files = [];

            for i = 1:num_cases

                current_save = fullfile(obj.saves_dir, sprintf("Case_%1.0f.txt", i));

                if exist(current_save, 'file')
                    delete(current_save);
                end

                saved_files = [saved_files, current_save];

                new_str = [
                            "X",...
                            sprintf("ST %s", current_save),...
                            "-",...
                            ];

                str = [str, new_str];
            end

            str = [str, "QUIT"];

            writelines(str, obj.cmd_file);

        end




        % Method for parsing avl solved data and updating object properties 
        % with solved aero data.
        % Params:
        %   - saved_files: str array of filepaths to saved files
        % Returns:
        %   - Updated object with aero data
        %
        function obj = parse_avl(obj, saved_files)

            alpha_arr = zeros(numel(saved_files), 1);
            CL_arr    = zeros(numel(saved_files), 1);
            Cm_arr    = zeros(numel(saved_files), 1);
            CD_arr    = zeros(numel(saved_files), 1);



            getValue = @(text, keyword) str2double( ...
                    regexp(text, keyword + "\s*=?\s*([-\d\.Ee]+)", 'tokens', 'once') ...
                );

            for f = 1:numel(saved_files)
                lines = readlines(saved_files(f));
                text = strjoin(lines, " ");

                

                alpha_arr(f) = getValue(text, "Alpha");
                CL_arr(f)    = getValue(text, "CLtot");
                Cm_arr(f)    = getValue(text, "Cmtot");
                CD_arr(f)    = getValue(text, "CDtot");
            end

        
        [alpha_arr, sort_idx] = sort(alpha_arr);
        CL_arr = CL_arr(sort_idx); obj.Cl_0 = CL_arr(alpha_arr == 0);
        Cm_arr = Cm_arr(sort_idx);
        CD_arr = CD_arr(sort_idx);

        alpha_arr = alpha_arr .* pi / 180;
        Cl_fit = polyfit(alpha_arr, CL_arr, 1);
        obj.Cl_alpha = Cl_fit(1);
        CM_fit = polyfit(alpha_arr, Cm_arr, 1); obj.CM_alpha = CM_fit(1);
        CD_fit = polyfit(CL_arr, CD_arr, 2); 
        obj.K2 = CD_fit(1); obj.K1 = CD_fit(2); 

        obj.SM = -obj.CM_alpha / obj.Cl_alpha;
        
        end

    end

    methods (Static)


        function [status, cmd_out] = run_avl(cmd_file)

            [status, cmd_out] = system(sprintf('"+lib/+AVL/avl" < "%s"', cmd_file)); 

        end


        % Method to obtain the DCM from Inertial to Body for a given set of
        % Euler angles.
        %
        function BI = get_DCM_BI(ph, th, ps)

            BI = [
                  cosd(th)*cosd(ps)                                 cosd(th)*sin(ps)                                    -sind(th)
                  sind(ph)*sind(th)*cosd(ps) - cosd(ph)*sind(ps)    sind(ph)*sind(th)*sind(ps) + cosd(ph)*cosd(ps)       sind(ph)*cosd(th)
                  cosd(ph)*sind(th)*cosd(ps) + sind(ph)*sind(ps)    cosd(ph)*sind(th)*sind(ps) - sind(ph)*cosd(ps)       cosd(ph)*cosd(th)
                  ];

        end

        % Method to reconcile the wind frame L and D into the corresponding
        % body forces. Assumes no sideslip.
        %
        function [Fx_b, Fz_b] = reconcile_L_D(L, D, alpha)

           Fx_b = +L*sind(alpha) - D*cosd(alpha);
           Fz_b = -L*cosd(alpha) - D*sind(alpha);

        end

    end

    

end

