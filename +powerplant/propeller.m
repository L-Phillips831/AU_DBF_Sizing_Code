classdef propeller
    % Propeller class for Auburn DBF sizing code uses APC propeller data to
    % create propeller performance codes for use in sizing scripts

    properties
        perfData (1,:) cell % Cell array of tables containing propellor performance data
        GI_Thrust_fcn_V_RPM griddedInterpolant = griddedInterpolant(1:2,1:2) % Gridded interpolant to calculate Thrust {N} from V {m/s} and RPM
        GI_Power_fcn_V_RPM griddedInterpolant = griddedInterpolant(1:2,1:2) % Gridded interpolant to calculate Power reqiured {W} from V {m/s} and RPM
        GI_RPM_fcn_V_Thrust griddedInterpolant = griddedInterpolant(1:2,1:2) % Gridded interpolant to calculate RPM reqiured from V {m/s} and Thrust reqiured {N}
        pitch (1,1) double % Propellor pitch {in}
        diameter (1,1) double % Propellor diameter {m}
    end

    methods
        function prop = propeller(pitch,diameter,perfDataFile)
            % Constructor method. Creates the propeller class. Inputs are
            % pitch, diameter {in}, and an APC format propellor performance
            % ".dat" file. 

            prop.pitch = pitch;
            prop.diameter = diameter;
            prop.perfData = prop.mkPropData(perfDataFile);
            prop = prop.mkGriddedInterps; 
        end

    end

    methods(Static)
        function perfData = mkPropData(perfDataFile)

            % Static method to convert APC format propellor performance
            % ".dat" files to a cell array of matlab tables.

            % Initialize counters and flags
            TableFlag = false;
            n = 0;
            i = 1;

            % Initialize Data array
            Data = zeros(30,15,50);
            
            % Open .dat file and start reading line by line
            fid = fopen(perfDataFile);
            tline = fgetl(fid);
            
            % Loop until a line has no characters in it. This should be end
            % of the text file.
            while ischar(tline)
                tline = fgetl(fid);
                if tline == -1
                    
                elseif contains(tline,"RPM =") % If a line contains "RPM =" sets flag to grab data on next line
                    i = i+1;
                    n = n+1;
                    TableFlag = true;
                elseif TableFlag == true % Gets prop data and resets TableFlag

                    Data(:,:,i-1) = readmatrix(perfDataFile,"Range",string(n+5)+":"+string(n+35));
                    TableFlag = false;
                    n = n+1;
                else
                    n = n+1;
                end
            
            end
            
            % Initializes cell array of correct length
            perfData = {1,i-1};
            
            % Loop through each layer of array and create formatted table
            for j=1:i-1
                perfData{j} = array2table(Data(:,:,j));
                perfData{j}.Properties.VariableNames = {'V_mph','J','Pe','Ct','Cp','PWR_Hp','Torque_in_lbf',...
                    'Thrust_lbf','PWR_W','Torque_Nm','Thrust_N','THR2PWR_g2W','Mach','Reyn','FOM'};
                perfData{j}.Properties.UserData.RPM = j*1000; % Sets custom table RPM property for later reference
            end
        end
    
    end

    methods(Access=private)
        function prop = mkGriddedInterps(prop)
            % Creates gridded interpolant for thrust {N} and power {W} as a function of
            % velocity {m/s} and RPM.

            % Loop through all velocities and get maximum value.
            Vmax = 0;
            for i=1:length(prop.perfData)
                Vtest = max(prop.perfData{i}.V_mph) * 0.44704;
                if Vtest > Vmax
                    Vmax = Vtest;
                end
            end

            % Create vector of common velocities and RPM values
            Vvec = linspace(0,Vmax,100)';
            RPMvec = (1:length(prop.perfData)) * 1000;

            % Convert vectors to matrices
            Vmat = repmat(Vvec,1,length(RPMvec));
            RPMmat = repmat(RPMvec,length(Vvec),1);
            
            % Initialize Thrust and Power matrices
            Tmat = zeros(length(Vvec),length(prop.perfData));
            PWRmat = zeros(length(Vvec),length(prop.perfData));
            
            % Get values for Thrust and PWR at each advance ratio and RPM
            for i=1:length(prop.perfData)
                % Remove any NaN from data and interpolate to generate new data
                V = prop.perfData{i}.V_mph(~isnan(prop.perfData{i}.Thrust_N)) * 0.44704;
                T = prop.perfData{i}.Thrust_N(~isnan(prop.perfData{i}.Thrust_N));
                P = prop.perfData{i}.PWR_W(~isnan(prop.perfData{i}.Thrust_N));

                Tmat(:,i) = interp1(V,T,Vvec,'linear','extrap');
                PWRmat(:,i) = interp1(V,P,Vvec,'linear','extrap');
            end
            % Make any negative values in the power matrix 0
            PWRmat(Tmat<0) = 0;
            
            % Make gridded interpolants from matrices
            prop.GI_Thrust_fcn_V_RPM = griddedInterpolant(Vmat,RPMmat,Tmat,'linear','linear');
            prop.GI_Power_fcn_V_RPM = griddedInterpolant(Vmat,RPMmat,PWRmat,'linear','nearest');

            % Make mirrored GI for getting RPM for a given thrust reqiured
            prop = prop.mkRPMGriddedInterp;
            
        end

        function prop = mkRPMGriddedInterp(prop)
            % Creates a GI for RPM as a function of velocity {m/s}
            % and thrust reqiured {N}
            
            % get the maximum thrust value
            maxThrust = max(max(prop.GI_Thrust_fcn_V_RPM.Values));
            
            % create vector of common velocities and thrusts
            Tvec = linspace(0,maxThrust,50);
            Vvec = prop.GI_Thrust_fcn_V_RPM.GridVectors{1,1}';
            
            % convert vectors to matrices
            Vmat = repmat(Vvec,1,length(Tvec));
            Tmat = repmat(Tvec,length(Vvec),1);

            % get RPM test vector and initialize RPM matrix
            RPMvec = prop.GI_Thrust_fcn_V_RPM.GridVectors{1,2};
            RPMmat = zeros(length(Vvec),length(Tvec));

            % get rpm value for each velocity and thrust pair
            for i=1:length(Vvec)
                % get thrust values aligned with RPM test vector
                T = prop.GI_Thrust_fcn_V_RPM(Vvec(i).*ones(1,length(RPMvec)),RPMvec);

                % get RPM for the common thrust value
                RPMmat(i,:) = interp1(T,RPMvec,Tvec,'linear');
            end

            % create gridded interpolants from matrices
            prop.GI_RPM_fcn_V_Thrust = griddedInterpolant(Vmat,Tmat,RPMmat,'linear','linear');
        end

    end
end