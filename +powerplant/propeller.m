classdef propeller
    % Propeller class for Auburn DBF sizing code uses APC propeller data to
    % create propeller performance codes for use in sizing scripts

    properties
        perfData (1,:) cell % Cell array of tables containing propellor performance data
        GI_Thrust_fcn_J_RPM griddedInterpolant = griddedInterpolant(1:2,1:2) % Gridded interpolant to calculate Thrust {N} from J and RPM
        GI_Power_fcn_J_RPM griddedInterpolant = griddedInterpolant(1:2,1:2) % Gridded interpolant to calculate Power reqiured {W} from J and RPM
        pitch (1,1) double % Propellor pitch
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
            TableFlag = 0;
            n = 0;
            i = 1;

            % Initialize Data array
            Data = zeros(30,15,50);
            
            % Open .dat file and start reading line by line
            perfDataFile
            fid = fopen(perfDataFile)
            tline = fgetl(fid);
            
            % Loop until a line has no characters in it. This should be end
            % of the text file.
            while ischar(tline)
                tline = fgetl(fid);
                if tline == -1
                    
                elseif contains(tline,"RPM =") % If a line contains "RPM =" sets flag to grab data on next line
                    i = i+1;
                    n = n+1;
                    TableFlag = 1;
                elseif TableFlag == 1 % Gets prop data and resets TableFlag
            
                    Data(:,:,i-1) = readmatrix(perfDataFile,"Range",string(n+5)+":"+string(n+35));
                    TableFlag = 0;
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

            fclose(fid);
        end
    
    end

    methods(Access=private)
        function prop = mkGriddedInterps(prop)
            % Creates gridded interpolant for thrust {N} and power {W} as a function of
            % advance ratio (j) and RPM.

            % Loop through all advance ratios and get maximum value.
            Jmax = 0;
            for i=1:length(prop.perfData)
                Jtest = max(prop.perfData{i}.J);
                if Jtest > Jmax
                    Jmax = Jtest;
                end
            end

            % Create vector of common advance ratios and RPM values
            Jvec = linspace(0,Jmax,100)';
            RPMvec = (1:length(prop.perfData)) * 1000;

            % Convert vectors to matrices
            Jmat = repmat(Jvec,1,length(RPMvec));
            RPMmat = repmat(RPMvec,length(Jvec),1);
            
            % Initialize Thrust and Power matrices
            Tmat = zeros(length(Jvec),length(prop.perfData));
            PWRmat = zeros(length(Jvec),length(prop.perfData));
            
            % Get values for Thrust and PWR at each advance ratio and RPM
            for i=1:length(prop.perfData)
                % Remove any NaN from data and interpolate to generate new data
                J = prop.perfData{i}.J(~isnan(prop.perfData{i}.Thrust_N));
                T = prop.perfData{i}.Thrust_N(~isnan(prop.perfData{i}.Thrust_N));
                P = prop.perfData{i}.PWR_W(~isnan(prop.perfData{i}.Thrust_N));

                Tmat(:,i) = interp1(J,T,Jvec,'spline','extrap');
                PWRmat(:,i) = interp1(J,P,Jvec,'spline','extrap');
            end
            % Make any negative values in the pwoer matrix 0
            PWRmat(Tmat<0) = 0;
            
            % Make gridded interpolants from matrices
            prop.GI_Thrust_fcn_J_RPM = griddedInterpolant(Jmat,RPMmat,Tmat,'linear','linear');
            prop.GI_Power_fcn_J_RPM = griddedInterpolant(Jmat,RPMmat,PWRmat,'linear','nearest');
            
        end

    end
end