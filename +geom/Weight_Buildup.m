%% WeightBuildup %%
%{
Description:
A payload and battery based MTOW buildup. Empty weight is an implicit
function of historical competition exponential constants and MTOW. Battery
weight is a function of battery energy and energy density averaged across
several suppliers and energy amounts.

Intended Use:
During iteration of energy consumption across functions, input the battery
energy, payload/scoring details passed in, and select the mission being 
ran. It is good practice to estimate weight for the heaviest mission for
each battery energy iteration
%}

% Inputs:
% E_Battery [Wh]
% num_Ducks
% num_Pucks
% length_Banner [m]
% Mission_Flag
% (E_Battery, Mission_Flag, num_Ducks, num_Pucks, L_Banner)

% Outputs:
% W0
% W_Empty
% W_Battery
% W_Payload
% (W0, W_Empty, W_Battery, W_Payload)


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
switch Mission_Flag
    case 2 % Mission 2 payload summation
        W_Payload = mass_Duck * numDucks + mass_Puck * num_Pucks;
    case 3 % Mission 3 payload summation
        W_Payload = 0.2 * L_Banner^2 * rho_Banner;
    otherwise
        W_Payload = 0;
end

% Empty Weight
% Note: Includes attached banner mechanisms and M2 payload securement
for i = 1:100
    W_Empty_Frac = A*W0^C;
    W_Empty = W_Empty_Frac * W0;
    W0_Prev = W0;
    W0 = W_Empty + W_Payload + W_Battery;
    if abs((W0_Prev - W0)/W0) < 0.001
        break;
    end
end

W0 = W_Empty + W_Payload + W_Battery;

