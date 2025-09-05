%% WeightBuildup %%
%{
Description:
A physics-based weight build-up that takes aircraft sizing and loading
parameters and estimates the total a/c mass. Each component (wing, tail,
fuselage, etc) will have its respective mass approximation method.
Part-by-part weight buildup is utilized with adjustment factors to find
component masses. Adjustment factors account for the small inaccuracies
between these method's estimates and the true fabricated masses. This
buildup, for now, is suited for conventional configuration aircraft
%}

% W0 = W_Empty + W_Payload + W_Fuel
% W_Empty = A*W0_Guess^C; Historical Data Implicit Calculation
% W_Fuel = W_Battery
% W_PL = f(ducks,pucks, payload)

% Inputs:
% E_Battery [Wh]
% num_Ducks
% num_Pucks
% length_Banner [m]
% W0_Guess [kg]

% To do:
% - Make convergence logic for empty weight
% - Add flags for M1, M2, or M3
% - Add a better description of method
% - Get Logan's formatting

A = 2.36;
C = -0.18;
rho_Battery = 155; % [Wh/kg]

% Battery Weight
W_Battery = E_Battery / rho_Battery;

% Payload Weight

% Empty Weight
for i = 1:20
    W_Empty = A*W0^C;
    W0 = W_Empty + W_Payload + W_Battery;
end

W0 = W_Empty + W_Payload + W_Battery;

