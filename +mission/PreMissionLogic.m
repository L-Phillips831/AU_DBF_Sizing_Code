%% PreMissionLogic %%
% Description: Logic that joins together the resizer methods and the
% mission model. Calculates required values for the mission model such as
% optimal climb characteristics, etc. Probably needs to be run for
% multiple mission configurations due to changing weights (payload/battery)

% Required values:
% CL_max
% mass
% g (gravitational acceleration)
% Sref
% rho
% k

% Additional options:
% - Have hard coded values like ground friction coefficient and nVals
% - logic that differentiates tricycle gear configs (aoaSet, vRot, etc)



% aoaLOF = The angle of attack for takeoff

% vLOF = velocity required for liftoff, assumed to be 1.3 * stall
weight = mass * g;
vStall = sqrt(weight*2/(Sref * rho * CL_Max));
vLOF = 1.3 * vStall;