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
aoaLOF = (CL_TO - CL0) / CL_Alpha;

% vLOF = velocity required for liftoff, assumed to be 1.3 * stall
weight = mass * g;
vStall = sqrt(weight*2/(Sref * rho * CL_Max));
vLOF = 1.3 * vStall;

% aoaSet = The aoa at forward ground run
if TailDragger == 1
    aoaSet = aoaLOF;
else
    aoaSet = 0;
end

% vROT = The speed at which elevator authority starts. Where the
% moment about the rear gear is above zero. Need distance that CG is
% forward of rear gear, the aircraft weight, thrust, centerline height
% above ground, and dCL/dE elevator effectiveness or d_alpha/dE and
% dCL/d_alpha for the elevator. vRot = vLOF for tail draggers.
vTest   = linspace(0, vMax, nVals);
M_about_rear = zeros(1, nVals);
for i = 1 : nVals
    M_about_rear = F_HT*(tail_arm - d_gear_to_quarter_chord)...
                    - thrust * h_thrust - weight * d_gear_to_CG;
end

aoaTest = linspace(0, aoaMax, nVals);
hDot = zeros(1, nVals^2);
aoa  = zeros(1, nVals^2);
v    = zeros(1, nVals^2);
for i = 1 : nVals % aoa iterations
    for j = 1 : nVals % v iterations
        aoa(i*(j-1) + j) = aoaTest(i);
        CL = CL0 + CL_Alpha * aoaTest(i);
        CD = CD0 + k*CL^2;
        v(i*(j-1) + j) = vTest(j);
        thrust = ThrustCalc(vTest(j));
        hDot(i*(j-1) + j)   = (thrust - 0.5*rho*v^2 * CD * Sref)*v / weight;
        vGround = sqrt(vTest(j)^2 - hDot(i*(j-1) + j)^2);
    end
end

[max_climb_hDot, max_climb_idx] = max(hDot);    % best climb rate
max_climb_v = v(max_climb_idx);                 % best airspeed for climb
max_climb_aoa = aoa(max_climb_idx);             % best aoa for climb