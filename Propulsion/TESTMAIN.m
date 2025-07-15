clear
close all
clc

prop = propeller(7,11*0.0254,"PER3_11x8.dat");

bt = battery(10000,6,100,"LiPo");

mt = motor(470,130,0.0129);

esc = esc(60,80);

propulsion = Propulsion(bt,esc,mt,prop);

%Mark for Test Branch

n = 100;

tbl = table;
tbl.V = linspace(0,100,n)';
tbl.alt = ones(n,1).*0;
[tbl.T,tbl.a,tbl.P,tbl.rho] = atmosisa(tbl.alt);
tbl.q = 0.5.*tbl.rho.*tbl.V.^2;
tbl.Throttle = ones(n,1).*1;
tbl.RPM =  ones(n,1).*8000;%propulsion.calcRPM(propulsion,tbl.Throttle);
tbl.J = tbl.V ./ tbl.RPM .* 60 ./ propulsion.propeller.diameter;
tbl.Thrust = propulsion.propeller.GI_Thrust_fcn_J_RPM(tbl.J,tbl.RPM);
tbl.Power = propulsion.propeller.GI_Power_fcn_J_RPM(tbl.J,tbl.RPM);
tbl.dt = ones(n,1).*1;
tbl.dE = tbl.Power .* tbl.dt;


figure
hold on
yyaxis left
plot(tbl.V,tbl.Thrust)
ylim([0,ceil((max(tbl.Thrust)+10)/10)*10])

% yyaxis right
% plot(tbl.V,tbl.Power)
% ylim([0,(ceil(max(tbl.Power)+100)/1000)*1000])

% take off - (4 mini segments)
% climb 
% cruise / acceleration @ constant power
% turn 
% descent

% Drag Polar (Cd0, k)
% Cl Max

% geom wing fuselage/pod/ LG -> Cd0 Weight

% Cd0 -> D -> sum(Cd0_a) = CD0_a 
