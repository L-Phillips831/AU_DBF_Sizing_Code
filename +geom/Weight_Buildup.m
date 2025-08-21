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

% Need Logan's MATLAB expertise to format

%% Densities
rhoCF = 0; % [g/m^3]
rhoBalsa = 0; % [g/m^3]
rhoPly = 0; % [g/m^3]
rhoMonokote = 0; % A 2D Density [g/m^2]

%% Adjustments
% Description: Non-dimensional constants determined by historical and experimental averages
% of real components
adjRib = 0; % Ratio of rib area to MAC^2
adjMonokote = 0; % Ratio of Monokote area used to wetted area
adjLeading = 0; % Ratio of leading edge skin arc length to MAC
adjAdhesion = 0; % Ratio of adhesion skin arc to MAC
adjWinglet = 0; % Ratio of winglet area to MAC^2
adjRibControl = 0; % Ratio of control surface rib area to MAC^2

%% Wing
% Inputs:
% - MAC
% - numPlyRib
% - numBalsaRib
% - span
% - wettedArea
% - loadFactor (atMTOW)
% - MTOW
% - nacelleMass
% - wingletsBool

% Function Finds for Wing:
% - spaceIn / spaceOut
fsCF = 5; % Wing spar stress factor of safety
sigmaCF = 0; % N/m^2 normal stress, from Rockwest Composites Site

mSpec = @(y) 4*y/(pi*b) * sqrt(1 - (2*y/b)^2);
moment = I*weight*lfMTOW;
rOuter = 0; % Implement bisection solver here
xcSpar = pi*(rOuter^2 - (rOuter - thicknessSpar)^2);

% Components, discounting quantity and variants/configs:

% Ribs = MAC^2 * adjustment of area * density
rib = MAC^2 * adjRib; % Independent of density and quantity
adjPlyRib = numPlyRib * rhoPly;
adjBalsaRib = numBalsaRib * rhoBalsa;
ribs = rib*(adjPlyRib + adjBalsaRib);

% Wing Spar = span*f(total estimated normal stress * FS)
spar = span * crossSectionSparWing * rhoCF;

% Stringers
stringer = 0.003175^2 * span * rhoBalsa; % Independent of quantity
stringers = numStringers * stringer;

% Monokote
monokote = wettedArea * adjMonokote * rhoMonokote;

% LE Skin
skinLeading = span * MAC * adjLeading;

% Adhesion Skin
skinAdhesion = 4 * MAC * adjAdhesion * rhoBalsa * 0.003175/4 * (spaceIn + spaceOut);

% Winglets
winglets = wingletsBool * MAC^2 * adjWinglet * rhoPly;

% Control Surface Ribs
ribControl = MAC^2 * adjRibControl; % Independent of density and quantity
adjControlHorn = numPlyRib * rhoPly;
adjBalsaControl = numBalsaRib * rhoBalsa;
ribs = rib*(adjControlHorn + adjBalsaControl);

% Control Surface Spar
sparControl

% Control Surface Skin


% Control Surface Stringers


wingMass = ribs + spar + stringers + monokote + nacelleMass + skinLeading + skinAdhesion + winglets;
wingControlsMass = ribsControl + sparControl + skinControl + stringersControl;
wingMass = wingMass + wingControlsMass;

%% Tail

%% Fuselage
% To include the fuselage section, connection to empennage (boom or
% extended bulkheads), and landing gear

%% Powerplant

%% Summations
% Total Mass = Wing + HTail + VTail + Fuselage + Powerplant + Mission
% Payload

function remainder = CalcRoRemainder (moment, stress, rO, thickness)
    remainder = stress*pi/(4*moment) - rO / (rO^4 - (rO-thickness)^4);
end