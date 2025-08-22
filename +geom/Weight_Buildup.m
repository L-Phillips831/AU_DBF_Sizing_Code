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

%{
To Do:
- Add logic to differentiate control surface and rib adjustments for
differing %MAC control surfaces. Model after last year? Is this necessary?
- Add material densities
- Figure out rib spacing according to load/lf, skin support, and AR
- Figure out bulkhead spacing from a historical average
- Add adjustment values from CAD or measurements
- Brainstorm how wing/tail surfaces may need to have different function
inputs
- Brainstorm how to obtain tail loadings
- Continue building out gear and fuselage hardware mass estimations

Inputs:
- All Aero MACs
- All Aero spans
- propulsion mass

Undetermined:
num structure for each aero surface
load for each aero surface

%}

%% Densities
rho.CF = 0; % [g/m^3]
rho.Balsa = 0; % [g/m^3]
rho.Ply = 0; % [g/m^3]
rho.Monokote = 0; % A 2D Density [g/m^2]

%% Adjustments
% Description: Non-dimensional constants determined by historical and 
% experimental averages of real components. Should roughly apply to all
% components regardless of size. 
adj.Rib        = 0; % Ratio of rib area to MAC^2
adj.Monokote   = 1.1; % Ratio of Monokote area used to wetted area
adj.Leading    = 0.2; % Ratio of leading edge skin arc length to MAC
adj.Adhesion   = 0.6; % Ratio of adhesion skin arc to MAC
adj.Winglet    = 0.5; % Ratio of winglet area to MAC^2
adj.RibControl = 0; % Ratio of control surface rib area to MAC^2, assuming 25% chord
adj.Panel      = 0; % Ratio of access panel PLA mass to access panel area

adj.EmpConnectBoom

%% Spacings
% Define the general spacing distances between ribs, bulkheads, etc
spacing.Taper = 0;

%% Aero Component Mass Calculations
loadWing     = 0.5 * lfMTOW * weight;
loadHorzTail = 0;
loadVertTail = 0;

massWing  = CalcWing (MAC_Wing, spanWing, adj, rho, num, loadWing);
massHTail = CalcWing (MAC_HorzTail, span, adj, rho, num, loadHorzTail);
massVTail = CalcWing (MAC_VertTail, span, adj, rho, num, loadVertTail);

%% Fuselage
% To include the fuselage section, connection to empennage (boom or
% extended bulkheads), and landing gear

%{
Inputs:
- lengthFullBox = fuselage length until taper
- aircraft length
- fuselage width, assuming box
- boolBoom = set to true if we are making a boom connection to empennage
- widthFus = fuselage width, accounting for internal components and
structural margin

Undetermined:
- widthFusEnd
- taperAngle = angle at which the fuselage tapers. Could be high of 20 deg.
%}

% Full width section of fuselage mass estimation
massFullBoxBulk = lengthFullBox * spacingBulk * widthFus^2 * 0.003175 * rho.Ply;

% Taper section of fuselage mass estimation
lengthTaper = widthFus / tand(taperAngle);  % Length based on width and angle
numBulkTaper = spacing.Taper * lengthTaper; % spacing defines #bulkheads
widthDif = widthFus - widthFusEnd;          % the width difference the taper needs to cover
widthIncrement = widthDif / numBulkTaper;   % increment of width covered by each bulkhead
massTaperBulk = ((widthFusEnd + 0.0127)^2 - widthFusEnd^2) * 0.003175 * rho.Ply; % First taper bulkhead mass, 0.5" larger than the boom/lightening hole width
if numBulkTaper > 1
    for i = 2 : numBulkTaper
        largerWidth = widthFusEnd + (i - 1) * widthIncrement;
        massTaperBulk = massTaperBulk + (largerWidth^2 - widthFusEnd^2) * 0.003175 * rho.Ply; % What about for an extended taper where lightening is not the spar diameter?
    end
end

% massFus = straight bulkhead method until taper then taper bulkheads
massBulks = massFullBoxBulk + massTaperBulk;
massFloors = 0; 
massPlates = 0; % 4 for wing structure, 5 small for motor
massLongerons = 0; % Assume 4 corners, 2 keel beams, 4 sides
massPanels = widthFus * lengthFullBox * adj.Panel;
massFus = massBulks + massFloors + massPlates + massLongerons;

% massEmpConnectBoom = massBoom + massHardware
% massEmpConnectExt  = bulkhead taper method + hardware
% massGear = 3*(wheel+axle+strut/structure+reinforcement)
massFus = massFus + massEmpConnect + massGear;

%% Propulsion

massProp = inputs;

%% Summations

massEmpty = massWing + massHTail + massVTail + massFus + massProp;

%% Functions

function massWing = CalcWing (MAC, span, adj, rho, num, load)
    % Inputs:
    % - MAC
    % - num
    % - span
    % - loadFactor (atMTOW)
    % - MTOW
    % - nacelleMass
    % - wingletsBool

    % Function Finds for Wing:
    % - spaceIn / spaceOut
    fsCF = 5; % Wing spar stress factor of safety
    sigmaCF = 0; % N/m^2 normal stress, from Rockwest Composites Site
    thickness = 0;

    mSpec    = @(y, b) 4/(pi*b) * sqrt(1 - (2*y/b)^2);
    mWrapper = @(y) mSpec(y, span);
    I        = integral(mWrapper, 0, b/2);
    moment   = I*load*fsCF;
    roWrap   = @(rO) CalcRoRemainder (moment, sigmaCF, rO, thickness);
    rOuter   = bisection(roWrap, 1e-6, 0.1, 1e-6, 1000);
    xcSpar   = pi*(rOuter^2 - (rOuter - thicknessSpar)^2);

    % Components, discounting quantity and variants/configs:

    % Ribs
    rib         = MAC^2 * adjRib; % Independent of density and quantity
    adjPlyRib   = numPlyRib * rho.Ply;
    adjBalsaRib = numBalsaRib * rho.Balsa;
    ribs        = rib*(adjPlyRib + adjBalsaRib);

    % Wing Spar
    spar = span * xcSpar * rho.CF;

    % Stringers
    stringer  = 0.003175^2 * span * rho.Balsa; % Independent of quantity
    stringers = num.Stringers * stringer;

    % Monokote
    monokote = (2.1 * span*MAC) * adj.Monokote * rho.Monokote; % Consider better approximation of wetted area

    % LE Skin
    skinLeading = span * MAC * adj.Leading;

    % Adhesion Skin
    skinAdhesion = 4 * MAC * adj.Adhesion * rho.Balsa * 0.003175/4 * (spaceIn + spaceOut);

    % Winglets
    winglets = 2 * wingletsBool * MAC^2 * adj.Winglet * rho.Ply;
    
    % Control Surface Ribs
    ribControl      = MAC^2 * adj.RibControl; % Independent of density and quantity
    adjControlHorn  = num.RibControl * rho.Ply;
    adjBalsaControl = num.RibHorn * rho.Balsa;
    ribsControl     = ribControl*(adjControlHorn + adjBalsaControl);

    % Control Surface Spar
    sparControl = MAC * span * adj.SparControl * 0.003175 * rho.Balsa;

    % Control Surface Skin
    skinControl = span * MAC * adj.LeadingControl;

    % Control Surface Stringers
    stringerControl  = span * 0.003175 * 0.0127; % 
    stringersControl = num.StringersControl * stringerControl;

    massWing     = ribs + spar + stringers + monokote + nacelleMass + skinLeading + skinAdhesion + winglets;
    massControls = ribsControl + sparControl + skinControl + stringersControl;
    massWing     = massWing + massControls;
end


function remainder = CalcRoRemainder (moment, stress, rO, thickness)
    % Calculates the difference between a manipulated normal stress due to
    % bending formula.
    % moment    : The total moment at the root of an aero surface
    % stress    : The normal stress a carbon fiber tube can survive
    % rO        : The outer radius of the carbon fiber tubing
    % thickness : The thickness of the carbon fiber tubing

    remainder = stress*pi/(4*moment) - rO / (rO^4 - (rO-thickness)^4);
end


function root = bisection(f, a, b, tol, maxIter)
    % Bisection method for root finding
    % f        : function handle
    % a, b     : interval endpoints
    % tol      : tolerance for convergence
    % maxIter  : maximum iterations
    
    if f(a)*f(b) > 0
        error('f(a) and f(b) must have opposite signs');
    end
    
    for k = 1:maxIter
        c = (a+b)/2;        % midpoint
        fc = f(c);
        
        if abs(fc) < tol || (b-a)/2 < tol
            root = c;
            return;
        end
        
        if f(a)*fc < 0
            b = c;          % root is in [a,c]
        else
            a = c;          % root is in [c,b]
        end
    end
    
    root = (a+b)/2;
end
