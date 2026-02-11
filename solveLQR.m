 % x = [R P Y, Rdot, Pdot, Ydot]^T
% u = [delta1, delta2, delta3, delta4]^T where delta is the canard deflection
% angle
% d = [alpha, beta]^T for disturbances of angle of attack and sideslip angle

% note this solution of A and B are based on the linearization that cos
% delta is approx 1. This is for a max deflection of 10 degrees
addpath(genpath('Models'));
kins = HPRC_VoyagerKinematics();
Jr = kins.I_x_empty;
Jl = kins.I_y_empty; % Iy and Iz are symmetric
A_ref = kins.S; % body tubes cross sectional area 
dbody = kins.diameter;
span_fin = kins.canard.span * 2; %still needed, placeholder %%FIX
span_canard = kins.canard.span;
Afin = kins.canard.Area * 2; % placeholder %%FIX
Acanard = kins.canard.Area;
radius_canard = kins.canard.span + kins.diameter / 2; %body radius at canard position
radius_fin = radius_canard * 2; %body radius at fin position %%FIX
Gamma_c_canard = kins.canard.gamma_c; %midchord sweep angle, make sure this is in radians, placeholder
Gamma_c_fin = deg2rad(45); %placeholder, fin midchord sweep angle %%FIX
Xcp_canards = kins.canard.x_cp;
Xcp_nose = kins.x_cp / 4; %approx location of nose cp %%FIX
Xcp_body = kins.x_cp; 
Xcp_fins = kins.len * 7/8; %approx location of fins cp %%FIX
X_cp = [Xcp_nose, Xcp_body, Xcp_fins, Xcp_canards]; % moment arm between the CP and CG, X_cp = X_cp - X_cg for each component of the rocket
cnalpha_nose = 2; % normal force coeff derivative of the nose section (2/A_ref * (A_ref - 0)) == 2
cnalpha_body = 0; % normal force coeff derivative of the body section (2/A_ref * (A_ref - A_ref)) == 0
aspect_ratio_canard = span_canard^2 / Acanard;
CLa = 2 * pi * aspect_ratio_canard / (2 + sqrt(4 + aspect_ratio_canard^2)); % lift curve slope of canards

cr_canard = kins.canard.rootChord;
ct_canard = kins.canard.tipChord;
cr_fin = cr_canard; %%FIX
ct_fin = ct_canard; %%FIX
hells_constant_canard = cr_canard/12 + ct_canard/4; %hells constnat, needs recalculating of the integral when final canard shape is known
hells_constant_fin = hells_constant_canard; %%FIX
%“How much roll damping comes from the fact that different parts of the fin move at different tangential speeds when the rocket spins

N = 4; %num fins

gainSched_K = containers.Map('KeyType', 'char', 'ValueType', 'any');
gainSched_AB = containers.Map('KeyType', 'char', 'ValueType', 'any');
vels = 1:2.5:100; % 1 to 100 m/s
heights = 40:5:500; % 40m to 500m elevation
for vel = vels
    for h = heights

        mach = mach_from_velocity(vel, h);
        cnalpha_0 = 2 * pi / sqrt(1 - mach^2);
        [T, a, P, rho, nu] = atmosisa(h); % Temp, speed of sound, pressure, density, kinematic viscosity, and dynamic viscosity
        q = 0.5*rho*vel^2; 

        cnalpha_components = [cnalpha_nose, cnalpha_body, calc_cnalpha_fins(mach, Afin, span_fin, radius_fin, Gamma_c_fin, A_ref), calc_cnalpha_canards(mach, Acanard, span_canard, radius_canard, Gamma_c_canard, A_ref)]; 

        %Cdp = ;% Roll damping moment coeff derivative w.r.t roll rate
        Cdp_canard = q / vel * A_ref * dbody * N * cnalpha_0 * hells_constant_canard;
        Cdp_fin = q / vel * A_ref * dbody * N * cnalpha_0 * hells_constant_fin;
        Cdp_total = Cdp_fin + Cdp_canard;
        cnalpha = calc_cnalpha_total(mach, Afin, span_fin, radius_fin, Gamma_c_fin, Acanard, span_canard, radius_canard, Gamma_c_canard, A_ref, cnalpha_nose, cnalpha_body);
        C1 = q * A_ref * cnalpha * Xcp_body; 
        La = q * A_ref * CLa;
        Ma = q*A_ref*CLa*Xcp_canards; % pitch moment derivative wrt aoa canards 
        Na = q*A_ref*CLa*Xcp_canards; % yaw moment derivative wrt aoa canards 
        C2 = q/vel * A_ref * sum((cnalpha_components + X_cp).^2); 
        
        
        A = [0, 0, 0, 1,        0,      0;
             0, 0, 0, 0,        1,      0;
             0, 0, 0, 0,        0,      1;
             0, 0, 0, -Cdp_total/Jr,  0,      0;
             0, 0, 0, 0,        -C2/Jl, 0;
             0, 0, 0, 0,        0,      -C2/Jl];
        
        B = [0,      0,      0,      0;      
             0,      0,      0,      0;      
             0,      0,      0,      0;      
             -La/Jr, -La/Jr, -La/Jr, -La/Jr; 
             -Ma/Jl, Ma/Jl,  0,      0;      
             0,      0,      -Na/Jl, Na/Jl];

        
        Q = diag([1e-9, 4, 4, 4, 1, 1]);
        R = diag([5, 5, 5, 5]);
        [K,~,~] = lqr(A,B,Q,R);
        key = sprintf('%.1f_%.1f', vel, h);
        gainSched_K(key) = K;
        gainSched_AB(key) = struct("A", A, "B", B);  % For stability check
    end
end

% After computing all gains, save to CSV
filename = 'lqr_gains.csv';

% Create header
header = {'velocity', 'height', 'K11', 'K12', 'K13', 'K14', 'K15', 'K16', ...
          'K21', 'K22', 'K23', 'K24', 'K25', 'K26', ...
          'K31', 'K32', 'K33', 'K34', 'K35', 'K36', ...
          'K41', 'K42', 'K43', 'K44', 'K45', 'K46'};

% Open file for writing
fid = fopen(filename, 'w');
fprintf(fid, '%s,', header{1:end-1});
fprintf(fid, '%s\n', header{end});

% Write data
for vel = vels
    for h = heights
        key = sprintf('%.1f_%.1f', vel, h);
        K = gainSched_K(key);
        
        % Write velocity and height
        fprintf(fid, '%.1f,%.1f,', vel, h);
        
        % Write K matrix elements (row-major order)
        K_row = reshape(K', 1, []);  % Transpose then flatten
        fprintf(fid, '%.6e,', K_row(1:end-1));
        fprintf(fid, '%.6e\n', K_row(end));
    end
end

fclose(fid);
fprintf('Gains saved to %s\n', filename);


% -------- Check closed-loop stability --------
maxReal = zeros(numel(vels), numel(heights));

for i=1:numel(vels)
    for j=1:numel(heights)
        key = sprintf('%.1f_%.1f', vels(i), heights(j));
        K = gainSched_K(key);
        AB = gainSched_AB(key);
        Acl = AB.A - AB.B*K;
        maxReal(i,j) = max(real(eig(Acl)));
    end
end

imagesc(heights, vels, maxReal); colorbar;
title('max real(eig(A-BK)) (should be < 0)');
xlabel('Height [m]');
ylabel('Velocity [m/s]');
% -------- Helper Functions --------

function M = mach_from_velocity(v, h)
    % v = velocity [m/s]
    % h = altitude [m]

    gamma = 1.4;
    R = 287.05;

    % ISA troposphere model (up to 11 km)
    T0 = 288.15;      % sea level temp [K]
    L = -0.0065;      % lapse rate [K/m]

    if h < 11000
        T = T0 + L*h;
    else
        % isothermal above 11 km (simplified)
        T = 216.65;
    end

    a = sqrt(gamma * R * T);   % speed of sound
    M = v / a;
end

function cnalpha_fins = calc_cnalpha_fins(mach, Afin, span_fin, radius_fin, Gamma_c_fin, A_ref)
    cnalpha_1 = calc_cnalpha_1(mach, Afin, span_fin, Gamma_c_fin, A_ref);
    cnalpha_fins = 2 * cnalpha_1 * (1 + radius_fin / (span_fin + radius_fin));

end

function cnalpha_canards = calc_cnalpha_canards(mach, Acanard, span_canard, radius_canard, Gamma_c_canard, A_ref)
    cnalpha_1 = calc_cnalpha_1(mach, Acanard, span_canard, Gamma_c_canard, A_ref);
    cnalpha_canards = 2 * cnalpha_1 * (1 + radius_canard / (span_canard + radius_canard));
end

function cnalpha_1 = calc_cnalpha_1(mach, A_currFin, span_currfin, Gamma_c_currfin, A_ref)
    numerator = 2 * pi * span_currfin^2 / A_ref;
    denom = 1 + sqrt(1 + ((span_currfin^2 * sqrt(1-mach^2) / (A_currFin * cos(Gamma_c_currfin))))^2);
    cnalpha_1 = numerator / denom;
end

function cnalpha = calc_cnalpha_total(mach, Afin, span_fin, radius_fin, Gamma_c_fin, Acanard, span_canard, radius_canard, Gamma_c_canard, A_ref, cnalpha_nose, cnalpha_body)
    cnalpha_fins = calc_cnalpha_fins(mach, Afin, span_fin, radius_fin, Gamma_c_fin, A_ref);
    cnalpha_canards = calc_cnalpha_canards(mach, Acanard, span_canard, radius_canard, Gamma_c_canard, A_ref);
    cnalpha = cnalpha_nose + cnalpha_body + cnalpha_fins + cnalpha_canards;
end








