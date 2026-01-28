 % x = [R P Y, Rdot, Pdot, Ydot]^T
% u = [delta1, delta2, delta3, delta4]^T where delta is the canard deflection
% angle
% d = [alpha, beta]^T for disturbances of angle of attack and sideslip angle

% note this solution of A and B are based on the linearization that cos
% delta is approx 1. This is for a max deflection of 10 degrees

kins = HPMR_MissileKinematics();
Jr = kins.I_x;
Jl = kins.I_y; % Iy and Iz are symmetric
A_ref = kins.S; % body tubes cross sectional area 
dbody = kins.diameter;
span_fin = kins.canard.height * 2; %still needed, placeholder
span_canard = kins.canard.height; %still needed
Afin = kins.canard.S * 2; % placeholder
Acanard = kins.canard.S;
radius_canard = kins.canard.height + kins.diameter / 2; %body radius at canard position
radius_fin = radius_canard * 2; %body radius at fin position
Gamma_c_canard = deg2rad(30); %midchord sweep angle, make sure this is in radians, placeholder
Gamma_c_fin = deg2rad(45); %placeholder, fin midchord sweep angle
Xcp_canards = kins.canard.x_cp;
Xcp_nose = kins.x_cp / 4; %approx location of nose cp
Xcp_body = kins.x_cp;
Xcp_fins = kins.len * 7/8; %approx location of fins cp
X_cp = [Xcp_nose, Xcp_body, Xcp_fins, Xcp_canards]; % moment arm between the CP and CG, X_cp = X_cp - X_cg for each component of the rocket
cnalpha_nose = 2; % normal force coeff derivative of the nose section (2/A_ref * (A_ref - 0)) == 2
cnalpha_body = 0; % normal force coeff derivative of the body section (2/A_ref * (A_ref - A_ref)) == 0
aspect_ratio_canard = span_canard^2 / Acanard;
CLa = 2 * pi * aspect_ratio_canard / (2 + sqrt(4 + aspect_ratio_canard^2)); % lift curve slope of canards

cr = kins.canard.rootChord;
ct = kins.canard.tipChord;
hells_constant = cr/12 + ct/4; %hells constnat, needs recalculating of the integral when final canard shape is known
%“How much roll damping comes from the fact that different parts of the fin move at different tangential speeds when the rocket spins

N = 4; %num fins

gainSched = dictionary();
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
        Cdp = q / vel * A_ref * dbody * N * cnalpha_0 * hells_constant;
        cnalpha = calc_cnalpha_total(mach, Afin, span_fin, radius_fin, Gamma_c_fin, Acanard, span_canard, radius_canard, Gamma_c_canard, A_ref, cnalpha_nose, cnalpha_body);
        C1 = q * A_ref * cnalpha * Xcp_body; 
        La = q * A_ref * Cla;
        Ma = q*A_ref*CLa*Xcp_canards; % pitch moment derivative wrt aoa canards 
        Na = q*A_ref*CLa*Xcp_canards; % yaw moment derivative wrt aoa canards 
        C2 = q/vel * A_ref * sum((cnalpha_components + X_cp).^2); 
        
        
        A = [0, 0, 0, 1,        0,      0;
             0, 0, 0, 0,        1,      0;
             0, 0, 0, 0,        0,      1;
             0, 0, 0, -Cdp/Jr,  0,      0;
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
        %gainSched(key) = struct("K",K,"A",A,"B",B);
        gainSched(key) = K;
    end
end

% -------- Check closed-loop stability --------
% maxReal = zeros(numel(vels), numel(heights));
%
%for i=1:numel(vels)
%  for j=1:numel(heights)
%    key = sprintf('%.1f_%.1f', vels(i), heights(j));
%    s = gainSched(key);
%    Acl = s.A - s.B*s.K; % use the A,B for that (v,h)
%    maxReal(i,j) = max(real(eig(Acl)));
%  end
%end
%
%imagesc(heights, vels, maxReal); colorbar;
%title('max real(eig(A-BK)) (should be < 0)');
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








