% x = [R P Y, Rdot, Pdot, Ydot]^T
% u = [delta1, delta2, delta3, delta4]^T where delta is the canard deflection
% angle
% d = [alpha, beta]^T for disturbances of angle of attack and sideslip angle

% note this solution of A and B are based on the linearization that cos
% delta is approx 1. This is for a max deflection of 10 degrees

kins = HPMR_MissileKinematics();
Jr = kins.I_x;
Jl = kins.I_y; % Iy and Iz are symmetric
Aref = kins.S;
dbody = kins.diameter;
N = 4; %num fins

gainSched = dictionary;
vels = 1:2.5:100; % 1 to 100 m/s
heights = 40:5:500; % 40m to 500m elevation
for vel = vels
    for h = heights

        mach = mach_from_velocity(vel, h);
        %Cdp = ;% Roll damping moment coeff derivative w.r.t roll rate
        Cdp = 1;
        C2 = 1;
        La = 1;
        Ma = 1;
        Na = 1;
        C1 = 1;
        
        
        A = [0 0 0 1       0        0;...
             0 0 0 0       1        0;...
             0 0 0 0       0        1;...
             0 0 0 -Cdp/Jr 0        0;...
             0 0 0 0       -C2/Jl   0;...
             0 0 0 0       0        -C2/Jl];
        
        B = [0      0       0       0;...
             0      0       0       0;...
             0      0       0       0;...
             -La/Jr -La/Jr  -La/Jr  -La/Jr;...
             -Ma/Jl Ma/Jl   0       0;...
             0      0       -Na/Jl  Na/Jl];
        
        E = [0              0;...
             0              0;...
             0              0;...
             0              0;...
             2*Ma/Jl-C1/Jl  0;...
             0              2*Na/Jl-C1/Jl];
    end
end


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

