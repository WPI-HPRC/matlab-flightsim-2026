function CanardSizer(varargin)
% CANARDSIZER - Canard sizing for Stability (2.25 cal) & Torque limits
%   CanardSizer('NACA','0012','Velocity',83.7,'MaxTorque',2.25,'TargetSM',2.25)
%
%   BOLT MOUNTING OPTIONS (all in meters):
%   'BoltDiameter'  - hex bolt across-flats diameter (default: 0.25in = 6.35mm)
%   'BoltLength'    - bolt shank length (default: 0.50in = 12.70mm)
%   'RootExtension' - straight (un-swept) root tab depth in span direction
%                     (default: 0.25in = 6.35mm). This tab precedes the 60 deg
%                     LE sweep and provides a rectangular pocket for the bolt.

    %% 1. INPUTS
    p = inputParser;
    addParameter(p, 'NACA', '0016', @ischar);
    addParameter(p, 'Velocity', 130, @isnumeric);
    addParameter(p, 'Altitude', 0, @isnumeric);
    addParameter(p, 'MaxTorque', 2.25, @isnumeric); 
    addParameter(p, 'MaxAOA', 10, @isnumeric);
    addParameter(p, 'TargetSM', 2.5, @isnumeric);  
    addParameter(p, 'CanardLE', 0.508, @isnumeric);  
    addParameter(p, 'SweepLE', 60, @isnumeric);
    addParameter(p, 'TaperRatio', 0.5, @isnumeric);
    addParameter(p, 'BoltDiameter',  0.25 * 0.0254, @isnumeric);  % 1/4" hex bolt [m]
    addParameter(p, 'BoltLength',    0.50 * 0.0254, @isnumeric);  % 1/2" shank [m]
    addParameter(p, 'RootExtension', 0.25 * 0.0254, @isnumeric);  % 1/4" straight root tab [m]
    parse(p, varargin{:});
    
    NACA_str = p.Results.NACA;
    V = p.Results.Velocity;
    M_limit = p.Results.MaxTorque;
    alpha_deg = p.Results.MaxAOA;
    SM_target = p.Results.TargetSM;
    X_can_LE = p.Results.CanardLE;
    Sweep_LE_deg = p.Results.SweepLE;
    lambda = p.Results.TaperRatio;
    d_bolt   = p.Results.BoltDiameter;
    L_bolt   = p.Results.BoltLength;
    ext_root = p.Results.RootExtension;
    
    % --- STABILITY DATA LOADING ---
    opts = detectImportOptions('Voyager(Non-Canards)V2.csv');
    opts.VariableNames = {'Time','Alt','V_vert','V_total','AOA','Mass','CP','CG','SM','CN','D_ref','A_ref'};
    f_data = readtable('Voyager(Non-Canards)V2.csv', opts);
    f_data = f_data(~isnan(f_data.SM), :); 
    
    D = f_data.D_ref(1) / 100;             % Reference diameter [m] (from CSV, cm→m)
    S_ref = f_data.A_ref(1) / 1e4;         % Reference area [m²] (from CSV, cm²→m²)
    N_canards = 4;                          % Number of canards
    % N_eff: calibrated effective fin count for stability
    % Higher=More stable
    N_eff = 10;
    
    [~, idx] = min(abs(f_data.V_total - V));
    X_cg = f_data.CG(idx) / 100;           % CG position [m]
    X_cp_base = f_data.CP(idx) / 100;       % Base CP [m] (no canards)
    
    % CN_alpha: use points with AOA > 2° for reliable slope estimate
    valid = f_data.AOA > 2;
    if any(valid)
        CN_a_base = mean(f_data.CN(valid) ./ deg2rad(f_data.AOA(valid)));
    else
        CN_a_base = f_data.CN(idx) / deg2rad(max(0.01, f_data.AOA(idx)));
    end
    
    t_c = parse_naca_thickness(NACA_str);
    [rho, a, mu] = isa_atmosphere(f_data.Alt(idx));
    Mach = V / a;
    q = 0.5 * rho * V^2;
    alpha_rad = deg2rad(alpha_deg);
    
    fprintf('\n========================================\n');
    fprintf('   CANARD STABILITY OPTIMIZATION\n');
    fprintf('========================================\n');
    fprintf('NACA: %s | V=%.1fm/s | Target SM=%.2f | Max Torque=%.2fNm\n', ...
        NACA_str, V, SM_target, M_limit);
    fprintf('\n--- BASE ROCKET (from CSV at V=%.1fm/s) ---\n', V);
    fprintf('Body Diameter (D):    %6.2f mm\n', D * 1000);
    fprintf('Reference Area:       %6.2f cm²\n', S_ref * 1e4);
    fprintf('CG Position:          %6.2f mm\n', X_cg * 1000);
    fprintf('CP Position (base):   %6.2f mm\n', X_cp_base * 1000);
    fprintf('Base SM:              %6.2f cal\n', (X_cp_base - X_cg) / D);
    fprintf('CN_alpha (base):      %6.3f /rad\n', CN_a_base);
    fprintf('Num Canards:          %d\n', N_canards);
    
    %% 2. SWEEP ASPECT RATIO
    AR_range = linspace(1.0, 4.0, 100);
    data_AR = []; data_Force = []; data_CruiseDrag = []; data_Span = [];
    data_CL = []; data_CD_cruise = []; data_CD_max = []; data_e = [];
    
    for AR = AR_range
        e_oswald = calculate_oswald_raymer(AR, Sweep_LE_deg);
        sm_fun = @(s) calc_stability_error(s, AR, Sweep_LE_deg, lambda, Mach, ...
                                          SM_target, X_can_LE, X_cg, X_cp_base, CN_a_base, D, S_ref, N_eff);
        try
            [s_sol, ~, exitflag] = fzero(sm_fun, [0.001, 0.5]);
        catch
            exitflag = -1;
        end
        
        if exitflag > 0 && s_sol > 0
            [~, geom, aero] = solve_physics(s_sol, AR, Sweep_LE_deg, lambda, q, Mach, alpha_rad, M_limit, e_oswald, t_c, rho, V, mu);
            if aero.M_total <= M_limit
                [CruiseDrag, CD_cruise] = calculate_cruise_drag(geom, AR, t_c, Sweep_LE_deg, rho, V, mu, Mach);
                [~, CD_max] = calculate_full_drag(geom, aero, AR, t_c, Sweep_LE_deg, rho, V, mu, Mach, e_oswald);
                data_AR = [data_AR, AR]; data_Force = [data_Force, aero.Force];
                data_CruiseDrag = [data_CruiseDrag, CruiseDrag]; data_Span = [data_Span, s_sol];
                data_CL = [data_CL, aero.C_L]; data_CD_cruise = [data_CD_cruise, CD_cruise];
                data_CD_max = [data_CD_max, CD_max]; data_e = [data_e, e_oswald];
            end
        end
    end
    
    %% 3. SKEWED CURVE INTERSECTION
    if isempty(data_AR), error('No designs satisfy SM and Torque limits.'); end
    skew_factor = 1;
    F_norm = (data_Force - min(data_Force)) / (max(data_Force) - min(data_Force) + 1e-6);
    D_norm = (data_CruiseDrag - min(data_CruiseDrag)) / (max(data_CruiseDrag) - min(data_CruiseDrag) + 1e-6);
    ar_fine = linspace(min(data_AR), max(data_AR), 1000);
    F_smooth = interp1(data_AR, F_norm, ar_fine, 'spline');
    D_smooth = interp1(data_AR, D_norm, ar_fine, 'spline');
    D_skewed = D_smooth * skew_factor;
    [~, idx_cross] = min(abs(F_smooth - D_skewed));
    best_AR = ar_fine(idx_cross);
    best_s = interp1(data_AR, data_Span, best_AR);
    best_e = calculate_oswald_raymer(best_AR, Sweep_LE_deg);
    [~, final_geom, final_aero] = solve_physics(best_s, best_AR, Sweep_LE_deg, lambda, q, Mach, alpha_rad, M_limit, best_e, t_c, rho, V, mu);
    [best_CruiseDrag, best_CD_cruise] = calculate_cruise_drag(final_geom, best_AR, t_c, Sweep_LE_deg, rho, V, mu, Mach);
    [best_MaxDrag, best_CD_max, CD0, CDi, CDv] = calculate_full_drag(final_geom, final_aero, best_AR, t_c, Sweep_LE_deg, rho, V, mu, Mach, best_e);
    best_CL = interp1(data_AR, data_CL, best_AR);
    Re_final = (rho * V * final_geom.MAC) / mu;
    
    %% 4. OUTPUT REPORT
    fprintf('\n========================================\n');
    fprintf('   OPTIMAL CANARD DIMENSIONS\n');
    fprintf('========================================\n');
    fprintf('\n--- GEOMETRY ---\n');
    fprintf('Optimal Aspect Ratio: %6.2f\n', best_AR);
    fprintf('Semi-Span (s):        %6.2f mm\n', best_s * 1000);
    fprintf('Full Span (b):        %6.2f mm\n', best_s * 2000);
    fprintf('Root Chord (Cr):      %6.2f mm\n', final_geom.Cr * 1000);
    fprintf('Tip Chord (Ct):       %6.2f mm\n', final_geom.Ct * 1000);
    fprintf('Planform Area:        %6.4f m^2\n', final_geom.A_fin);
    fprintf('MAC:                  %6.2f mm\n', final_geom.MAC * 1000);
    
    fprintf('\n--- AERODYNAMICS (Max AOA = %.1f deg) ---\n', alpha_deg);
    fprintf('Reynolds Number:      %6.2e\n', Re_final);
    fprintf('Lift Coefficient:     %6.4f\n', best_CL);
    
    fprintf('\n--- DRAG BREAKDOWN (Max AOA) ---\n');
    fprintf('Total Drag Coeff:     %6.5f\n', best_CD_max);
    fprintf('  Profile Drag:       %6.5f (%.1f%%)\n', CD0, 100*CD0/best_CD_max);
    fprintf('  Induced Drag:       %6.5f (%.1f%%)\n', CDi, 100*CDi/best_CD_max);
    fprintf('  Vortex Drag:        %6.5f (%.1f%%)\n', CDv, 100*CDv/best_CD_max);
    
    fprintf('\n--- FORCES & MOMENTS ---\n');
    fprintf('Max Normal Force:     %6.2f N\n', final_aero.Force);
    fprintf('TOTAL MOMENT:         %6.3f Nm (Limit: %.2f Nm)\n', final_aero.M_total, M_limit);
    
    Xt_straight = final_geom.Cr - final_geom.Ct;
    TE_Sweep_deg = rad2deg(atan((final_geom.Xt - Xt_straight) / best_s));
    fprintf('TE Sweep Angle:       %6.2f deg\n', TE_Sweep_deg);
    
    % -----------------------------------------------------------------------
    %  BOLT MOUNTING & ROOT EXTENSION GEOMETRY
    % -----------------------------------------------------------------------
    % Root chord max thickness (NACA symmetric, t/c at 30% chord):
    t_root_mm        = t_c * final_geom.Cr * 1000;   % [mm]
    d_bolt_mm        = d_bolt  * 1000;
    L_bolt_mm        = L_bolt  * 1000;
    ext_mm           = ext_root * 1000;
    phys_semi_span_mm = best_s * 1000 + ext_mm;
    phys_full_span_mm = phys_semi_span_mm * 2;
    tab_LE_setback_mm = ext_mm * tand(Sweep_LE_deg);  % LE offset at tab OB edge

    fprintf('\n--- BOLT MOUNTING GEOMETRY ---\n');
    fprintf('Bolt diameter:        %6.2f mm  (%.4f in)\n', d_bolt_mm, d_bolt/0.0254);
    fprintf('Bolt shank length:    %6.2f mm  (%.4f in)\n', L_bolt_mm, L_bolt/0.0254);
    fprintf('Root tab depth (span):%6.2f mm  (%.4f in)\n', ext_mm,    ext_root/0.0254);
    fprintf('Max airfoil thickness at root: %6.2f mm  (NACA %s x Cr)\n', t_root_mm, NACA_str);

    if t_root_mm >= d_bolt_mm
        fprintf('  >> Thickness CHECK PASS : %.2f mm >= bolt %.2f mm\n', t_root_mm, d_bolt_mm);
    else
        fprintf('  >> *** THICKNESS FAIL   : root %.2f mm thick, bolt needs %.2f mm ***\n', t_root_mm, d_bolt_mm);
        thk_needed = (d_bolt_mm / (final_geom.Cr * 1000)) * 100;
        fprintf('     Minimum NACA t/c needed: NACA 00%.0f  (t/c = %.1f%%)\n', ceil(thk_needed), thk_needed);
    end

    if final_geom.Cr * 1000 >= L_bolt_mm
        fprintf('  >> Chord   CHECK PASS : Cr %.2f mm >= bolt length %.2f mm\n', final_geom.Cr*1000, L_bolt_mm);
    else
        fprintf('  >> *** CHORD FAIL     : Cr %.2f mm shorter than bolt %.2f mm ***\n', final_geom.Cr*1000, L_bolt_mm);
    end

    fprintf('\n--- PHYSICAL DIMENSIONS (WITH ROOT TAB) ---\n');
    fprintf('Root tab depth:            %6.2f mm  (no LE sweep, straight section)\n', ext_mm);
    fprintf('LE setback at tab OB edge: %6.2f mm  (where 60 deg sweep begins)\n', tab_LE_setback_mm);
    fprintf('Aero semi-span:            %6.2f mm  (swept planform, unchanged)\n', best_s * 1000);
    fprintf('Physical semi-span:        %6.2f mm  (= tab + aero)\n', phys_semi_span_mm);
    fprintf('Physical full span:        %6.2f mm\n', phys_full_span_mm);
    fprintf('Root chord:                %6.2f mm  (unchanged)\n', final_geom.Cr * 1000);
    fprintf('Tip chord:                 %6.2f mm  (unchanged)\n', final_geom.Ct * 1000);
    fprintf('\n  Root-tab cross section schematic (looking inboard):\n');
    fprintf('  [<-- %.1f mm chord -->]  thickness %.1f mm\n', final_geom.Cr*1000, t_root_mm);
    fprintf('  Bolt pocket: %.1f mm dia x %.1f mm long (fits chord-wise)\n', d_bolt_mm, L_bolt_mm);
    fprintf('  Tab span width: %.1f mm  |  Straight before 60 deg LE sweep\n', ext_mm);
    
    % --- STABILITY VERIFICATION (re-compute all intermediate values) ---
    fprintf('\n--- STABILITY DEBUG ---\n');
    beta_v = sqrt(max(1-Mach^2, 0.01));
    Sweep_mid_v = atan(tan(deg2rad(Sweep_LE_deg)) - 2*(1-lambda)/(best_AR*(1+lambda)));
    CN_a_fin_v = (2*pi*best_s^2/S_ref) / (1 + sqrt(1 + (beta_v*best_s^2/(final_geom.A_fin*cos(Sweep_mid_v)))^2));
    R_body = D/2;
    K_fb_v = 1 + R_body/(R_body + best_s);
    CN_a_canard_v = N_eff * K_fb_v * CN_a_fin_v;
    y_MAC_v = (best_s/3)*(1+2*lambda)/(1+lambda);
    X_cp_can_v = X_can_LE + y_MAC_v*tan(deg2rad(Sweep_LE_deg)) + 0.25*final_geom.MAC;
    X_cp_total_v = (CN_a_base * X_cp_base + CN_a_canard_v * X_cp_can_v) / (CN_a_base + CN_a_canard_v);
    SM_computed = (X_cp_total_v - X_cg) / D;
    
    fprintf('CN_a per fin (Barr):  %6.4f /rad\n', CN_a_fin_v);
    fprintf('K (body-fin interf):  %6.3f\n', K_fb_v);
    fprintf('CN_a total canards:   %6.4f /rad\n', CN_a_canard_v);
    fprintf('CN_a base rocket:     %6.4f /rad\n', CN_a_base);
    fprintf('CN_a ratio (can/base):%6.4f\n', CN_a_canard_v/CN_a_base);
    fprintf('Canard CP:            %6.2f mm (from nose)\n', X_cp_can_v * 1000);
    fprintf('Base CP:              %6.2f mm (from nose)\n', X_cp_base * 1000);
    fprintf('Combined CP:          %6.2f mm (from nose)\n', X_cp_total_v * 1000);
    fprintf('CG:                   %6.2f mm (from nose)\n', X_cg * 1000);
    fprintf('Body Diameter:        %6.2f mm\n', D * 1000);
    fprintf('Computed SM:          %6.2f cal\n', SM_computed);
    fprintf('Target SM:            %6.2f cal\n', SM_target);
    fprintf('TE Sweep Angle:       %6.2f deg\n', TE_Sweep_deg);
    
    %% 5. PLOTS
    figure('Color','white', 'Position', [100, 100, 1200, 800]);
    subplot(2,2,1); yyaxis left; plot(data_AR, data_Force, 'b-', 'LineWidth', 2); ylabel('Force (N)');
    yyaxis right; plot(data_AR, data_CruiseDrag, 'r-', 'LineWidth', 2); ylabel('Drag (N)');
    xline(best_AR, '--k', sprintf('AR=%.2f', best_AR)); xlabel('AR'); grid on;
    
    subplot(2,2,2); plot(ar_fine, F_smooth, 'b', 'LineWidth', 2); hold on;
    plot(ar_fine, D_smooth, 'r--', 'LineWidth', 1); plot(ar_fine, D_skewed, 'r', 'LineWidth', 2);
    plot(best_AR, F_smooth(idx_cross), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g');
    legend('Lift', 'Drag', 'Drag×Skew', 'Optimal'); xlabel('AR'); ylabel('Normalized'); grid on;
    
    subplot(2,2,3); CD0_all=[]; CDi_all=[]; CDv_all=[];
    for i = 1:length(data_AR)
        e_i = calculate_oswald_raymer(data_AR(i), Sweep_LE_deg);
        [~, g, a] = solve_physics(data_Span(i), data_AR(i), Sweep_LE_deg, lambda, q, Mach, alpha_rad, M_limit, e_i, t_c, rho, V, mu);
        [~,~,cd0,cdi,cdv] = calculate_full_drag(g, a, data_AR(i), t_c, Sweep_LE_deg, rho, V, mu, Mach, e_i);
        CD0_all=[CD0_all,cd0]; CDi_all=[CDi_all,cdi]; CDv_all=[CDv_all,cdv];
    end
    area(data_AR, [CD0_all; CDi_all; CDv_all]'); xlabel('AR'); ylabel('CD'); grid on; legend('Profile','Induced','Vortex');
    
    subplot(2,2,4); plot(data_AR, data_CL./data_CD_max, 'm-', 'LineWidth', 2);
    xline(best_AR, '--k'); xlabel('AR'); ylabel('L/D'); grid on;
    sgtitle(sprintf('NACA %s | V=%.0fm/s | SM=%.2f', NACA_str, V, SM_target));
end

function err = calc_stability_error(s, AR, Sweep_deg, lambda, Mach, target_sm, X_can_LE, X_cg, X_cp_base, CN_a_base, D, S_ref, N_canards)
    A_fin = (2*s^2)/AR; Cr = (2*A_fin)/(s*(1+lambda)); Ct = Cr*lambda;
    MAC = (2/3)*Cr*((1+lambda+lambda^2)/(1+lambda));
    Sweep_rad = deg2rad(Sweep_deg);
    beta = sqrt(max(1-Mach^2, 0.01));
    
    % BARROWMAN FIN CN_alpha (matches OpenRocket)
    % CN_α = (2π·s²/S_ref) / (1 + √(1 + (β·s²/(A_fin·cos(Λ_mid)))²))
    Sweep_mid = atan(tan(Sweep_rad) - 2*(1-lambda)/(AR*(1+lambda)));
    CN_a_fin = (2*pi*s^2/S_ref) / (1 + sqrt(1 + (beta*s^2/(A_fin*cos(Sweep_mid)))^2));
    
    % Body-fin interference: K = 1 + R/(R+s) (Barrowman)
    R = D/2;
    K_fb = 1 + R/(R + s);
    
    % Total CN_alpha: use calibrated N_eff
    CN_a_canard = N_canards * K_fb * CN_a_fin;
    
    % Canard CP (Barrowman): X_LE + MAC_LE_offset + 25% MAC
    X_cp_canard = X_can_LE + (s/3)*((1+2*lambda)/(1+lambda))*tan(Sweep_rad) + 0.25*MAC;
    
    X_cp_total = (CN_a_base * X_cp_base + CN_a_canard * X_cp_canard) / (CN_a_base + CN_a_canard);
    err = ((X_cp_total - X_cg) / D) - target_sm;
end

function t_c = parse_naca_thickness(naca_str)
    if length(naca_str) >= 4, t_c = str2double(naca_str(end-1:end)) / 100;
    else, t_c = 0.12; end
end

function [rho, a, mu] = isa_atmosphere(h)
    T0=288.15; P0=101325; L=0.0065; R=287.05; gamma=1.4;
    if h < 11000, T = T0 - L*h; P = P0 * (T/T0)^(9.80665/(R*L));
    else, T = 216.65; P = P0 * (216.65/T0)^(9.80665/(R*L)) * exp(-9.80665*(h-11000)/(R*T)); end
    rho = P/(R*T); a = sqrt(gamma*R*T); mu = 1.716e-5 * (T/273.15)^1.5 * 383.55/(T+110.4);
end

function e = calculate_oswald_raymer(AR, LE_sweep_deg)
    e = 1.78*(1-0.045*AR^0.68)*(cos(deg2rad(LE_sweep_deg)))^0.15 - 0.64;
    e = max(0.5, min(0.95, e));
end

function [diff, geom, aero] = solve_physics(s, AR, Sweep_deg, lambda, q, Mach, alpha, M_target, e_oswald, t_c, rho, V, mu)
    A_fin = (2*s^2)/AR; Cr = (2*A_fin)/(s*(1+lambda)); Ct = Cr*lambda;
    MAC = (2/3)*Cr*((1+lambda+lambda^2)/(1+lambda)); Xt = s*tan(deg2rad(Sweep_deg));
    b = 2*s; Sweep_LE_rad = deg2rad(Sweep_deg);
    beta = sqrt(max(1-Mach^2, 0.01));
    Sweep_c2_rad = atan(tan(Sweep_LE_rad) - 2*(1-lambda)/(AR*(1+lambda)));
    Kp = (2*pi*AR) / (2 + sqrt(4 + (AR^2*beta^2/0.95^2)*(1 + tan(Sweep_c2_rad)^2/beta^2)));
    Kv_LE = (pi*AR)/(2*cos(Sweep_LE_rad)) * (1 + 0.3*sin(Sweep_LE_rad)^2);
    Kv_SE = (4*Ct)/(b*(1+lambda)) * cos(Sweep_LE_rad)^0.5 / (1 + 0.1*AR);
    S_tip = max(0, 0.5*Ct^2/tan(Sweep_LE_rad + 0.01)); Kv_aug = Kv_LE * (S_tip/A_fin) * 0.3;
    sin_a = sin(alpha); cos_a = cos(alpha);
    CN_pot = Kp * sin_a * cos_a; CN_vtx = (Kv_LE + Kv_SE + Kv_aug) * sin_a^2; C_N = CN_pot + CN_vtx;
    CL_pot = Kp * sin_a * cos_a^2; CL_vtx_LE = Kv_LE * sin_a^2 * cos_a; CL_vtx_SE = (Kv_SE + Kv_aug) * sin_a^2 * cos_a; C_L = CL_pot + CL_vtx_LE + CL_vtx_SE;
    y_MAC = (s/3)*(1+2*lambda)/(1+lambda); X_LE_MAC = y_MAC * tan(Sweep_LE_rad);
    if Mach < 0.8, x_ac = 0.25; elseif Mach > 1.2, x_ac = 0.50; else, x_ac = 0.25 + (Mach-0.8)/0.4*0.25; end
    X_cp_pot = X_LE_MAC + x_ac*MAC; X_cp_vtx = (Xt/3)*((Cr+2*Ct)/(Cr+Ct)) + (1/3)*((Cr^2+Ct^2+Cr*Ct)/(Cr+Ct));
    if C_N > 0, X_cp_comp = (CN_pot*X_cp_pot + CN_vtx*X_cp_vtx)/C_N; else, X_cp_comp = X_cp_pot; end
    X_hinge = 0.5 * X_cp_comp;
    F_pot = q * A_fin * CN_pot; F_vtx = q * A_fin * CN_vtx; F_total = F_pot + F_vtx;
    M_pot = F_pot * (X_cp_pot - X_hinge); M_vtx = F_vtx * (X_cp_vtx - X_hinge); M_calc = M_pot + M_vtx;
    diff = M_calc - M_target;
    geom = struct('A_fin',A_fin,'Cr',Cr,'Ct',Ct,'Xt',Xt,'MAC',MAC,'Sweep_rad',Sweep_LE_rad,'b',b);
    aero = struct('Force',F_total,'X_cp_potential',X_cp_pot,'X_cp_vortex',X_cp_vtx,'X_cp_composite',X_cp_comp,...
        'X_hinge',X_hinge,'M_potential',M_pot,'M_vortex',M_vtx,'M_total',M_calc,'C_L',C_L,'CL_potential',CL_pot,...
        'CL_vortex_LE',CL_vtx_LE,'CL_vortex_SE',CL_vtx_SE,'Kp',Kp,'Kv_LE',Kv_LE,'Kv_SE',Kv_SE+Kv_aug,...
        'C_N',C_N,'CN_potential',CN_pot,'CN_vortex',CN_vtx,'x_ac_factor',x_ac);
end

function [Drag, C_D] = calculate_cruise_drag(geom, AR, t_c, Sweep_deg, rho, V, mu, Mach)
    Re = (rho * V * geom.MAC) / mu; Re_crit = 5e5;
    if Re < Re_crit, Cf = 1.328 / sqrt(Re);
    else, Cf_turb = 0.455 / (log10(Re))^2.58; Cf_crit = 0.455 / (log10(Re_crit))^2.58; Cf_lam = 1.328 / sqrt(Re_crit); A = Re_crit * (Cf_crit - Cf_lam); Cf = Cf_turb - A/Re; end
    Cf = Cf / (1 + 0.144*Mach^2)^0.65;
    t_eff = t_c * cos(deg2rad(Sweep_deg)); FF = (1 + 2*t_eff + 60*t_eff^4) * cos(deg2rad(Sweep_deg))^0.28;
    C_D = Cf * FF * 2.05; Drag = 0.5 * rho * V^2 * geom.A_fin * C_D;
end

function [Drag, C_D, CD0, CDi, CDv] = calculate_full_drag(geom, aero, AR, t_c, Sweep_deg, rho, V, mu, Mach, e)
    [~, CD0] = calculate_cruise_drag(geom, AR, t_c, Sweep_deg, rho, V, mu, Mach);
    CDi = (aero.CL_potential^2) / (pi * AR * e);
    CL_vtx = aero.CL_vortex_LE + aero.CL_vortex_SE; Kv = aero.Kv_LE + aero.Kv_SE;
    if Kv > 0 && CL_vtx > 0, sin_sq = CL_vtx / (Kv * 0.985); CDv = Kv * sin_sq * sqrt(max(sin_sq,0)); else, CDv = 0; end
    C_D = CD0 + CDi + CDv; Drag = 0.5 * rho * V^2 * geom.A_fin * C_D;
end