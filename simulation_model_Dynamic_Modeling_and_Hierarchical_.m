%% HYBRID IRRIGATION SIMULATION - LA MESA (by Sergio Andres Castro Acuna | github: scastroac@unal.edu.co)
clear; clc; close all;

%% 1. PARAMETERS (Site-Specific & Agronomic limits from LaTeX)
A_roof  = 195;       % [m^2]
A_soil  = 440;       % [m^2]
V_max   = 10000;     % [L]
V_min   = 1000;      % [L] Minimum volume for cavitation protection

% Shallow Clay-Loam Soil Properties (Class IV - 25cm depth)
H_sat   = 50600;     % [L] Saturation capacity
H_cc    = 39600;     % [L] Field capacity
H_min   = 31000;     % [L] 50% MAD threshold
H_pmp   = 22000;     % [L] Permanent Wilting Point
D_max   = 14.7;      % [L/min] Saturated drainage rate
gamma   = 2;         % [-] Drainage exponent
f_max   = 0.70;      % [-] Maximum infiltration factor

% Actuators & Evapotranspiration
Q_max   = 60;        % [L/min] Pump capacity
tau     = 2;         % [min] Upscaled actuator inertia
Ce      = 0.9;       % [-] Roof runoff coefficient
ET0     = 1.2;       % [L/min] Base ET for 'pastos limpios'
k_T     = 0.03;      % [1/degC] Empirical thermal sensitivity

% Temperature & Economics
T_ref   = 22.0; T_mean = 22.0; A_annual = 3.0; A_daily = 4.0;
Cost_m3 = 2122;      % [COP/m^3] November 2025 dated Volumetric tariff

%% 2. TIME, CLIMATE & THERMODYNAMIC WDEL COUPLING
days = 365; dt = 1;
t = 1:dt:(days*1440); N = length(t); t_days = t / 1440;
hour_of_day = mod(t_days * 24, 24);

% Temperature Modeling (Diurnal wave with 8-hour phase shift)
T_year = 365 * 1440;
T_t = T_mean + A_annual*sin(2*pi*t/T_year) + A_daily*sin(2*pi*(t - 480)/1440);

% WDEL Thermodynamic Coupling with Strict Aerodynamic Bounds
eta_t = 0.85 - 0.015 * (T_t - 18);
eta_t = max(0.70, min(0.85, eta_t)); 

% Nocturnal Irrigation Window Boolean (18:00 to 06:00)
W_t = (hour_of_day >= 18) | (hour_of_day <= 6);

% Bimodal Stochastic Rainfall Generator (191 days, 1213 mm)
rng(42); % Fixed seed for reproducibility
Rain_mm = zeros(1, N);
days_with_rain = false(1, 365);
while sum(days_with_rain) < 191
    day = randi([1, 365]);
    prob = 0.1 + 0.9*exp(-0.5*((day-115)/30)^2) + 0.9*exp(-0.5*((day-300)/30)^2);
    if rand() < prob && ~days_with_rain(day)
        days_with_rain(day) = true;
        start_idx = (day-1)*1440 + randi([1, 1300]);
        duration = randi([40, 80]); 
        intensity = 0.03 + rand()*0.11; 
        Rain_mm(start_idx : start_idx+duration) = intensity;
    end
end
total_generated = sum(Rain_mm);
Rain_mm = Rain_mm * (1213 / total_generated); % Normalize to exact annual total

%% 3. INITIALIZATION (STATE VECTOR DEFINITION: x = [V_r, H, m]^T)
% Conservative cold start for rain tank, ideal initial state for soil
V_tank = zeros(1, N); V_tank(1) = V_min; 
H_soil = zeros(1, N); H_soil(1) = H_cc;
m_state = zeros(1, N); % Discrete memory state (Layer 2 Automaton)

H_baseline = zeros(1, N); H_baseline(1) = H_cc;

q_r = zeros(1, N); q_a = zeros(1, N);
Cost_Real = zeros(1, N); Cost_Baseline = zeros(1, N);

%% 4. SIMULATION LOOP 
timer_duration = 30; % Baseline operation duration [min]

for k = 1:N-1
    % --- EVAPOTRANSPIRATION DYNAMICS (FAO-56 derived) ---
    if H_soil(k) >= H_min, Ks = 1;
    elseif H_soil(k) > H_pmp, Ks = (H_soil(k) - H_pmp) / (H_min - H_pmp);
    else, Ks = 0; end
    ET_actual = ET0 * Ks * (1 + k_T*(T_t(k) - T_ref));
    
    if H_baseline(k) >= H_min, Ks_base = 1;
    elseif H_baseline(k) > H_pmp, Ks_base = (H_baseline(k) - H_pmp) / (H_min - H_pmp);
    else, Ks_base = 0; end
    ET_base_actual = ET0 * Ks_base * (1 + k_T*(T_t(k) - T_ref));
    
    % --- VARIABLE INFILTRATION DYNAMICS ---
    if H_soil(k) <= H_cc
        f_inf_actual = f_max;
    else
        f_inf_actual = f_max * max(0, 1 - (H_soil(k) - H_cc)/(H_sat - H_cc));
    end
    
    if H_baseline(k) <= H_cc
        f_inf_base_actual = f_max;
    else
        f_inf_base_actual = f_max * max(0, 1 - (H_baseline(k) - H_cc)/(H_sat - H_cc));
    end

    % --- HYBRID CONTROLLER (Hierarchical Layers) ---
    
    % Layer 2: Regulatory Control (Hybrid Automaton Jump Map for m(t))
    if k > 1
        m_state(k) = m_state(k-1); % Inherit previous state (m(t^+) = m(t) 'otherwise' condition)
    end
    
    % Evaluate transition logic (Eq. 13)
    if H_soil(k) <= H_min && W_t(k) == 1
        m_state(k) = 1;
    elseif H_soil(k) >= H_cc || W_t(k) == 0
        m_state(k) = 0;
    end
    
    % Layer 3: Rain-Priority Dispatch Logic (Eq. 14)
    u_r = 0; u_a = 0;
    if m_state(k) == 1
        if V_tank(k) > V_min
            u_r = 1; 
        else
            u_a = 1; 
        end
    end
    
    % Actuator Inertia (Eq. 9)
    q_r(k+1) = q_r(k) + (dt/tau)*(Q_max*u_r - q_r(k));
    q_a(k+1) = q_a(k) + (dt/tau)*(Q_max*u_a - q_a(k));
    
    % --- TANK DYNAMICS & EXACT LCP COMPLEMENTARITY ---
    inflow = Ce * A_roof * Rain_mm(k);
    % Layer 1: Strict Cavitation Protection enforcement
    pump_draw = q_r(k) * (V_tank(k) >= V_min); 
    
    % Exact calculation of unconstrained volume to compute theoretical overflow
    V_unbounded = V_tank(k) + (inflow - pump_draw) * dt;
    if V_unbounded > V_max
        overflow = (V_unbounded - V_max) / dt;
        V_tank(k+1) = V_max;
    else
        overflow = 0;
        V_tank(k+1) = max(0, V_unbounded); % Safeguard lower bound
    end
    
    % --- DRAINAGE DYNAMICS ---
    if H_soil(k) > H_cc, D = D_max * ((H_soil(k) - H_cc)/(H_sat - H_cc))^gamma; else, D = 0; end
    if H_baseline(k) > H_cc, D_base = D_max * ((H_baseline(k) - H_cc)/(H_sat - H_cc))^gamma; else, D_base = 0; end
    
    % --- SOIL STATE UPDATE ---
    soil_in = eta_t(k) * (q_r(k) + q_a(k)) + (f_inf_actual * A_soil * Rain_mm(k));
    H_soil(k+1) = min(H_sat, max(0, H_soil(k) + (soil_in - ET_actual - D)*dt));
    
    % --- TRADITIONAL BASELINE UPDATE ---
    minute_of_day = mod(k, 1440);
    % Fair comparison: Baseline triggered inside nocturnal window (20:00 = min 1200)
    if minute_of_day >= 1200 && minute_of_day < 1200 + timer_duration
        q_a_baseline = Q_max; 
    else
        q_a_baseline = 0;
    end
    soil_in_baseline = eta_t(k) * q_a_baseline + (f_inf_base_actual * A_soil * Rain_mm(k));
    H_baseline(k+1) = min(H_sat, max(0, H_baseline(k) + (soil_in_baseline - ET_base_actual - D_base)*dt));
    
    % --- HYDRO-ECONOMIC OPEX TRACKING ---
    Cost_Real(k+1) = Cost_Real(k) + (q_a(k)/1000)*Cost_m3*dt;
    Cost_Baseline(k+1) = Cost_Baseline(k) + (q_a_baseline/1000)*Cost_m3*dt;
end

Savings = Cost_Baseline(end) - Cost_Real(end);
Percentage = (Savings / Cost_Baseline(end)) * 100;

%% 5. VISUALIZATION 1: MAIN DOCUMENT FIGURE
fig1 = figure('Color', 'k', 'Position', [10, 10, 950, 1400]);

% --- PANEL A: TEMPERATURE ---
subplot(6,1,1);
plot(t_days(1:60:end), T_t(1:60:end), 'Color', '#FFA500', 'LineWidth', 1.5);
ylabel('Temp [°C]'); title('(A) Temperature Profile (La Mesa Climatology)', 'Color', 'w');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', '#FFA500'); grid on; xlim([0 365]); ylim([10 32]);

% --- PANEL B: RAINFALL ---
subplot(6,1,2);
daily_rain = sum(reshape(Rain_mm(1:365*1440), 1440, 365), 1);
bar(1:365, daily_rain, 'FaceColor', '#1E90FF', 'EdgeColor', 'none', 'BarWidth', 1);
ylabel('Rain [mm/day]'); title('(B) Bimodal Rainfall Profile', 'Color', 'w');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); grid on; xlim([0 365]);

% --- PANEL C: STORAGE REGULATION ---
subplot(6,1,3);
plot(t_days, H_baseline, 'Color', [1 0.2 0.2], 'LineWidth', 1.2); hold on;
plot(t_days, H_soil, 'Color', '#00FF7F', 'LineWidth', 1.5); 
yline(H_cc, '--w', 'H_{cc}', 'LabelHorizontalAlignment','left');
yline(H_min, '--y', 'H_{min} (MAD)', 'LabelHorizontalAlignment','left');
yline(H_pmp, '--r', 'H_{pmp} (PMP)', 'LabelHorizontalAlignment','left');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
ylabel('Storage [L]'); title('(C) Storage Regulation: Closed-Loop vs Open-Loop Timer', 'Color', 'w');
legend({'Baseline (Timer)', 'Hybrid System'}, 'TextColor', 'w', 'Color', 'k', 'Location', 'SouthWest');
grid on; xlim([0 365]); ylim([H_pmp*0.6, H_sat*1.05]); 

% --- PANEL D: RAIN TANK ---
subplot(6,1,4);
idx_a = 1:5:N; 
area(t_days(idx_a), V_tank(idx_a), 'FaceColor', 'cyan', 'FaceAlpha', 0.4, 'EdgeColor', 'cyan'); hold on;
yline(V_max, '--w', 'V_{max}', 'LabelHorizontalAlignment','left'); 
yline(V_min, 'r-', 'V_{min}', 'LabelHorizontalAlignment','left');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
ylabel('Volume [L]'); title('(D) Rain Tank Level', 'Color', 'w');
grid on; xlim([0 365]); ylim([0 V_max*1.15]);

% --- PANEL E: MONTHLY CONSUMPTION ---
subplot(6,1,5);
months = 1:12; usage_aq = zeros(1,12); usage_rn = zeros(1,12);
for m=1:12
    idx = (t_days >= (m-1)*30.41) & (t_days < m*30.41);
    usage_aq(m) = sum(q_a(idx)); usage_rn(m) = sum(q_r(idx));
end
b = bar(months, [usage_aq; usage_rn]', 'stacked');
b(1).FaceColor = '#FF00FF'; b(2).FaceColor = '#00FFFF'; b(1).EdgeColor = 'none'; b(2).EdgeColor = 'none';
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
legend({'Aqueduct', 'Rain'}, 'TextColor', 'w', 'Color', 'k', 'Location', 'NorthWest');
ylabel('Litres'); title('(E) Monthly Consumption', 'Color', 'w'); grid on; xticks(1:12);

% --- PANEL F: ECONOMIC IMPACT ---
subplot(6,1,6);
plot(t_days, Cost_Baseline, 'r--', 'LineWidth', 2); hold on;
plot(t_days, Cost_Real, 'g-', 'LineWidth', 2);
idx_f = 1:10:N; fill([t_days(idx_f), fliplr(t_days(idx_f))], [Cost_Real(idx_f), fliplr(Cost_Baseline(idx_f))], 'y', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
ylabel('Cost [COP]'); xlabel('Day of Year');
title(sprintf('(F) Cumulative Expenditure (OPEX Savings): COP $%d (%.1f%%)', round(Savings), Percentage), 'Color', 'w');
legend({'Baseline', 'Hybrid', 'Savings'}, 'TextColor', 'w', 'Color', 'k', 'Location', 'NorthWest'); grid on; xlim([0 365]);