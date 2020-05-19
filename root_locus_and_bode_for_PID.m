%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Iterable params
Ux_ = 5:3:29;       % [m/s] NOTE: make the number of iterable values have
                    %       a round square root! (e.g 4, 9, 16)
                    
lenUx = length(Ux_);

%% GENERATE CONTROLLER TF
s = tf('s');
a = 0.1;
b = 0.1;
K_s = (a*s^2 + s + b) / s;

%% PLOT ROOT LOCI
figure;
plot_dim = ceil(sqrt(lenUx));
subplot(plot_dim, plot_dim, 1);
% sgtitle("Plant Dynamics Without Control");
sgtitle("Plant Dynamics With Control, a = " + a + ", b = " + b);

% Run for different Ux values
for i = 1:lenUx
    % Select speed
    Ux = Ux_(i);

    % CALCULATE PLANT DYNAMICS
    % A = [aM,  bM,  cM,  dM]
    %     [eM,  fM,  gM,  hM]
    %     [iM,  jM,  kM,  lM]
    %     [mM,  nM,  oM,  pM]
    aM = 0;
    bM = 1;
    cM = 0;
    dM = 0;
    eM = 0;
    fM = -(f_tire.Ca_lin + r_tire.Ca_lin)/(veh.m*Ux);
    gM = (f_tire.Ca_lin + r_tire.Ca_lin)/veh.m;
    hM = (-veh.a * f_tire.Ca_lin + veh.b*r_tire.Ca_lin)/(veh.m*Ux);
    iM = 0;
    jM = 0;
    kM = 0;
    lM = 1;
    mM = 0;
    nM = (veh.b * r_tire.Ca_lin - veh.a*f_tire.Ca_lin)/(veh.Iz*Ux);
    oM = (veh.a * f_tire.Ca_lin - veh.b*r_tire.Ca_lin)/veh.Iz;
    pM = -(veh.a^2 * f_tire.Ca_lin + veh.b^2 *r_tire.Ca_lin)/(veh.Iz*Ux);

    A = [[aM,  bM,  cM,  dM];
         [eM,  fM,  gM,  hM];
         [iM,  jM,  kM,  lM];
         [mM,  nM,  oM,  pM]];
     
    B = [0; f_tire.Ca_lin / veh.m; 0; veh.a * f_tire.Ca_lin / veh.Iz];
    C = [1, 0, 0, 0];
    D = [0];
    
    plant_sys = ss(A, B, C, D);
    
    subplot(plot_dim, plot_dim, i);
    
%     rlocus(plant_sys);
    rlocus(plant_sys * K_s);
    hold on;
    xlabel('Real Axis')
    ylabel('Imaginary Axis')
    title("Ux = " + Ux + " m/s");
    
end

%% GENERATE BODE PLOTS OF FINAL CONTROL + PLANT
opts = bodeoptions('cstprefs');
opts.Magunits = 'abs';
opts.Magscale = 'log';

figure;
plot_dim = ceil(sqrt(lenUx));
subplot(plot_dim, plot_dim, 1);
% sgtitle("Plant Dynamics Without Control");
sgtitle("Plant Dynamics With Control, a = " + a + ", b = " + b);

% Run for different Ux values
for i = 1:lenUx
    % Select speed
    Ux = Ux_(i);

    % CALCULATE PLANT DYNAMICS
    % A = [aM,  bM,  cM,  dM]
    %     [eM,  fM,  gM,  hM]
    %     [iM,  jM,  kM,  lM]
    %     [mM,  nM,  oM,  pM]
    aM = 0;
    bM = 1;
    cM = 0;
    dM = 0;
    eM = 0;
    fM = -(f_tire.Ca_lin + r_tire.Ca_lin)/(veh.m*Ux);
    gM = (f_tire.Ca_lin + r_tire.Ca_lin)/veh.m;
    hM = (-veh.a * f_tire.Ca_lin + veh.b*r_tire.Ca_lin)/(veh.m*Ux);
    iM = 0;
    jM = 0;
    kM = 0;
    lM = 1;
    mM = 0;
    nM = (veh.b * r_tire.Ca_lin - veh.a*f_tire.Ca_lin)/(veh.Iz*Ux);
    oM = (veh.a * f_tire.Ca_lin - veh.b*r_tire.Ca_lin)/veh.Iz;
    pM = -(veh.a^2 * f_tire.Ca_lin + veh.b^2 *r_tire.Ca_lin)/(veh.Iz*Ux);

    A = [[aM,  bM,  cM,  dM];
         [eM,  fM,  gM,  hM];
         [iM,  jM,  kM,  lM];
         [mM,  nM,  oM,  pM]];
     
    B = [0; f_tire.Ca_lin / veh.m; 0; veh.a * f_tire.Ca_lin / veh.Iz];
    C = [1, 0, 0, 0];
    D = [0];
    
    plant_sys = ss(A, B, C, D);
    
    subplot(plot_dim, plot_dim, i);
    
    bode(plant_sys * K_s, opts);
    hold on;
    grid on;
    title("Ux = " + Ux + " m/s");
    
end