%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% !!! NOTE !!!  THIS DOESN'T CURRENTLY WORK; I THINK I DID BODE FOR THE
% CLOSED-LOOP SYSTEM AND NEED TO DO IT FOR THE OPEN-LOOP SYSTEM AS I DID
% FOR THE PID CONTROLLER.  NEED TO FIND A TRANSFER FUNCTION FOR THE
% LOOKAHEAD CONTROLLER ON ITS OWN, THEN CAN COMBINE WITH THE PLANT DYNAMICS
% LIKE FOR THE PID CONTROLLER.

% Load Niki params
setup_niki;

% Iterable params
Ux_ = 5:3:29;       % [m/s] NOTE: make the number of iterable values have
                    %       a round square root! (e.g 4, 9, 16)
                    
lenUx = length(Ux_);

% Controller gains
K_la = 7000;              % [N/m]
x_la = 12;                % [m]
K_long = veh.m*.1*g;      % [N/m]

%% GENERATE CONTROLLER TF
s = tf('s');
controller = s;

%% GENERATE BODE PLOTS OF FINAL CONTROL + PLANT
opts = bodeoptions('cstprefs');
opts.Magunits = 'abs';
opts.Magscale = 'log';

figure;
plot_dim = ceil(sqrt(lenUx));
subplot(plot_dim, plot_dim, 1);
sgtitle("Bode of Plant With Lookahead Control");

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
    
    subplot(controller * plant_sys, plot_dim, i);
    
    bode(full_sys, opts);
    hold on;
    grid on;
    title("Ux = " + Ux + " m/s");
    
end