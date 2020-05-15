%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;


% From ME227_simulator, PID guesses. 
% gains.K_lat_p_PID = 0.1;           % [] TODO:  Fill in units here and tune
% gains.K_lat_i_PID = 0.001;    % []        more thoughtfully
% gains.K_lat_d_PID = 7;         % []

% Gains and conditions
K_i = 0.001; % we will just not touch the integral term. 
K_p_ = 0:1:10;   % 
K_d_ = 0:.1:.5;           %  


% Values of Ux_ chosen based on Ux_desired. avg des Ux = 7.7, max is 12.5
Ux_ = 3:3:12;       % [m/s] %NOTE: make the number of iterable values have a round square root! (e.g 4, 9, 16)
lenKp = length(K_p_);
lenKd = length(K_d_);
lenUx = length(Ux_);

% Allocate space for poles (We know there are 4)
poles_ = zeros(5,lenKp);

%--------------------------------------------------------------------------
%% CREATE SYSTEM MATRIX
%--------------------------------------------------------------------------
for i = 1:lenKd
    % Select lookahead distance
    K_d = K_d_(i);
    
    figure;
    plot_dim = ceil(sqrt(lenUx));
    subplot(plot_dim, plot_dim, 1);
    sgtitle("K_{d} Gain of " + K_d);
    
    for j = 1:lenUx
        % Select speed
        Ux = Ux_(j);
        
        subplot(plot_dim, plot_dim, j);
        
        for k = 1:lenKp
        % Select lookahead gain
        K_p = K_p_(k);
        
            %{
            To make the state matrix easier to input, create each term separately
            here according to this template - we'll complile these into the matrix
            at the end. We recommend you keep this general and let MATLAB fill in
            each of the values as you set them up above. Then you can copy and
            paste this section into later problems.

                A = [uM, qM,  rM,  sM,  tM]
                    [vM, aM,  bM,  cM,  dM]
                    [wM, eM,  fM,  gM,  hM]
                    [xM, iM,  jM,  kM,  lM]
                    [yM, mM,  nM,  oM,  pM]
            %}
        
            % we are changing wM, eM, fM, yM, mM, nM. Added another column
            % for new state, integral of e. 
                        
            d_row3 = -f_tire.Ca_lin/veh.m;
            d_row5 = -veh.a*f_tire.Ca_lin/veh.Iz;
            
            uM = 0;
            vM = 0;
            wM = 0 + d_row3*K_i;
            xM = 0;
            yM = 0 + d_row5*K_i;
            qM = 1;
            rM = 0;
            sM = 0;
            tM = 0;
            
            aM = 0;
            bM = 1;
            cM = 0;
            dM = 0;
            eM = 0 + d_row3*K_p;
            fM = -(f_tire.Ca_lin + r_tire.Ca_lin)/(veh.m*Ux)  + d_row3*K_d;
            gM = (f_tire.Ca_lin + r_tire.Ca_lin)/veh.m;
            hM = (-veh.a * f_tire.Ca_lin + veh.b*r_tire.Ca_lin)/(veh.m*Ux);
            iM = 0;
            jM = 0;
            kM = 0;
            lM = 1;
            mM = 0 + d_row5*K_p;
            nM = (veh.b * r_tire.Ca_lin - veh.a*f_tire.Ca_lin)/(veh.Iz*Ux) + d_row5*K_d;
            oM = (veh.a * f_tire.Ca_lin - veh.b*r_tire.Ca_lin)/veh.Iz;
            pM = -(veh.a^2 * f_tire.Ca_lin + veh.b^2 *r_tire.Ca_lin)/(veh.Iz*Ux);

            A = [[uM,  qM,  rM,  sM,  tM];
                 [vM,  aM,  bM,  cM,  dM];
                 [wM,  eM,  fM,  gM,  hM];
                 [xM,  iM,  jM,  kM,  lM];
                 [yM,  mM,  nM,  oM,  pM]];

            % Calculate pole positions
            poles_(:,k) = eig(A);
        end
        cmap = colormap(winter(lenKp));
        for idx = 1:lenKp
            plot(real(poles_(:,idx)), imag(poles_(:,idx)), 'x', 'Color', cmap(idx,:))
            hold on
        end
        xline(0,'--');
        grid on
        xlabel('Real Axis')
        ylabel('Imaginary Axis')
        cbar = colorbar('Ticks', K_p_);
        caxis([K_p_(1) K_p_(end)])
        cbar.Label.String = 'K_{p}';
        title(Ux + " m/s");
        xlim([-10 2]);
        ylim([-20 20]);
        
        
    end
   
end

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------

figure
cmap = colormap(winter(lenKp));
for idx = 1:lenKp
    plot(real(poles_(:,idx)), imag(poles_(:,idx)), 'x', 'Color', cmap(idx,:))
    hold on
end
xline(0,'--');
grid on
xlabel('Real Axis')
ylabel('Imaginary Axis')
cbar = colorbar('Ticks', K_p_);
caxis([K_p_(1) K_p_(end)])
cbar.Label.String = 'K_{p} [N/m]';