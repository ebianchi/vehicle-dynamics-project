%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Gains and conditions
K_la_ = 1000:1000:10000;   % [N/m]
x_la_ = 5:1:15;           % [m]    
Ux_ = 5:1:13;       % [m/s] %NOTE: make the number of iterable values have a round square root! (e.g 4, 9, 16)
lenKla = length(K_la_);
lenXla = length(x_la_);
lenUx = length(Ux_);

% Allocate space for poles (We know there are 4)
poles_ = zeros(4,lenKla);

%--------------------------------------------------------------------------
%% CREATE SYSTEM MATRIX
%--------------------------------------------------------------------------
for i = 1:lenXla
    % Select lookahead distance
    x_la = x_la_(i);
    
    figure;
    plot_dim = ceil(sqrt(lenUx));
    subplot(plot_dim, plot_dim, 1);
    sgtitle("Lookahead Distance of " + x_la + " m");
    
    for j = 1:lenUx
        % Select speed
        Ux = Ux_(j);
        
        subplot(plot_dim, plot_dim, j);
        
        for k = 1:lenKla
        % Select lookahead gain
        K_la = K_la_(k);
        
            %{
            To make the state matrix easier to input, create each term separately
            here according to this template - we'll complile these into the matrix
            at the end. We recommend you keep this general and let MATLAB fill in
            each of the values as you set them up above. Then you can copy and
            paste this section into later problems.

                A = [aM,  bM,  cM,  dM]
                    [eM,  fM,  gM,  hM]
                    [iM,  jM,  kM,  lM]
                    [mM,  nM,  oM,  pM]
            %}

            aM = 0;
            bM = 1;
            cM = 0;
            dM = 0;
            eM = -K_la/veh.m;
            fM = -(f_tire.Ca_lin + r_tire.Ca_lin)/(veh.m*Ux);
            gM = (f_tire.Ca_lin + r_tire.Ca_lin)/veh.m - K_la * x_la / veh.m;
            hM = (-veh.a * f_tire.Ca_lin + veh.b*r_tire.Ca_lin)/(veh.m*Ux);
            iM = 0;
            jM = 0;
            kM = 0;
            lM = 1;
            mM = -K_la * veh.a / veh.Iz;
            nM = (veh.b * r_tire.Ca_lin - veh.a*f_tire.Ca_lin)/(veh.Iz*Ux);
            oM = (veh.a * f_tire.Ca_lin - veh.b*r_tire.Ca_lin)/veh.Iz - K_la * veh.a * x_la / veh.Iz;
            pM = -(veh.a^2 * f_tire.Ca_lin + veh.b^2 *r_tire.Ca_lin)/(veh.Iz*Ux);

            A = [[aM,  bM,  cM,  dM];
                 [eM,  fM,  gM,  hM];
                 [iM,  jM,  kM,  lM];
                 [mM,  nM,  oM,  pM]];

            % Calculate pole positions
            poles_(:,k) = eig(A);
        end
        cmap = colormap(winter(lenKla));
        for idx = 1:lenKla
            plot(real(poles_(:,idx)), imag(poles_(:,idx)), 'x', 'Color', cmap(idx,:))
            hold on
        end
        xline(0,'--');
        grid on
        xlabel('Real Axis')
        ylabel('Imaginary Axis')
        cbar = colorbar('Ticks', K_la_);
        caxis([K_la_(1) K_la_(end)])
        cbar.Label.String = 'K_{la} [N/m]';
        title(Ux + " m/s");
    end
   
end

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
figure
cmap = colormap(winter(lenKla));
for idx = 1:lenKla
    plot(real(poles_(:,idx)), imag(poles_(:,idx)), 'x', 'Color', cmap(idx,:))
    hold on
end
xline(0,'--');
grid on
xlabel('Real Axis')
ylabel('Imaginary Axis')
cbar = colorbar('Ticks', K_la_);
caxis([K_la_(1) K_la_(end)])
cbar.Label.String = 'K_{la} [N/m]';