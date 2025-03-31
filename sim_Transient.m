
%% Simulation settings Overall
velocity_init = 40/3.6;

simulation.veh0.x_init     = [0; velocity_init+1; 0];
simulation.veh1.x_init     = [-(control.ri + control.h*velocity_init + control.L)+1; velocity_init; 0];
mode0 = 0;

% Controller init
rho11_init = - (control.kd/control.h*(simulation.veh0.x_init(1) - simulation.veh1.x_init(1) - control.L) - control.kd*simulation.veh1.x_init(2) + model.tau/control.h*simulation.veh1.x_init(3));
rho12_init = rho11_init;
rho21_init = - (control.kd/control.h*(simulation.veh0.x_init(1) - simulation.veh1.x_init(1) - control.L) - control.kd*simulation.veh1.x_init(2));
rho22_init = rho21_init;

simulation.rho_init = [rho11_init;
                       rho12_init;
                       rho21_init
                       rho22_init];

% run simulation
out = sim("CDC2025_HybridAutomaton", [0, sim_T])

%% Allocate simulation output
simulation.veh0.v   = out.x_0(:,2);
simulation.veh0.a   = out.x_0(:,3);
simulation.veh0.u   = out.u_0;

simulation.veh1.d   = out.x_0(:,1) - out.x_1(:,1) - control.L;
simulation.veh1.v   = out.x_1(:,2);
simulation.veh1.a   = out.x_1(:,3);
simulation.veh1.u   = out.u_1;
simulation.veh1.dr  = control.ri + control.h*simulation.veh1.v;

simulation.y5a = squeeze(out.y_att_1(3,1,:));
simulation.y5b = squeeze(out.y_att_1(4,1,:));
simulation.y6a = squeeze(out.y_att_1(5,1,:));
simulation.y6b = squeeze(out.y_att_1(6,1,:));

simulation.rho1a = out.rho_hybrid(:,1);
simulation.rho1b = out.rho_hybrid(:,2);
simulation.rho2a = out.rho_hybrid(:,3);
simulation.rho2b = out.rho_hybrid(:,4);

% Determine ground truth mode q

for it = 1:length(out.tout) 
    if simulation.att_5a_start1 <= out.tout(it) && out.tout(it) < simulation.att_5a_end1
        q_true(it) = 1;

    elseif simulation.att_5a_start2 <= out.tout(it) && out.tout(it) < simulation.att_5a_end2
        q_true(it) = 1;

    elseif simulation.att_5b_start <= out.tout(it) && out.tout(it) < simulation.att_5b_end
        q_true(it) = 2;

    elseif simulation.att_6a_start <= out.tout(it) && out.tout(it) < simulation.att_6a_end
        q_true(it) = 3;

    elseif simulation.att_6b_start <= out.tout(it) && out.tout(it) < simulation.att_6b_end
        q_true(it) = 4;

    else
        q_true(it) = 0;
    end
end

%% Mode plot
% figure
% plot(out.t_hybrid, out.mode, 'color', colorblind(2,:), 'linewidth', 1.5); hold on;
% plot(out.t_hybrid, q_true, 'color', colorblind(1,:), 'linewidth', 1.5); 
% set(gca, 'YTick', [0, 1, 2, 3, 4], 'YTickLabel', {'$q_{0}$','$q_{1|1}$', '$q_{1|2}$', '$q_{2|1}$', '$q_{2|2}$', ''}, 'FontSize', FontSizeAxes); ylim([0 4.2]); xlim([0 out.tout(end)])
% xlabel('Time [s]', 'fontsize', FontSizeAxes); %ylabel('$[-]$', 'fontsize', 15)
% legend( 'Mode $q_{j|k}$','Ground truth', 'fontsize', FontSizeLegend)

%% System output and Controller input
figure
subplot(2,1,1)
plot(out.t_hybrid, out.u1a, 'color', colorblind(1,:), 'linewidth', 1.5); hold on
plot(out.t_hybrid, out.u1b, 'color', colorblind(2,:), 'linewidth', 1.5); 
plot(out.t_hybrid, out.u2a, 'color', colorblind(5,:), 'linewidth', 1.5); 
plot(out.t_hybrid, out.u2b, 'color', colorblind(6,:), 'linewidth', 1.5); 
plot(out.t_hybrid, out.uhealthy, 'color', colorblind(8,:), 'linewidth', 1.5); 
legend('$u_{1|1}$', '$u_{1|2}$', '$u_{2|1}$', '$u_{2|2}$', '$u^*$', 'fontsize', FontSizeLegend-4, 'location', 'northwest')
xlabel('Time [s]', 'fontsize', FontSizeAxes); ylabel('$u_{j|k} \, [m/s^{-2}]$', 'fontsize', FontSizeAxes)

% figure
subplot(2,1,2)
plot(out.t_hybrid, simulation.y5a, 'color', colorblind(1,:), 'linewidth', 1.5); hold on
plot(out.t_hybrid, simulation.y5b, 'color', colorblind(2,:), 'linewidth', 1.5); 
plot(out.t_hybrid, simulation.y6a, 'color', colorblind(5,:), 'linewidth', 1.5); 
plot(out.t_hybrid, simulation.y6b, 'color', colorblind(6,:), 'linewidth', 1.5); 
legend('$y_{5|1}$', '$y_{5|2}$', '$y_{6|1}$', '$y_{6|2}$', 'fontsize', FontSizeLegend-4, 'location', 'northwest')
xlabel('Time [s]', 'fontsize', FontSizeAxes); ylabel('$y_{4+j|k} \, [m/s^{-2}]$', 'fontsize', FontSizeAxes)


%% Controller states
figure
subplot(2,1,1)
plot(out.t_hybrid, simulation.rho1a, 'color', colorblind(1,:), 'linewidth', 1.5); hold on
plot(out.t_hybrid, simulation.rho1b, 'color', colorblind(2,:), 'linewidth', 1.5);
legend('$\rho_{1|1}$', '$\rho_{1|2}$', 'fontsize', FontSizeLegend, 'location', 'northwest')
xlabel('Time [s]', 'fontsize', FontSizeAxes); ylabel('$\rho_{1|k} \, [\frac{m}{s^2}]$', 'fontsize', FontSizeAxes)

% create a new pair of axes inside current figure
axes('position',[.3 .75 .2 .15])
box on % put box around new pair of axes
indexOfInterest = (out.tout < 1 & out.tout > 0.5); % range of t near perturbation
plot(out.tout(indexOfInterest), simulation.rho1a(indexOfInterest), 'color', colorblind(1,:), 'linewidth', 1.5); hold on % plot on new axes
plot(out.tout(indexOfInterest), simulation.rho1b(indexOfInterest), 'color', colorblind(2,:), 'linewidth', 1.5) % plot on new axes
axis tight

subplot(2,1,2)
plot(out.t_hybrid, simulation.rho2a, 'color', colorblind(5,:), 'linewidth', 1.5); hold on
plot(out.t_hybrid, simulation.rho2b, 'color', colorblind(6,:), 'linewidth', 1.5);
legend('$\rho_{2|1}$', '$\rho_{2|2}$', 'fontsize', FontSizeLegend, 'location', 'northwest')
xlabel('Time [s]', 'fontsize', FontSizeAxes); ylabel('$\rho_{2|k} \, [\frac{m}{s^2}]$', 'fontsize', FontSizeAxes)

%% Vehicle states
figure
subplot(3,1,1)
plot(out.t_hybrid, simulation.veh1.d, 'color', colorblind(1,:), 'linewidth', 1.5); hold on
plot(out.t_hybrid, simulation.veh1.dr, '--', 'color', colorblind(2,:), 'linewidth', 1.5);
legend('$Actual$', '$Desired$', 'fontsize', FontSizeLegend, 'location', 'southeast')
xlabel('Time [s]', 'fontsize', FontSizeAxes); ylabel('$d_i \, [m]$', 'fontsize', FontSizeAxes)

subplot(3,1,2)
plot(out.t_hybrid, simulation.veh1.v, 'color', colorblind(1,:), 'linewidth', 1.5); hold on
plot(out.t_hybrid, simulation.veh0.v, 'color', colorblind(2,:), 'linewidth', 1.5);
legend('Vehicle 1', 'Vehicle 2', 'fontsize', FontSizeLegend, 'location', 'southeast')
xlabel('Time [s]', 'fontsize', FontSizeAxes); ylabel('$v_i \, [\frac{m}{s}]$', 'fontsize', FontSizeAxes)

subplot(3,1,3)
plot(out.t_hybrid, out.u1a, 'color', colorblind(1,:), 'linewidth', 1.5); hold on
plot(out.t_hybrid, out.u1b, 'color', colorblind(2,:), 'linewidth', 1.5); 
plot(out.t_hybrid, out.u2a, 'color', colorblind(5,:), 'linewidth', 1.5); 
plot(out.t_hybrid, out.u2b, 'color', colorblind(6,:), 'linewidth', 1.5); 
plot(out.t_hybrid, out.uhealthy, 'color', colorblind(8,:), 'linewidth', 1.5); 
legend('$u_{1|1}$', '$u_{1|2}$', '$u_{2|1}$', '$u_{2|2}$', '$u^*$', 'fontsize', FontSizeLegend-5, 'location', 'best', 'NumColumns', 2)
xlabel('Time [s]', 'fontsize', FontSizeAxes); ylabel('$u_{j|k} \, [m/s^{-2}]$', 'fontsize', FontSizeAxes)

%% Mode plot
figure
subplot(2,1,1)
plot(out.t_hybrid, out.mode, 'color', colorblind(2,:), 'linewidth', 1.5); hold on;
plot(out.t_hybrid, q_true, 'color', colorblind(1,:), 'linewidth', 1.5); 
set(gca, 'YTick', [0, 1, 2, 3, 4], 'YTickLabel', {'$q_{0}$','$q_{1|1}$', '$q_{1|2}$', '$q_{2|1}$', '$q_{2|2}$', ''}, 'FontSize', FontSizeAxes); ylim([0 4.2]); xlim([0 out.tout(end)])
xlabel('Time [s]', 'fontsize', FontSizeAxes); %ylabel('$[-]$', 'fontsize', 15)
legend( 'Mode $q_{j|k}$','Ground truth', 'fontsize', FontSizeLegend)

subplot(2,1,2)
plot(out.t_hybrid, simulation.y5a, 'color', colorblind(1,:), 'linewidth', 1.5); hold on
plot(out.t_hybrid, simulation.y5b, 'color', colorblind(2,:), 'linewidth', 1.5); 
plot(out.t_hybrid, simulation.y6a, 'color', colorblind(5,:), 'linewidth', 1.5); 
plot(out.t_hybrid, simulation.y6b, 'color', colorblind(6,:), 'linewidth', 1.5); 
legend('$y_{5|1}$', '$y_{5|2}$', '$y_{6|1}$', '$y_{6|2}$', 'fontsize', FontSizeLegend-4, 'location', 'best', 'NumColumns', 2)
xlabel('Time [s]', 'fontsize', FontSizeAxes); ylabel('$y_{4+j|k} \, [m/s^{-2}]$', 'fontsize', FontSizeAxes)
