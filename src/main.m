close all
clear variables

%% Experiment 1

decay1 = 0.8;
decay2 = 1;

Scenario1_GM(20,50,50,2,['a','b','c','d','e','f']);
[RFS_errors1, RFS_falsePositiveErrors1, RFS_falseNegativeErrors1] = Scenario1_RFS(20,50,50,2,decay1,['a','b','c','g','h','i']);
[RFS_errors2, RFS_falsePositiveErrors2, RFS_falseNegativeErrors2] = Scenario1_RFS(20,50,50,2,decay2,['a','b','c','j','k','l']);
[SPP_errors1, SPP_falsePositiveErrors1, SPP_falseNegativeErrors1] = Scenario1_IDO(20,50,50,2,0.5,decay1,['a','b','c','m','n','o']);
[SPP_errors2, SPP_falsePositiveErrors2, SPP_falseNegativeErrors2] = Scenario1_IDO(20,50,50,2,0.5,decay2,['a','b','c','p','q','r']);


%% Plots Experiment 1

figure
fig=gcf;
fig.Position(1:4)=[0,0,700,700];

% Plot average error
subplot(3,1,1)
plot(RFS_errors1,'-.','Color','#1984c5')
hold on
plot(RFS_errors2,':','Color','#1984c5','LineWidth',1)
plot(SPP_errors1,'--','Color','#c23728')
plot(SPP_errors2,'-','Color','#c23728')
legend({strcat('Baseline \alpha=', num2str(decay1)),strcat('Baseline \alpha=', num2str(decay2)), strcat('Ours \delta=', num2str(decay1)), strcat('Ours \delta=', num2str(decay2))})
yl = ylim;
text(2,0.9*yl(2),'(a)')
hold off

% Plot false possitive error
subplot(3,1,2)
plot(RFS_falsePositiveErrors1,'-.','Color','#1984c5')
hold on
plot(RFS_falsePositiveErrors2,':','Color','#1984c5','LineWidth',1)
plot(SPP_falsePositiveErrors1,'--','Color','#c23728')
plot(SPP_falsePositiveErrors2,'-','Color','#c23728')
yl = ylim;
text(2,0.9*yl(2),'(b)')
hold off

% Plot false negative error
subplot(3,1,3)
plot(RFS_falseNegativeErrors1,'-.','Color','#1984c5')
hold on
plot(RFS_falseNegativeErrors2,':','Color','#1984c5','LineWidth',1)
plot(SPP_falseNegativeErrors1,'--','Color','#c23728')
plot(SPP_falseNegativeErrors2,'-','Color','#c23728')
yl = ylim;
text(2,0.9*yl(2),'(c)')
xlabel('Time (s)')
hold off

% Save the plot
saveas(gcf,'../figures/Experiment1.error.svg')


%% Experiment 2
% Running all the experiments can take a while. If you don't want to run
% them all, you can load the saved workspace at the following section.

rng(2023)
n_simulations = 100;
sim_horizon = 100;

% Patameters to run for each method
parameters = [0.70 0.75 0.80 0.85 0.90 0.95 1;
              0.70 0.75 0.80 0.85 0.90 0.95 1];
n_parameters = size(parameters,2);

% Error variables
error = zeros(n_simulations,n_parameters,2,sim_horizon,3);
IDO_elapsedTimes = zeros(n_simulations,n_parameters,sim_horizon);
RFS_elapsedTimes = zeros(n_simulations,n_parameters,sim_horizon);

for i = 1:n_simulations
    % Randomly select the number of obstacles
    n_obstacles = randi(5);
    % Generate random trajectory for the obstacles
    ob_trajectories = randomParticles(n_obstacles, 20, 3 ,4, 1, sim_horizon, 0.5,25,25);
    % Run the scenario for each parameter
    for j = 1:n_parameters
        strcat('Parameter', int2str(parameters(1,j)*100))
        [error(i,j,1,:,1), error(i,j,1,:,2), error(i,j,1,:,3), IDO_elapsedTimes(i,j,:)] = Simulate_IDO(0.5,parameters(1,j),ob_trajectories,[sim_horizon],sim_horizon,i);
        [error(i,j,2,:,1), error(i,j,2,:,2), error(i,j,2,:,3), RFS_elapsedTimes(i,j,:)] = Simulate_RFS(parameters(2,j),ob_trajectories,[sim_horizon],sim_horizon,i);
    end
end

% Save the current workspace
save('workspace')

%% Compute mean and std dev error
% The previous workspace can be loaded with the following command:
% load('workspace')

IDO_mean_error = mean(mean(error(:,:,1,:,1),4),1);
RFS_mean_error = mean(mean(error(:,:,2,:,1),4),1);
IDO_free_error = mean(mean(error(:,:,1,:,2),4),1);
RFS_free_error = mean(mean(error(:,:,2,:,2),4),1);
IDO_occ_error = mean(mean(error(:,:,1,:,3),4),1);
RFS_occ_error = mean(mean(error(:,:,2,:,3),4),1);

IDO_mean_error_std = std(mean(error(:,:,1,:,1),4),1);
RFS_mean_error_std = std(mean(error(:,:,2,:,1),4),1);
IDO_free_error_std = std(mean(error(:,:,1,:,2),4),1);
RFS_free_error_std = std(mean(error(:,:,2,:,2),4),1);
IDO_occ_error_std = std(mean(error(:,:,1,:,3),4),1);
RFS_occ_error_std = std(mean(error(:,:,2,:,3),4),1);

%% Plots Experiment 2

figure
fig=gcf;
fig.Position(1:4)=[0,0,700,700];
t=tiledlayout(3,1);

% Plot average error
ax1 = nexttile;
e1 = errorbar(ax1,parameters(1,:), IDO_mean_error, IDO_mean_error_std,IDO_mean_error_std,'Color','#c23728');
xlim([0.69, 1.01])
ylim([0 0.08])
xticks(parameters(1,:))
ax1.XColor = '#c23728';
xlabel('\delta (Decay factor)');

hold on

ax2 = axes(t);
ax2.XAxisLocation = 'top';
ax2.YAxis.Visible = 'off';
ax2.Color = 'none';
hold on
e2 = errorbar(ax2,parameters(2,:), RFS_mean_error, RFS_mean_error_std,RFS_mean_error_std,'Color','#1984c5');
xlim([0.69, 1.01])
ylim([0 0.08])
xticks(parameters(1,:))
ax2.XColor = '#1984c5';
xlabel('\alpha (Free-space discount factor)');
box off

legend([e1,e2],'Ours','Baseline')

% Plot free cells error
ax3 = nexttile;
e1 = errorbar(ax3,parameters(1,:), IDO_free_error, IDO_free_error_std,IDO_free_error_std,'Color','#c23728');
xlim([0.69, 1.01])
ylim([0 0.08])
xticks(parameters(1,:))
ax3.XColor = '#c23728';
xlabel('\delta (Decay factor)');

hold on

ax4 = axes(t);
ax4.Layout.Tile = 2;
ax4.XAxisLocation = 'top';
ax4.YAxis.Visible = 'off';
ax4.Color = 'none';
hold on
e2 = errorbar(ax4,parameters(2,:), RFS_free_error, RFS_free_error_std,RFS_free_error_std,'Color','#1984c5');
xlim([0.69, 1.01])
ylim([0 0.08])
xticks(parameters(1,:))
ax4.XColor = '#1984c5';
xlabel('\alpha (Free-space discount factor)');
box off

% Plot occupied cells error
ax5 = nexttile;
e1 = errorbar(ax5,parameters(1,:), IDO_occ_error, IDO_occ_error_std,IDO_occ_error_std,'Color','#c23728');
xlim([0.69, 1.01])
ylim([0.3 0.7])
xticks(parameters(1,:))
ax5.XColor = '#c23728';
xlabel('\delta (Decay factor)');

hold on

ax6 = axes(t);
ax6.Layout.Tile = 3;
ax6.XAxisLocation = 'top';
ax6.YAxis.Visible = 'off';
ax6.Color = 'none';
hold on
e2 = errorbar(ax6,parameters(2,:), RFS_occ_error, RFS_occ_error_std,RFS_occ_error_std,'Color','#1984c5');
xlim([0.69, 1.01])
ylim([0.3 0.7])
xticks(parameters(1,:))
ax6.XColor = '#1984c5';
xlabel('\alpha (Free-space discount factor)');
box off







