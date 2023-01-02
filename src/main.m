close all
clear variables

%% Experiment 1
Scenario1_GM(33,50,50,2, ['a','b','c','d','e','f'])
Scenario1_RFS(33,50,50,2,0.8, ['a','b','c','g','h','i'])
Scenario1_RFS(33,50,50,2,0.99, ['a','b','c','j','k','l'])
Scenario1_IDO(33,50,50,2,0.5, ['a','b','c','m','n','o'])
Scenario1_IDO(33,50,50,2,1, ['a','b','c','p','q','r'])

%% Experiment 2
[RFS_errors1, RFS_falsePositiveErrors1, RFS_falseNegativeErrors1] = Scenario2_RFS(20,50,50,2,0.8,['a','b','c','d','e','f']);
[RFS_errors2, RFS_falsePositiveErrors2, RFS_falseNegativeErrors2] = Scenario2_RFS(20,50,50,2,0.99,['a','b','c','g','h','i']);
[SPP_errors1, SPP_falsePositiveErrors1, SPP_falseNegativeErrors1] = Scenario2_IDO(20,50,50,2,0.5,['a','b','c','j','k','l']);
[SPP_errors2, SPP_falsePositiveErrors2, SPP_falseNegativeErrors2] = Scenario2_IDO(20,50,50,2,1,['a','b','c','m','n','o']);

%% Plot Experiment 2
fig=gcf;
fig.Position(1:4)=[0,0,700,700];

% Plot average error
subplot(3,1,1)
plot(RFS_errors1,'-.','Color','#1984c5')
hold on
plot(RFS_errors2,':','Color','#1984c5','LineWidth',1)
plot(SPP_errors1,'--','Color','#c23728')
plot(SPP_errors2,'-','Color','#c23728')
legend({'Baseline \alpha = 0.8','Baseline \alpha = 0.99', 'Ours v_{max} = 0.5', 'Ours v_{max} = 1'})
hold off

% Plot false possitive error
subplot(3,1,2)
plot(RFS_falsePositiveErrors1,'-.','Color','#1984c5')
hold on
plot(RFS_falsePositiveErrors2,':','Color','#1984c5','LineWidth',1)
plot(SPP_falsePositiveErrors1,'--','Color','#c23728')
plot(SPP_falsePositiveErrors2,'-','Color','#c23728')
hold off

% Plot false negative error
subplot(3,1,3)
plot(RFS_falseNegativeErrors1,'-.','Color','#1984c5')
hold on
plot(RFS_falseNegativeErrors2,':','Color','#1984c5','LineWidth',1)
plot(SPP_falseNegativeErrors1,'--','Color','#c23728')
plot(SPP_falseNegativeErrors2,'-','Color','#c23728')
hold off

% Save the plot
saveas(gcf,'../figures/Experiment2.error.svg')
close

%% Experiment 3
[RFS_ElapsedTimes] = Scenario3_RFS(10,7,7,15,0.99,'a');
mean(RFS_ElapsedTimes)
[SPP_ElapsedTimes] = Scenario3_IDO(10,7,7,15,0.5,'b');
mean(SPP_ElapsedTimes)
