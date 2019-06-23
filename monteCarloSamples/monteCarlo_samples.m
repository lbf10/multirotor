%% Markov sample generator for controller evaluation
% Defines limits
lim_massPercentage = [0,0.65];     % [%]
lim_endTime = [10,20];           % [s]
lim_disturbance = [0,15];       % [m/s2]
lim_coefficients = [0,1];
lim_failures = {{''}
            {'setRotorStatus(1,''motor loss'',0.78)'}
            {'setRotorStatus(1,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.78)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.78)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)','setRotorStatus(4,''motor loss'',0.78)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(3,''motor loss'',0.001)','setRotorStatus(4,''motor loss'',0.001)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.001)','setRotorStatus(8,''motor loss'',0.78)'}
            {'setRotorStatus(1,''motor loss'',0.001)','setRotorStatus(2,''motor loss'',0.001)','setRotorStatus(7,''motor loss'',0.001)','setRotorStatus(8,''motor loss'',0.001)'}
            {'setRotorStatus(1,''prop loss'',0.10)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(3,''prop loss'',0.10)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(3,''prop loss'',0.10)','setRotorStatus(4,''prop loss'',0.10)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(7,''prop loss'',0.10)','setRotorStatus(8,''prop loss'',0.10)'}};
lim_controlLoopTime = [0.010,0.060];    % [s]
lim_controlDelay = [0.05,0.35];           % [%]     

% Generates distributions
numberOfSamples = 3000;
rng shuffle

figure
massPercentage1 = diff(lim_massPercentage)*rand(1,numberOfSamples)+lim_massPercentage(1);
massPercentage2 = diff(lim_massPercentage)*rand(1,numberOfSamples)+lim_massPercentage(1);
edges = lim_massPercentage(1):(lim_massPercentage(2)-lim_massPercentage(1))/5:lim_massPercentage(2);
counts1 = histcounts(massPercentage1,edges);
counts2 = histcounts(massPercentage2,edges);
x = edges(1:end-1)+diff(edges)/2;
subplot(1,2,1)
bar(x,counts1,.9)
title('X_1 parameter histogram M1')
xlabel('Payload mass percentage')
ylabel('count')
subplot(1,2,2)
bar(x,counts2,.9)
title('X_1 parameter histogram M2')
xlabel('Payload mass percentage')
ylabel('count')
savefig('x1histogram.fig')

% rng shuffle
figure
endTime1 = diff(lim_endTime)*rand(1,numberOfSamples)+lim_endTime(1);
endTime2 = diff(lim_endTime)*rand(1,numberOfSamples)+lim_endTime(1);
edges = lim_endTime(1):(lim_endTime(2)-lim_endTime(1))/5:lim_endTime(2);
counts1 = histcounts(endTime1,edges);
counts2 = histcounts(endTime2,edges);
x = edges(1:end-1)+diff(edges)/2;
subplot(1,2,1)
bar(x,counts1,0.9)
title('X_2 parameter histogram M1')
xlabel('Trajectory duration (s)')
ylabel('count')
subplot(1,2,2)
bar(x,counts2,0.9)
title('X_2 parameter histogram M2')
xlabel('Trajectory duration (s)')
ylabel('count')
savefig('x2histogram.fig')

% rng shuffle
figure
disturbance1 = diff(lim_disturbance)*rand(1,numberOfSamples)+lim_disturbance(1);
disturbance2 = diff(lim_disturbance)*rand(1,numberOfSamples)+lim_disturbance(1);
edges = lim_disturbance(1):(lim_disturbance(2)-lim_disturbance(1))/5:lim_disturbance(2);
counts1 = histcounts(disturbance1,edges);
counts2 = histcounts(disturbance2,edges);
x = edges(1:end-1)+diff(edges)/2;
subplot(1,2,1)
bar(x,counts1,0.9)
title('X_3 parameter histogram M1')
xlabel('Disturbance (m/s)')
ylabel('count')
subplot(1,2,2)
bar(x,counts2,0.9)
title('X_3 parameter histogram M2')
xlabel('Disturbance (m/s)')
ylabel('count')
savefig('x3histogram.fig')

% rng shuffle
figure
coefficients1 = round(diff(lim_coefficients)*rand(1,numberOfSamples)+lim_coefficients(1));
coefficients2 = round(diff(lim_coefficients)*rand(1,numberOfSamples)+lim_coefficients(1));
edges = lim_coefficients(1):(lim_coefficients(2)-lim_coefficients(1))/2:lim_coefficients(2);
counts1 = histcounts(coefficients1,edges);
counts2 = histcounts(coefficients2,edges);
x = edges(1:end-1)+diff(edges)/2;
subplot(1,2,1)
bar(x,counts1,1)
title('X_4 parameter histogram M1')
xlabel('Variable coefficients consideration (True or False)')
ylabel('count')
subplot(1,2,2)
bar(x,counts2,1)
title('X_4 parameter histogram M2')
xlabel('Variable coefficients consideration (True or False)')
ylabel('count')
savefig('x4histogram.fig')

% rng shuffle
figure
failures1 = unidrnd(length(lim_failures),[1,numberOfSamples]);
failures2 = unidrnd(length(lim_failures),[1,numberOfSamples]);
edges = 1:1:length(lim_failures)+1;
counts1 = histcounts(failures1,edges);
counts2 = histcounts(failures2,edges);
x = edges(1:end-1)+diff(edges)/2;
subplot(1,2,1)
bar(x,counts1,1)
title('X_5 parameter histogram M1')
xlabel('Failure case')
ylabel('count')
subplot(1,2,2)
bar(x,counts2,1)
title('X_5 parameter histogram M2')
xlabel('Failure case')
ylabel('count')
savefig('x5histogram.fig')

% rng shuffle
figure
controlLoopTime1 = diff(lim_controlLoopTime)*rand(1,numberOfSamples)+lim_controlLoopTime(1);
controlLoopTime2 = diff(lim_controlLoopTime)*rand(1,numberOfSamples)+lim_controlLoopTime(1);
edges = lim_controlLoopTime(1):(lim_controlLoopTime(2)-lim_controlLoopTime(1))/5:lim_controlLoopTime(2);
counts1 = histcounts(controlLoopTime1,edges);
counts2 = histcounts(controlLoopTime2,edges);
x = edges(1:end-1)+diff(edges)/2;
subplot(1,2,1)
bar(x,counts1,1)
title('X_6 parameter histogram M1')
xlabel('Control loop time (s)')
ylabel('count')
subplot(1,2,2)
bar(x,counts2,1)
title('X_6 parameter histogram M2')
xlabel('Control loop time (s)')
ylabel('count')
savefig('x6histogram.fig')

% rng shuffle
figure
controlDelay1 = diff(lim_controlDelay)*rand(1,numberOfSamples)+lim_controlDelay(1);
controlDelay2 = diff(lim_controlDelay)*rand(1,numberOfSamples)+lim_controlDelay(1);
edges = lim_controlDelay(1):(lim_controlDelay(2)-lim_controlDelay(1))/5:lim_controlDelay(2);
counts1 = histcounts(controlDelay1,edges);
counts2 = histcounts(controlDelay2,edges);
x = edges(1:end-1)+diff(edges)/2;
subplot(1,2,1)
bar(x,counts1,1)
title('X_7 parameter histogram M1')
xlabel('Control delay percentage')
ylabel('count')
subplot(1,2,2)
bar(x,counts2,1)
title('X_7 parameter histogram M2')
xlabel('Control delay percentage')
ylabel('count')
savefig('x7histogram.fig')


N0 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N0(it,1:7) = {massPercentage2(it),endTime2(it),disturbance2(it),coefficients2(it),lim_failures{failures2(it)},controlLoopTime2(it),controlDelay2(it)};
end
N1 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N1(it,1:7) = {massPercentage1(it),endTime2(it),disturbance2(it),coefficients2(it),lim_failures{failures2(it)},controlLoopTime2(it),controlDelay2(it)};
end
N2 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N2(it,1:7) = {massPercentage2(it),endTime1(it),disturbance2(it),coefficients2(it),lim_failures{failures2(it)},controlLoopTime2(it),controlDelay2(it)};
end
N3 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N3(it,1:7) = {massPercentage2(it),endTime2(it),disturbance1(it),coefficients2(it),lim_failures{failures2(it)},controlLoopTime2(it),controlDelay2(it)};
end
N4 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N4(it,1:7) = {massPercentage2(it),endTime2(it),disturbance2(it),coefficients1(it),lim_failures{failures2(it)},controlLoopTime2(it),controlDelay2(it)};
end
N5 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N5(it,1:7) = {massPercentage2(it),endTime2(it),disturbance2(it),coefficients2(it),lim_failures{failures1(it)},controlLoopTime2(it),controlDelay2(it)};
end
N6 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N6(it,1:7) = {massPercentage2(it),endTime2(it),disturbance2(it),coefficients2(it),lim_failures{failures2(it)},controlLoopTime1(it),controlDelay2(it)};
end
N7 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N7(it,1:7) = {massPercentage2(it),endTime2(it),disturbance2(it),coefficients2(it),lim_failures{failures2(it)},controlLoopTime2(it),controlDelay1(it)};
end
N1234567 = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    N1234567(it,1:7) = {massPercentage1(it),endTime1(it),disturbance1(it),coefficients1(it),lim_failures{failures1(it)},controlLoopTime1(it),controlDelay1(it)};
end

columnNames =  {{'mass percentage (%)'}
                {'end time (s)'}
                {'disturbance (m/s)'}
                {'coefficients (true or false)'}
                {'failures'}
                {'control loop time (s)'}
                {'control delay percentage (%)'}};
save('monteCarlo_samples.mat','columnNames','N0','N1','N2','N3','N4','N5','N6','N7','N1234567')



