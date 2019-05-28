%% Markov sample generator for controller evaluation
% Defines limits
lim_massPercentage = [0,1];     % [%]
lim_endTime = [5,20];           % [s]
lim_disturbance = [0,45];       % [m/s2]
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
            {'setRotorStatus(1,''prop loss'',0.78)'}
            {'setRotorStatus(1,''prop loss'',0.10)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.78)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(3,''prop loss'',0.78)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(3,''prop loss'',0.10)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(3,''prop loss'',0.10)','setRotorStatus(4,''prop loss'',0.78)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(3,''prop loss'',0.10)','setRotorStatus(4,''prop loss'',0.10)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(7,''prop loss'',0.10)','setRotorStatus(8,''prop loss'',0.78)'}
            {'setRotorStatus(1,''prop loss'',0.10)','setRotorStatus(2,''prop loss'',0.10)','setRotorStatus(7,''prop loss'',0.10)','setRotorStatus(8,''prop loss'',0.10)'}};
lim_controlLoopTime = [0.020,0.060];    % [s]
lim_controlDelay = [0.2,0.8];           % [%]     

% Generates distributions
numberOfSamples = 1200;
% rng shuffle
figure
massPercentage = diff(lim_massPercentage)*rand(1,numberOfSamples)+lim_massPercentage(1);
edges = lim_massPercentage(1):(lim_massPercentage(2)-lim_massPercentage(1))/5:lim_massPercentage(2);
counts = histcounts(massPercentage,edges);
x = edges(1:end-1)+diff(edges)/2;
bar(x,counts,1)
title('X_1 parameter histogram')
xlabel('Payload mass percentage')
ylabel('count')
savefig('x1histogram.fig')

% rng shuffle
figure
endTime = diff(lim_endTime)*rand(1,numberOfSamples)+lim_endTime(1);
edges = lim_endTime(1):(lim_endTime(2)-lim_endTime(1))/5:lim_endTime(2);
counts = histcounts(disturbance,edges);
x = edges(1:end-1)+diff(edges)/2;
bar(x,counts,1)
title('X_2 parameter histogram')
xlabel('Trajectory duration (s)')
ylabel('count')
savefig('x2histogram.fig')

% rng shuffle
figure
disturbance = diff(lim_disturbance)*rand(1,numberOfSamples)+lim_disturbance(1);
edges = lim_disturbance(1):(lim_disturbance(2)-lim_disturbance(1))/5:lim_disturbance(2);
counts = histcounts(disturbance,edges);
x = edges(1:end-1)+diff(edges)/2;
bar(x,counts,1)
title('X_3 parameter histogram')
xlabel('Disturbance (m/s)')
ylabel('count')
savefig('x3histogram.fig')

% rng shuffle
figure
coefficients = round(diff(lim_coefficients)*rand(1,numberOfSamples)+lim_coefficients(1));
edges = lim_coefficients(1):(lim_coefficients(2)-lim_coefficients(1))/2:lim_coefficients(2);
counts = histcounts(coefficients,edges);
x = edges(1:end-1)+diff(edges)/2;
bar(x,counts,1)
title('X_4 parameter histogram')
xlabel('Variable coefficients consideration (True or False)')
ylabel('count')
savefig('x4histogram.fig')

% rng shuffle
figure
failures = unidrnd(length(lim_failures),[1,numberOfSamples]);
edges = 1:1:length(lim_failures)+1;
counts = histcounts(failures,edges);
x = edges(1:end-1)+diff(edges)/2;
bar(x,counts,1)
title('X_5 parameter histogram')
xlabel('Failure case')
ylabel('count')
savefig('x5histogram.fig')

% rng shuffle
figure
controlLoopTime = diff(lim_controlLoopTime)*rand(1,numberOfSamples)+lim_controlLoopTime(1);
edges = lim_controlLoopTime(1):(lim_controlLoopTime(2)-lim_controlLoopTime(1))/5:lim_controlLoopTime(2);
counts = histcounts(controlLoopTime,edges);
x = edges(1:end-1)+diff(edges)/2;
bar(x,counts,1)
title('X_6 parameter histogram')
xlabel('Control loop time (s)')
ylabel('count')
savefig('x6histogram.fig')

% rng shuffle
figure
controlDelay = diff(lim_controlDelay)*rand(1,numberOfSamples)+lim_controlDelay(1);
edges = lim_controlDelay(1):(lim_controlDelay(2)-lim_controlDelay(1))/5:lim_controlDelay(2);
counts = histcounts(controlDelay,edges);
x = edges(1:end-1)+diff(edges)/2;
bar(x,counts,1)
title('X_7 parameter histogram')
xlabel('Control delay percentage')
ylabel('count')
savefig('x7histogram.fig')

samples = cell(numberOfSamples,7);
for it = 1:numberOfSamples
    samples(it,1:7) = {massPercentage(it),endTime(it),disturbance(it),coefficients(it),lim_failures{failures(it)},controlLoopTime(it),controlDelay(it)};
end
columnNames =  {{'mass percentage (%)'}
                {'end time (s)'}
                {'disturbance (m/s)'}
                {'coefficients (true or false)'}
                {'failures'}
                {'control loop time (s)'}
                {'control delay percentage (%)'}};
save('monteCarlo_samples.mat','columnNames','samples')



