data = load('monteCarlo_samples.mat');
M1 = data.N1234567;
M2 = data.N0;
massPercentage1 = [M1{:,1}];
massPercentage2 = [M2{:,1}];
endTime1 = [M1{:,2}];
endTime2 = [M2{:,2}];
disturbance1 = [M1{:,3}];
disturbance2 = [M2{:,3}];
coefficients1 = [M1{:,4}];
coefficients2 = [M2{:,4}];
controlLoopTime1 = [M1{:,6}];
controlLoopTime2 = [M2{:,6}];
controlDelay1 = [M1{:,7}];
controlDelay2 = [M2{:,7}];

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
failures1 = [];
for it=1:length(M1)
    for jt=1:length(lim_failures)
        if isequal(size(M1{it,5}),size(lim_failures{jt}))
            if all(strcmp(M1{it,5},lim_failures{jt}))
                failures1(it) = jt;
            end
        end
    end
end
failures2 = [];
for it=1:length(M2)
    for jt=1:length(lim_failures)
        if isequal(size(M2{it,5}),size(lim_failures{jt}))
            if all(strcmp(M2{it,5},lim_failures{jt}))
                failures2(it) = jt;
            end
        end
    end
end
figure
subplot(1,2,1)
histogram(massPercentage1)
title('X_1 parameter histogram M1')
xlabel('Payload mass percentage')
ylabel('count')
subplot(1,2,2)
histogram(massPercentage2)
title('X_1 parameter histogram M2')
xlabel('Payload mass percentage')
ylabel('count')
savefig('x1histogram.fig')

% rng shuffle
figure
subplot(1,2,1)
histogram(endTime1)
title('X_2 parameter histogram M1')
xlabel('Trajectory duration (s)')
ylabel('count')
subplot(1,2,2)
histogram(endTime2)
title('X_2 parameter histogram M2')
xlabel('Trajectory duration (s)')
ylabel('count')
savefig('x2histogram.fig')

% rng shuffle
figure
subplot(1,2,1)
histogram(disturbance1)
title('X_3 parameter histogram M1')
xlabel('Disturbance (m/s)')
ylabel('count')
subplot(1,2,2)
histogram(disturbance2)
title('X_3 parameter histogram M2')
xlabel('Disturbance (m/s)')
ylabel('count')
savefig('x3histogram.fig')

% rng shuffle
figure
subplot(1,2,1)
histogram(coefficients1)
title('X_4 parameter histogram M1')
xlabel('Variable coefficients consideration (True or False)')
ylabel('count')
subplot(1,2,2)
histogram(coefficients2)
title('X_4 parameter histogram M2')
xlabel('Variable coefficients consideration (True or False)')
ylabel('count')
savefig('x4histogram.fig')

% rng shuffle
figure
subplot(1,2,1)
histogram(failures1)
title('X_5 parameter histogram M1')
xlabel('Failure case')
ylabel('count')
subplot(1,2,2)
histogram(failures2)
title('X_5 parameter histogram M2')
xlabel('Failure case')
ylabel('count')
savefig('x5histogram.fig')

% rng shuffle
figure
subplot(1,2,1)
histogram(controlLoopTime1)
title('X_6 parameter histogram M1')
xlabel('Control loop time (s)')
ylabel('count')
subplot(1,2,2)
histogram(controlLoopTime2)
title('X_6 parameter histogram M2')
xlabel('Control loop time (s)')
ylabel('count')
savefig('x6histogram.fig')

% rng shuffle
figure
subplot(1,2,1)
histogram(controlDelay1)
title('X_7 parameter histogram M1')
xlabel('Control delay percentage')
ylabel('count')
subplot(1,2,2)
histogram(controlDelay2)
title('X_7 parameter histogram M2')
xlabel('Control delay percentage')
ylabel('count')
savefig('x7histogram.fig')

