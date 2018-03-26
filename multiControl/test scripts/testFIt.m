% test fit
speed = [0
        416.5751859
        435.2676622
        462.5052705
        472.6526147
        491.345091
        501.4924353
        520.1849116
        530.3322559
        549.0247321
        567.7172084
        586.4096847
        748.2865294
        1000];
    
liftCoeff = [0
            0.00009663400821486720
            0.00010197039400480800
            0.00010177480503994200
            0.00010886498777293000
            0.00011048831185009000
            0.00011230119869840700
            0.00010908666646728400
            0.00011227432775784800
            0.00010996476733082600
            0.00010862374599149600
            0.00010409054272222600
            0.00006567742093581670
            0];
        
 
        
 dragCoeff = [0
            0.00000115158401406177
            0.00000131849846466781
            0.00000140132963964922
            0.00000156543968817590
            0.00000165553807692624
            0.00000178787094426600
            0.00000184631980295481
            0.00000195397512083756
            0.00000198893164777812
            0.00000201512348657737
            0.00000203398711313428
            0.00000136514255905061
            0];
    
 figure
 plot(speed, liftCoeff)
 hold on
 f = fit(speed, liftCoeff, 'smoothingspline') 
 plot(min(speed):10:max(speed),f(min(speed):10:max(speed)))
 legend('Sem fit','Com fit')
 
  figure
 plot(speed, dragCoeff)
 hold on
 f = fit(speed, dragCoeff, 'smoothingspline') 
 plot(min(speed):10:max(speed),f(min(speed):10:max(speed)))
 legend('Sem fit','Com fit')
 
%   figure
%  f = fit(speed, liftCoeff, 'linear') 
%  speeds = min(speed):10:max(speed);
%  plot(min(speed):10:max(speed),8*f(min(speed):10:max(speed)).*(speeds.^2)')
