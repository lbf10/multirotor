% test fit    
thrust = 0.8*[0
    0.5
        1.48
        1.71
        1.97
        2.22
        2.48
        2.72
        2.88
        3.01
        3.22
        3.38
        3.57
        3.65
        3.75];
    
speedThrust = [0
    250
404.3449656656
416.575185866
435.2676621549
462.5052704615
472.6526147326
491.3450910214
501.4924352925
520.1849115814
530.3322558525
549.0247321414
567.7172084302
586.4096847191
748.2865293806];
    
speed = [0
        200
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
        1511];
liftCoeff = [0.00004
            0.00007
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
 dragCoeff = [0.0000005
            0.00000075
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
            0.0000005];
    
 figure
 plot(speed, liftCoeff,'r*')
 hold on
 f = fit(speed, liftCoeff, 'smoothingspline') 
 plot(min(speed):10:max(speed),f(min(speed):10:max(speed)),'b')
 legend('Raw data','Spline fitting')
 title('Lift coefficient curve for Model 2')
 xlabel('Rotor speed (rad/s)')
 ylabel('Lift coeff (N s²)')
 grid minor
 
  figure
 plot(speed, dragCoeff,'r*')
 hold on
 f = fit(speed, dragCoeff, 'smoothingspline') 
 plot(min(speed):10:max(speed),f(min(speed):10:max(speed)),'b')
 legend('Raw data','Spline fitting')
 title('Drag coefficient curve for Model 2')
 xlabel('Rotor speed (rad/s)')
 ylabel('Drag coeff (Nm s²)')
 grid minor
%   figure
%  f = fit(speed, liftCoeff, 'linear') 
%  speeds = min(speed):10:max(speed);
%  plot(min(speed):10:max(speed),8*f(min(speed):10:max(speed)).*(speeds.^2)')


%  figure
%  plot(speedThrust, thrust)
%  hold on
%  f = fit(speedThrust, thrust, 'smoothingspline') 
%  plot(min(speedThrust):10:max(speedThrust),f(min(speedThrust):10:max(speedThrust)))
%  legend('Sem fit','Com fit')
