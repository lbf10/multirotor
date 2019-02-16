octanormal = load('/home/lbf10/multirotor/NMAC for different models/Model 3 R=1 Q=0 rotating/simulation.mat');
octaomni = load('/home/lbf10/multirotor/NMAC for different models/Model 3 R=1 Q=1.2e12 rotating/simulation.mat');

multirotor = multicontrol(4);
octanormalAttitude = 180.*multirotor.toEuler(octanormal.log_.attitude)./pi;
octaomniAttitude = 180.*multirotor.toEuler(octaomni.log_.attitude)./pi;
figure
subplot(3,1,1)
plot(octanormal.log_.time,octanormalAttitude(1,:))
title('Row')
ylabel('(\circ)')
xlim([0,22])
grid on
hold on
plot(octaomni.log_.time,octaomniAttitude(1,:))
subplot(3,1,2)
plot(octanormal.log_.time,octanormalAttitude(2,:))
title('Pitch')
ylabel('(\circ)')
xlim([0,22])
grid on
hold on
plot(octaomni.log_.time,octaomniAttitude(2,:))
subplot(3,1,3)
plot(octanormal.log_.time,octanormalAttitude(3,:))
title('Yaw')
ylabel('(\circ)')
xlabel('Time (s)')
xlim([0,22])
grid on
hold on
plot(octaomni.log_.time,octaomniAttitude(3,:))
legend('W_m = 1 and W_a = 0', 'W_m = 1 and W_a = 1.2^{11}')

figure
plot3(octanormal.trajectory_.position(1,:),octanormal.trajectory_.position(2,:),octanormal.trajectory_.position(3,:),'b')
hold on
plot3(octanormal.log_.position(1,:),octanormal.log_.position(2,:),octanormal.log_.position(3,:),'r')
hold on
plot3(octaomni.log_.position(1,:),octaomni.log_.position(2,:),octaomni.log_.position(3,:),'k')
grid on
title('3D Trajectory')
xlabel('X Position (m)')
ylabel('Y Position (m)')
zlabel('Z Position (m)')
legend('Reference','W_m = 1 and W_a = 0', 'W_m = 1 and W_a = 1.2^{11}')
