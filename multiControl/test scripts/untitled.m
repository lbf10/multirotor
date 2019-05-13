PIdata = load('simulation_results_PIPassive_canonical.mat');
RPIdata = load('simulation_results_RPIPassive_canonical.mat');
NMACdata = load('simulation_results_NMACPassive_canonical.mat');

figure
plot3(PIdata.trajectory_.position(1,:),PIdata.trajectory_.position(2,:),PIdata.trajectory_.position(3,:),'k--')
hold on
plot3(PIdata.log_.position(1,:),PIdata.log_.position(2,:),PIdata.log_.position(3,:),'r')
hold on
plot3(RPIdata.log_.position(1,:),RPIdata.log_.position(2,:),RPIdata.log_.position(3,:),'g')
hold on
plot3(NMACdata.log_.position(1,:),NMACdata.log_.position(2,:),NMACdata.log_.position(3,:),'b')
daspect([1 1 1])
grid minor
title('3D Position');
xlabel('X Position [m]');
ylabel('Y Position [m]');
zlabel('Z Position [m]');
legend('Desired trajectory','PI Passive','RPI Passive','Null-space Passive');

figure
plot(PIdata.log_.time,PIdata.log_.rotor(1).speed,'r')
hold on
plot(RPIdata.log_.time,RPIdata.log_.rotor(1).speed,'g')
hold on
plot(NMACdata.log_.time,NMACdata.log_.rotor(1).speed,'b')
title('Rotor speeds')
xlabel('Time (s)')
ylabel('Speed (rad/s)')
legend('PI Passive','RPI Passive','Null-space Passive')

%% 
PIdata = load('simulation_results_PIPassive.mat');
RPIdata = load('simulation_results_RPIPassive.mat');
NMACdata = load('simulation_results_NMACPassive.mat');

figure
plot3(NMACdata.trajectory_.position(1,:),NMACdata.trajectory_.position(2,:),NMACdata.trajectory_.position(3,:),'k--')
hold on
plot3(PIdata.log_.position(1,:),PIdata.log_.position(2,:),PIdata.log_.position(3,:),'r')
hold on
plot3(RPIdata.log_.position(1,:),RPIdata.log_.position(2,:),RPIdata.log_.position(3,:),'g')
hold on
plot3(NMACdata.log_.position(1,:),NMACdata.log_.position(2,:),NMACdata.log_.position(3,:),'b')
daspect([1 1 1])
grid minor
title('3D Position');
xlabel('X Position [m]');
ylabel('Y Position [m]');
zlabel('Z Position [m]');
legend('Desired trajectory','PI Passive','RPI Passive','Null-space Passive');

figure
plot(PIdata.log_.time,PIdata.log_.rotor(1).speed,'r')
hold on
plot(RPIdata.log_.time,RPIdata.log_.rotor(1).speed,'g')
hold on
plot(NMACdata.log_.time,NMACdata.log_.rotor(1).speed,'b')
title('Rotor speeds')
xlabel('Time (s)')
ylabel('Speed (rad/s)')
legend('PI Passive','RPI Passive','Null-space Passive')