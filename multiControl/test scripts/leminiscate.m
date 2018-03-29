% plot lemniscate of gerono
% x = @(t) (t.^2-1)./(t.^2+1);
% y = @(t) 2*t.*((t.^2-1)./(t.^2+1).^2);
% 
% t = -1000:0.1:1000;
comprimento = 7;
largura = 4;
altura = 4;
tmax = 12;


a = comprimento/2;
b = largura;
c = altura/2;
x = @(t) -a*cos(2*pi*t/tmax)+a;
y = @(t) -(b/2)*sin(4*pi*t/tmax);
z = @(t) c*(1-cos(2*pi*t/tmax));
velx = @(t) (2*pi*a/tmax)*sin(2*pi*t/tmax);
vely = @(t) -(2*pi*b/tmax)*cos(4*pi*t/tmax);
velz = @(t) c*(2*pi/tmax)*sin(2*pi*t/tmax);
vel = @(t) sqrt(velx(t).^2+vely(t).^2+velz(t).^2);
accx = @(t) a*(2*pi/tmax)^2*cos(2*pi*t/tmax);
accy = @(t) (8*pi*pi*b/(tmax^2))*sin(4*pi*t/tmax);
accz = @(t) c*(2*pi/tmax)^2*cos(2*pi*t/tmax);
acc = @(t) sqrt(accx(t).^2+accy(t).^2+accz(t).^2);
t = 0:0.001:tmax;

% figure
% plot3(x(t),y(t),z(t))
% title('Posi��o em X, Y e Z')
% daspect([1 1 1])
% grid on
% figure
% plot(t,x(t))
% title('Posicoes')
% hold on
% plot(t,y(t))
% hold on
% plot(t,z(t))
% legend('x','y','z')
figure
plot(t,vel(t))
title('Velocidade total')
% figure
% plot(t,velx(t))
% title('Velocidades')
% hold on
% plot(t,vely(t))
% hold on
% plot(t,velz(t))
% legend('x','y','z')
figure
plot(t,acc(t))
title('Aceleracao total')
% figure
% plot(t,accx(t))
% title('Aceleracoes')
% hold on
% plot(t,accy(t))
% hold on
% plot(t,accz(t))
% legend('x','y','z')