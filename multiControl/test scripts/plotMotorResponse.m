figure
plot(w100)
hold on
plot(w50,'r*')
hold on
plot(w10,'k.')
legend('\xi = 100 %','\xi = 50 %','\xi = 10 %')
title('Motor response to efficiency reduction')
xlabel('Time (s)')
ylabel('Rotor speed (rad/s)')
grid minor