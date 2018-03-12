
%%%%%%%%%%%%%%%%%% Universidade Federal de S�o Carlos %%%%%%%%%%%%%%%%%%%%%
%%%%%% Autora: Isabella Cristina Souza Faria.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: isamoreno2009@gmail.com %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Orientador: Roberto Santos Inoue.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% e-mail: rsinoue@ufscar.br %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data: 20/01/2015 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure(2)
figure_1=plot(t_v,x_v,t_v,xd_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Posi��o x (m)','Fontsize',16)
leg_1 = legend('Multirotor','Desejado','Location','SouthEast')
set(leg_1,'FontSize',16)

figure(3)
plot(t_v,y_v,t_v,yd_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Posi��o y (m)','Fontsize',16)
leg_2 = legend('Multirotor','Desejado','Location','SouthEast')
set(leg_2,'Fontsize',16)

figure(4)
plot(t_v,z_v,t_v,zd_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Posi��o z (m)','Fontsize',16)
leg_3 = legend('Multirotor','Desejado','Location','SouthEast')
set(leg_3,'Fontsize',16)

figure(5)
plot3(x_v,y_v,z_v,xd_v,yd_v,zd_v,'linewidth',2)
xlabel('Posi��o x (m)','Fontsize',16)
ylabel('Posi��o y (m)','Fontsize',16)
zlabel('Posi��o z (m)','Fontsize',16)
leg_4 = legend('Multirotor','Desejado','Location','NorthEast')
set(leg_4,'Fontsize',16)
grid on

figure(6)
plot(t_v,theta_r_v,t_v,theta_r_d_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Posi��o angular de rolagem \phi (�)','Fontsize',16)
leg_5 = legend('Multirotor','Desejado','Location','SouthEast')
set(leg_5,'Fontsize',16)

figure(7)
plot(t_v,theta_p_v,t_v,theta_p_d_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Posi��o angular de arfagem \theta (�)','Fontsize',16)
leg_6 = legend('Multirotor','Desejado','Location','SouthEast')
set(leg_6,'Fontsize',16)

figure(8)
plot(t_v,theta_y_v,t_v,theta_y_d_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Posi��o angular de guinada \psi (�)','Fontsize',16)
leg_7 = legend('Multirotor','Desejado','Location','SouthEast')
set(leg_7,'Fontsize',16)

figure(9)
plot(t_v,tau_r_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Torque de rolagem \tau \phi (m)','Fontsize',16)

figure(10)
plot(t_v,tau_p_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Torque de arfagem \tau \theta (m)','Fontsize',16)

figure(11)
plot(t_v,tau_y_v,'linewidth',2)
xlabel('Tempo (s)','Fontsize',16)
ylabel('Torque de guinada \tau \psi (m)','Fontsize',16)
