tspan = [0:20]
y0 = [2 0];

sol = ode45(@vdp1,tspan,y0);
t = sol.x
output = sol.y';
size(t)
size(output)
figure
plot(t,output)

sol_new = odextend(sol,@vdp1,35);
t = sol_new.x;
output = sol_new.y';
size(t)
size(output)
figure
plot(t,output)