x=0;
y=0;
z=0;
t = 0;
xf=str2double(get(gui.edit_posxd,'string'));
yf=str2double(get(gui.edit_posyd,'string'));
zf=str2double(get(gui.edit_poszd,'string'));

dz_d=0;

s=[x;y;z];
sf=[xf;yf;zf];
ds=[0;0;0];
d2s=[0;0;0];
dsf=[0;0;0];
d2sf=[0;0;0];
s_ant = [x; y; z];

dz=0;
dy=0;
dx=0;
ds_ant = [dx;dy;dz];
d2x=0;
d2y=0;
d2z=0;

dtheta_r_d=0;
dtheta_r=0;
theta_r_d=0;
theta_r=0;
dtheta_p_d=0;
dtheta_p=0;
theta_p_d=0;
theta_p=0;
dtheta_y_d=0;
dtheta_y=0;
theta_y_d=pi/180*str2double(get(gui.edit_yawd,'string'));
theta_y=0;
d2theta_r=0;
d2theta_p=0;
d2theta_y=0;
dn_ant=[dtheta_r;dtheta_p;dtheta_y];
n_ant=[theta_r;theta_p;theta_y];

set(gui.edit_time,'string',t)

set(gui.edit_posx,'string',x)
set(gui.edit_posy,'string',y)
set(gui.edit_posz,'string',z)

set(gui.edit_roll,'string',theta_r*180/pi)
set(gui.edit_pitch,'string',theta_p*180/pi)
set(gui.edit_yaw,'string',theta_y*180/pi)