% Buttons settings
set(gui.pushbutton_waypoint,'Callback','pushbutton_waypoint_f')
set(gui.edit_posxd,'Callback','position_display_f')
set(gui.edit_posyd,'Callback','position_display_f')
set(gui.pushbutton_plot_gui,'Callback','plot_gui_graphics_f')
set(gui.pushbutton_plot,'Callback','plot_graphics_f')
set(gui.pushbutton_save_plot,'Callback','save_graphics_f')
set(gui.pushbutton_start,'Callback','experiment_f')
set(gui.pushbutton_stop,'Callback','flag_exp = 0;')

% Axes settings
set(gcf,'CurrentAxes',gui.axes1)
plot3(0,0,0)
dist = 3;
set(gui.axes1,'Xlim',[-dist dist],'Ylim',[-dist dist],'Zlim',[0 dist])
hold on;
grid;

%% Object of the quadrotor in the Axes 1
[quad1_obj] = load_object_f;
[quad1_direcao]= load_inertial_vector;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(gcf,'CurrentAxes',gui.axes2)
dist = 3;
set(gui.axes2,'Xlim',[-dist dist],'Ylim',[-dist dist],'Zlim',[0 dist])
hold on;
grid;

%% Object of the quadrotor in the Axes 2
[multirotor_obj] = load_object_f;
[multirotor_direcao]= load_inertial_vector;