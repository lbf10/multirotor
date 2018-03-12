function [waypoints, time] = geronoToWaypoints(comprimento, width, height, endtime, timestep, yawtype, varargin)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    a = comprimento/2;
    b = width;
    c = height/2;
    x = @(t) -a*cos(2*pi*t/endtime)+a;
    y = @(t) -(b/2)*sin(4*pi*t/endtime);
    z = @(t) c*(1-cos(2*pi*t/endtime));
    dx = @(t) (2*pi*a/endtime)*sin(2*pi*t/endtime);
    dy = @(t) -(2*pi*b/endtime)*cos(4*pi*t/endtime);
    dz = @(t) c*(2*pi/endtime)*sin(2*pi*t/endtime);
    %vel = @(t) sqrt(velx(t).^2+vely(t).^2+velz(t).^2);
    d2x = @(t) a*(2*pi/endtime)^2*cos(2*pi*t/endtime);
    d2y = @(t) (8*pi*pi*b/(endtime^2))*sin(4*pi*t/endtime);
    d2z = @(t) c*(2*pi/endtime)^2*cos(2*pi*t/endtime);
    %acc = @(t) sqrt(accx(t).^2+accy(t).^2+accz(t).^2);
    
    switch yawtype
        case 'fixed'
            yaw = @(t) varargin{1}*ones(1,length(t));
            dyaw = @(t) zeros(1,length(t));
            d2yaw = @(t) zeros(1,length(t));
        case '360'
            yaw = @(t) pi*(1-cos(pi*t/endtime));
            dyaw = @(t) pi*(pi/endtime)*sin(pi*t/endtime);
            d2yaw = @(t) pi*(pi/endtime)^2*cos(pi*t/endtime);
        case 'goto'
            yaw = @(t) varargin{1}*(1-cos(pi*t/endtime))/2;
            dyaw = @(t) varargin{1}*(pi/endtime)*sin(pi*t/endtime)/2;
            d2yaw = @(t) varargin{1}*(pi/endtime)^2*cos(pi*t/endtime)/2;
        case 'sinusoidal'
            yaw = @(t) varargin{2}*sin(2*pi*t/varargin{3})+varargin{1};
            dyaw = @(t) varargin{2}*(2*pi/varargin{3})*cos(2*pi*t/varargin{3});
            d2yaw = @(t) -varargin{2}*(2*pi/varargin{3})^2*sin(2*pi*t/varargin{3});     
    end
    t = timestep:timestep:endtime;

    position = [x(t);y(t);z(t)];
    velocity = [dx(t(1:end-1)) 0;dy(t(1:end-1)) 0;dz(t(1:end-1)) 0];
    acceleration = [d2x(t(1:end-1)) 0;d2y(t(1:end-1)) 0;d2z(t(1:end-1)) 0];
    
    waypoints = [position;velocity;acceleration;yaw(t);dyaw(t(1:end-1)) 0;d2yaw(t(1:end-1)) 0];
    time = t;  
end

