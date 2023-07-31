% testing throttle and measure the altitude and the vertical velocity
clear
clc
% import XPC
addpath('../')
import XPlaneConnect.*
pause(10);
% setup
Socket = openUDP('127.0.0.1', 49009);

% check connection
getDREFs('sim/test/test_float', Socket);
disp('communication connected')

veloDREF = 'sim/cockpit2/gauges/indicators/vvi_fpm_pilot';
vertical_velocity = getDREFs(veloDREF, Socket) * 0.00508;

altiDREF = 'sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot';
altitude = getDREFs(altiDREF, Socket) * 0.3048;

fprintf(['initial values:\n' ...
    'altitude_ft_pilot: %4f m\n' ...
    'Ver. Velocity: %4f m/s\n\n'], ...
    altitude, vertical_velocity);

% increase throttle
disp('increasing the throttle\n')
controlToSend = [0, 0, 0, 1, 0, 0];
sendCTRL(controlToSend, 0, Socket);

t = 0:0.1:30;
altitude_graph = zeros(size(t));
ver_vel_graph = zeros(size(t));
altitude_graph(1) = 0;
ver_vel_graph(1) = 0;

for i = 2:length(t)
    try
        veloDREF = 'sim/cockpit2/gauges/indicators/vvi_fpm_pilot';
        vertical_velocity = getDREFs(veloDREF, Socket) * 0.00508;
        
        altiDREF = 'sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot';
        altitude = getDREFs(altiDREF, Socket) * 0.3048;

        altitude_graph(i) = altitude;
        ver_vel_graph(i) = vertical_velocity;
        fprintf(['altitude_ft_pilot: %4f m\n' ...
        'Ver. Velocity: %4f m/s\n'], ...
        altitude, vertical_velocity);

    catch
        altitude = 0;
        vertical_velocity = 0;
        altitude_graph(i) = altitude_graph(i-1);
        ver_vel_graph(i) = ver_vel_graph(i-1);
        fprintf('error\n');
    end
    if altitude > 500.0
        controlToSend = [0, 0, 0, 0, 0, 0];
        sendCTRL(controlToSend, 0, Socket);
    end
    pause(0.1);
end

% Exit
closeUDP(Socket);
disp('end');

% graph for the altitude vs time
subplot(2, 1, 1);
plot(t, altitude_graph)
grid on;
ylim([-10, 800])
xlim([0, 30])
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Above Ground vs Time');

% graph for the vertical velocity vs time
subplot(2, 1, 2);
plot(t, ver_vel_graph)
grid on;
ylim([-100, 100])
xlim([0, 30])
xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Vertical Velocity vs Time');
