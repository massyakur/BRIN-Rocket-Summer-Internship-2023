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

pitchDREF = 'sim/cockpit2/gauges/indicators/pitch_AHARS_deg_pilot';
% pitchDREF = 'sim/cockpit2/gauges/indicators/pitch_electric_deg_pilot';
pitch = getDREFs(pitchDREF, Socket);

rollDREF = 'sim/cockpit2/gauges/indicators/roll_AHARS_deg_pilot';
% rollDREF = '	sim/cockpit2/gauges/indicators/roll_electric_deg_pilot';
roll = getDREFs(rollDREF, Socket);


fprintf(['initial values:\n' ...
    'Altitude: %4f m\n' ...
    'Ver. Velocity: %4f m/s\n' ...
    'Pitch Angle: %4f\n' ...
    'Roll Angle: %4f\n\n'], ...
    altitude, vertical_velocity, pitch, roll);

% increase throttle
disp('increasing the throttle\n')
throttle = 1;
controlToSend = [0, 0, 0, throttle, 0, 0];
sendCTRL(controlToSend, 0, Socket);

% time,  altitude, velocity, pitch, roll matrices
dt = 0.1;
end_t = 20;
t = 0:dt:end_t;
throttle_graph = zeros(size(t));
altitude_graph = zeros(size(t));
ver_vel_graph = zeros(size(t));
pitch_graph = zeros(size(t));
roll_graph = zeros(size(t));

altitude_graph(1) = 0;
ver_vel_graph(1) = 0;

% desired values
des_altitude = 10;
des_velocity = 5;
des_pitch = 0;
des_roll = 0;

is_falling = false;
max_throttle_height = 50.0;
for i = 2:length(t)
    try
        veloDREF = 'sim/cockpit2/gauges/indicators/vvi_fpm_pilot';
        vertical_velocity = getDREFs(veloDREF, Socket) * 0.00508;

        altiDREF = 'sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot';
        altitude = getDREFs(altiDREF, Socket) * 0.3048;

        pitchDREF = 'sim/cockpit2/gauges/indicators/pitch_AHARS_deg_pilot';
        pitch = getDREFs(pitchDREF, Socket);
        p_rateDREF = 'sim/flightmodel/position/Q';
        pitch_rate = getDREFs(p_rateDREF, Socket);
        
        rollDREF = 'sim/cockpit2/gauges/indicators/roll_AHARS_deg_pilot';
        roll = getDREFs(rollDREF, Socket);
        r_rateDREF = 'sim/flightmodel/position/P';
        roll_rate = getDREFs(r_rateDREF, Socket);

        

        altitude_graph(i) = altitude;
        ver_vel_graph(i) = vertical_velocity;
        pitch_graph(i) = pitch;
        roll_graph(i) = roll;

        fprintf(['initial values:\n' ...
        'Altitude: %4f m\n' ...
        'Ver. Velocity: %4f m/s\n' ...
        'Pitch Angle: %4f\n' ...
        'Roll Angle: %4f\n\n'], ...
        altitude, vertical_velocity, pitch, roll);

    catch
        altitude = 0;
        vertical_velocity = 0;
        altitude_graph(i) = altitude_graph(i-1);
        ver_vel_graph(i) = ver_vel_graph(i-1);
        fprintf('error\n');
    end

    if ~is_falling && altitude > max_throttle_height
        is_falling = true;
        throttle = 0;
        %controlToSend = [0, 0, 0, throttle, 0, 0];
        %sendCTRL(controlToSend, 0, Socket);
    end

    altitude_err = abs(des_altitude - altitude);
    pitch_err = des_pitch - pitch;
    roll_err = des_roll - roll;

    %Pa = 1;
    Pp = 0.06;
    Dp = 0.001;
    
    Pr = 0.02;
    Dr = 0.002;

    if is_falling && altitude < max_throttle_height
        throttle = exp(42.65-0.09*altitude);
        %throttle = max_throttle_height/altitude;
        %throttle = 1;  
        if abs(vertical_velocity) < 5
            throttle = 0;
            gearDREF = 'sim/cockpit/switches/gear_handle_status';
            sendDREF(gearDREF, 1, Socket);
            is_falling = false;
        end
        Pp = 0.04;
        Dp = 0.0006;
    end
    
    elevator = pitch_err*Pp + pitch_rate*Dp;
    if elevator > 1
        elevator = 1;
    end
    aileron = roll_err*Pr + roll_rate*Dr;
    if aileron > 1
        aileron = 1;
    end
    if throttle > 1
        throttle = 1;
    end

    control_values = [elevator, aileron, 0, throttle, 0, 0];
    sendCTRL(control_values, 0, Socket);
    
    throttle_graph(i) = throttle;

    pause(dt);
    crash = getDREFs('sim/flightmodel2/misc/has_crashed', Socket);
    if crash
        disp('crash');
        break
    end
end

% Exit
closeUDP(Socket);
disp('end');

% graph for the altitude vs time
subplot(3, 2, 1);
plot(t, altitude_graph)
grid on;
ylim([-5, 80])
xlim([0, end_t])
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Above Ground vs Time');

% graph for the vertical velocity vs time
subplot(3, 2, 2);
plot(t, ver_vel_graph)
grid on;
ylim([-40, 40])
xlim([0, end_t])
xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Vertical Velocity vs Time');

% graph for the pitch angle vs time
subplot(3, 2, 3);
plot(t, pitch_graph)
grid on;
ylim([-10, 10])
xlim([0, end_t])
xlabel('Time (s)');
ylabel('Pitch Angle (degrees)');
title('Pitch Angle vs Time');

% graph for the roll angle vs time
subplot(3, 2, 4);
plot(t, roll_graph)
grid on;
ylim([-10, 10])
xlim([0, end_t])
xlabel('Time (s)');
ylabel('Roll Angle (degrees)');
title('Roll Angle vs Time');

% graph for the throttle vs time
subplot(3, 2, [5 6]);
plot(t, throttle_graph)
grid on;
ylim([-0.2, 1.2])
xlim([0, end_t])
xlabel('Time (s)');
ylabel('Throttle');
title('Throttle vs Time');

