clc; clear; close all;
%%
Data = load("record.txt");
    
PCmd = Data(:, 1:6);
VCmd = Data(:, 7:12);
P    = Data(:, 13:18);
V    = Data(:, 19:24);

t = 0.001 : 0.001 :0.001*length(Data(:, 1));

for i = 1 : length(PCmd(:, 1))
    PCmd_car(i, :) = ForwardKinematics(PCmd(i, :), 6);
    P_car(i, :) = ForwardKinematics(P(i, :), 6);
end

%% 
figure('Name', "Position")
for j = 1 : 6
    subplot(3, 2, j)
    plot(t, PCmd(:, j), t, P(:, j))
    title("Axis" + string(j))
    legend("PCmd", "P")
    xlabel("time (s)"); ylabel("position (rad)")
end

figure('Name', "Velocity")
for j = 1 : 6
    subplot(3, 2, j)
    plot(t, VCmd(:, j), t, V(:, j))
    title("Axis" + string(j))
    legend("VCmd", "V")
    xlabel("time (s)"); ylabel("velocity (rad/s)")
end

figure('Name', "Position (Cartesian)")
for j = 1 : 3
    subplot(3, 1, j)
    plot(t, PCmd_car(:, j), t, P_car(:, j))
    title("Axis" + string(j))
    legend("PCmd", "P")
    xlabel("time (s)"); ylabel("position (m)")
end