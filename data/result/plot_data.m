clc; clear; close all;
%%
data = load("record.txt");
t = 0.001 : 0.001 : length(data(:, 1))*0.001;
% figure(1)
% for i = 1 : 6
%     subplot(3, 2, i)
%     plot(t, data(:, i), t, data(:, i+12))
%     legend("Command", "Record")
%     xlabel("time (s)"); ylabel("Position (rad)")
% end
% 
% figure(2)
% for i = 1 : 6
%     subplot(3, 2, i)
%     plot(t, data(:, i)-data(:, i+12))
%     legend("Error")
%     xlabel("time (s)"); ylabel("Position Error (rad)")
% end

figure(1)
for i = 1 : 6
    subplot(3, 2, i)
    plot(t, data(:, i), t, data(:, i+6), t, data(:, i+13))
    legend("Command", "Record", "Command modified")
    xlabel("time (s)"); ylabel("Position (rad)")
end

figure(2)
Force_desired = -10;
plot(t, data(:, 13), t, Force_desired*ones(length(t), 1), t, -50*ones(length(t), 1))
legend("Force ext", "Force desired", "Force origin")
xlabel("time (s)"); ylabel("Force (N)")

% figure(3)
% for i = 1 : 6
%     subplot(3, 2, i)
%     plot(t, data(:, i+13))
%     legend("Command modified")
%     xlabel("time (s)"); ylabel("Position (rad)")
% end