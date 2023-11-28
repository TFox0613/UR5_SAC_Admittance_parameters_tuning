clc; clear; close all;
%%
% Data = load("command_joint.txt");

% for i = 1 : 8
%     PCmd(:, :, i) = Data(:, 6*(i-1)+1:6*(i-1)+1+5);
% end
% 
% t = 0.001 : 0.001 :0.001*length(Data(:, 1));
% 
% for i = 1 : 8
%     figure('Name', "Solution "+string(i))
%     for j = 1 : 6
%         subplot(3, 2, j)
%         plot(t, PCmd(:, j, i))
%         title("Axis" + string(j))
%         xlabel("time (s)"); ylabel("position (rad)")
%     end
% end

%%
Data = load("command.txt");
num_pieces = 8;
j = 1;

for i = 1:num_pieces
    
    PCmd(:, :, i) = Data(:, 18*(i-1)+1:18*(i-1)+6);
    VCmd(:, :, i) = Data(:, 18*(i-1)+7:18*(i-1)+12);
    ACmd(:, :, i) = Data(:, 18*(i-1)+13:18*(i-1)+18);

    dPCmd(:, :, i)  = (PCmd(2:end, :, i) - PCmd(1:end-1, :, i)) / 0.001;
    ddPCmd(:, :, i) = (dPCmd(2:end, :, i) - dPCmd(1:end-1, :, i)) / 0.001;
end

t = 0.001 : 0.001 :0.001*length(Data(:, 1));

for i = 1 : 8
    figure('Name', "Solution "+string(i))
    for j = 1 : 6
        subplot(3, 2, j)
        plot(t, PCmd(:, j, i))
        title("Axis" + string(j))
        xlabel("time (s)"); ylabel("position (rad)")
    end
end

for i = 1 : 8
    figure('Name', "Solution "+string(i))
    for j = 1 : 6
        subplot(3, 2, j)
        plot(t, VCmd(:, j, i), t(1:end-1), dPCmd(:, j, i))
        title("Axis" + string(j))
        legend("VCmd", "dPCmd")
        xlabel("time (s)"); ylabel("velocity (rad/s)")
    end
end

for i = 1 : 8
    figure('Name', "Solution "+string(i))
    for j = 1 : 6
        subplot(3, 2, j)
        plot(t, ACmd(:, j, i), t(1:end-2), ddPCmd(:, j, i))
        title("Axis" + string(j))
        legend("ACmd", "ddPCmd")
        xlabel("time (s)"); ylabel("acceleration (rad/s^2)")
    end
end