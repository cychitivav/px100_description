clc, clear, close all
run('px.m')
close all
%% ROS send command
rosshutdown
rosinit()

jointclient = rossvcclient('/dynamixel_workbench/dynamixel_command')

jointmsg = rosmessage(jointclient);
jointmsg.Id = 4;
jointmsg.AddrName = 'Goal_Position';
jointmsg.Value = 2508;

waitForServer(jointclient);
response = call(jointclient, jointmsg);
%% Get Positions
jointSub = rossubscriber('/joint_states');

% while true
%     [tf,status,~] = receive(jointSub,10);
%
%     if status
%         q = tf.Position
%
%         PX.plot(q(2:5)','noa','jaxes','notiles','floorlevel',0,'noshadow')
%         axis([-250 250 -250 250 -100 250])
%     end
% end
%% Send positions
q = [0, 0, 0, 0, 0;
    -20, 20, -20, 20, 0;
    30, -30, 30, -30, 0;
    -90, 15, -55, 17, 0;
    -90, 45, -65, 30, 20];

for i = 1:length(q)

    for j = 1:5
        jointmsg = rosmessage(jointclient);
        jointmsg.AddrName = 'Goal_Position';
        jointmsg.Id = j;
        jointmsg.Value = round(q(i, j) / 360 * 4095 + 2048);

        waitForServer(jointclient);
        response = call(jointclient, jointmsg);

        [tf, status, ~] = receive(jointSub);

        if status
            PX.plot(tf.Position([4 3 1 5])', 'noa', 'jaxes', 'notiles', 'floorlevel', 0, 'noshadow')
            axis([-250 250 -250 250 -100 250])
            view([30 30])
        end

    end

    pause(2);
end
