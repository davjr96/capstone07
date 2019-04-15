ipaddress = "192.168.8.250";
rosinit(ipaddress);

z = 'bot 2'
vicon_sub_2 = rossubscriber('/vicon/turtlebot_2/turtlebot_2');
vicon_data_2 = receive(vicon_sub_2, 1);
vicon_data_2.Transform.Translation.X
vicon_data_2.Transform.Translation.Y
% vicon_data_2.Transform.Rotation.Z

x = 'bot 5'
vicon_sub_5 = rossubscriber('/vicon/turtlebot_5/turtlebot_5');
vicon_data_5 = receive(vicon_sub_5, 1);
vicon_data_5.Transform.Translation.X
vicon_data_5.Transform.Translation.Y
% vicon_data_5.Transform.Rotation.Z

y = 'bot 3'
vicon_sub_3 = rossubscriber('/vicon/turtlebot_3/turtlebot_3');
vicon_data_3 = receive(vicon_sub_3, 1);
vicon_data_3.Transform.Translation.X
vicon_data_3.Transform.Translation.Y
% vicon_data_3.Transform.Rotation.Z

rosshutdown
